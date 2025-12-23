/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Driver for TDK InvenSense ICP-20100 barometric pressure and temperature
 * sensor.
 *
 * The implementation is inspired by the reference Arduino driver published
 * by TDK InvenSense (pressure.arduino.ICP201XX) and the existing ICP10100
 * Linux IIO driver. The device exposes a register map identical to the
 * ICP201xx family documented by the vendor.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#define ICP20100_REG_DEVICE_ID		0x0C
#define ICP20100_DEVICE_ID		0x63

#define ICP20100_REG_VERSION		0xD3
#define ICP20100_REG_MODE_SELECT	0xC0
#define ICP20100_REG_INTERRUPT_STATUS	0xC1
#define ICP20100_REG_INTERRUPT_MASK	0xC2
#define ICP20100_REG_FIFO_CONFIG	0xC3
#define ICP20100_REG_FIFO_FILL	0xC4
#define ICP20100_REG_DEVICE_STATUS	0xCD
#define ICP20100_REG_FIFO_BASE	0xFA

#define ICP20100_MODE_MEAS_CONFIG_MASK	GENMASK(7, 5)
#define ICP20100_MODE_FORCED_TRIGGER_MASK	BIT(4)
#define ICP20100_MODE_MEAS_MODE_MASK	BIT(3)
#define ICP20100_MODE_POWER_MASK	BIT(2)
#define ICP20100_MODE_FIFO_READOUT_MASK	GENMASK(1, 0)

#define ICP20100_MODE_MEAS_CONFIG_SHIFT	5
#define ICP20100_MODE_MEAS_MODE_SHIFT	3
#define ICP20100_MODE_POWER_SHIFT	2

#define ICP20100_DEVICE_STATUS_MODE_READY	BIT(0)
#define ICP20100_FIFO_FLUSH_MASK		BIT(7)
#define ICP20100_FIFO_LEVEL_MASK		GENMASK(4, 0)

#define ICP20100_OP_MODE0		0
#define ICP20100_FIFO_MODE_PRESS_TEMP	0

#define ICP20100_MEAS_MODE_FORCED	0
#define ICP20100_MEAS_MODE_CONTINUOUS	1

#define ICP20100_MODE_READY_RETRIES	50
#define ICP20100_DATA_READY_RETRIES	20
#define ICP20100_MEAS_DELAY_US	20000
#define ICP20100_DATA_POLL_US	2000

#define ICP20100_FIFO_PACKET_LEN	6

struct icp20100_state {
	struct i2c_client *client;
	struct regulator *vdd;
	struct mutex lock; /* protects on-demand measurements */
};

static int icp20100_read_block(struct icp20100_state *st, u8 reg,
				 u8 *buf, int len)
{
	struct i2c_client *client = st->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return 0;
}

static int icp20100_write_reg(struct icp20100_state *st, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(st->client, reg, val);
}

static int icp20100_read_reg(struct icp20100_state *st, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(st->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int icp20100_wait_mode_ready(struct icp20100_state *st)
{
	unsigned int retries;
	u8 status;
	int ret;

	for (retries = 0; retries < ICP20100_MODE_READY_RETRIES; retries++) {
		ret = icp20100_read_reg(st, ICP20100_REG_DEVICE_STATUS, &status);
		if (ret)
			return ret;

		if (status & ICP20100_DEVICE_STATUS_MODE_READY)
			return 0;

		usleep_range(500, 1000);
	}

	return -ETIMEDOUT;
}

static int icp20100_update_bits(struct icp20100_state *st, u8 reg,
				 u8 mask, u8 value)
{
	u8 tmp;
	int ret;

	if (reg == ICP20100_REG_MODE_SELECT) {
		ret = icp20100_wait_mode_ready(st);
		if (ret)
			return ret;
	}

	ret = icp20100_read_reg(st, reg, &tmp);
	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= value & mask;

	if (reg == ICP20100_REG_MODE_SELECT) {
		ret = icp20100_wait_mode_ready(st);
		if (ret)
			return ret;
	}

	return icp20100_write_reg(st, reg, tmp);
}

static int icp20100_flush_fifo(struct icp20100_state *st)
{
	u8 val;
	int ret;

	ret = icp20100_read_reg(st, ICP20100_REG_FIFO_FILL, &val);
	if (ret)
		return ret;

	val |= ICP20100_FIFO_FLUSH_MASK;
	return icp20100_write_reg(st, ICP20100_REG_FIFO_FILL, val);
}

static int icp20100_clear_interrupts(struct icp20100_state *st)
{
	u8 status;
	int ret;

	ret = icp20100_read_reg(st, ICP20100_REG_INTERRUPT_STATUS, &status);
	if (ret)
		return ret;

	if (!status)
		return 0;

	return icp20100_write_reg(st, ICP20100_REG_INTERRUPT_STATUS, status);
}

static int icp20100_soft_reset(struct icp20100_state *st)
{
	int ret;

	ret = icp20100_write_reg(st, ICP20100_REG_MODE_SELECT, 0);
	if (ret)
		return ret;

	usleep_range(2000, 2500);

	ret = icp20100_flush_fifo(st);
	if (ret)
		return ret;

	ret = icp20100_write_reg(st, ICP20100_REG_FIFO_CONFIG, 0x00);
	if (ret)
		return ret;

	ret = icp20100_write_reg(st, ICP20100_REG_INTERRUPT_MASK, 0xFF);
	if (ret)
		return ret;

	return icp20100_clear_interrupts(st);
}

static int icp20100_config(struct icp20100_state *st)
{
	int ret;

	ret = icp20100_soft_reset(st);
	if (ret)
		return ret;

	ret = icp20100_update_bits(st, ICP20100_REG_MODE_SELECT,
				ICP20100_MODE_FIFO_READOUT_MASK,
				ICP20100_FIFO_MODE_PRESS_TEMP);
	if (ret)
		return ret;

	ret = icp20100_update_bits(st, ICP20100_REG_MODE_SELECT,
				ICP20100_MODE_MEAS_CONFIG_MASK,
				ICP20100_OP_MODE0 <<
				ICP20100_MODE_MEAS_CONFIG_SHIFT);
	if (ret)
		return ret;

	ret = icp20100_update_bits(st, ICP20100_REG_MODE_SELECT,
				ICP20100_MODE_POWER_MASK,
				ICP20100_MEAS_MODE_FORCED <<
				ICP20100_MODE_POWER_SHIFT);
	if (ret)
		return ret;

	ret = icp20100_update_bits(st, ICP20100_REG_MODE_SELECT,
				ICP20100_MODE_MEAS_MODE_MASK,
				ICP20100_MEAS_MODE_CONTINUOUS <<
				ICP20100_MODE_MEAS_MODE_SHIFT);
	if (ret)
		return ret;

	/* final flush to start from a clean FIFO */
	return icp20100_flush_fifo(st);
}

static int icp20100_trigger_measurement(struct icp20100_state *st)
{
	return icp20100_update_bits(st, ICP20100_REG_MODE_SELECT,
			       ICP20100_MODE_FORCED_TRIGGER_MASK,
			       ICP20100_MODE_FORCED_TRIGGER_MASK);
}

static int icp20100_get_fifo_level(struct icp20100_state *st)
{
	u8 val;
	int ret;

	ret = icp20100_read_reg(st, ICP20100_REG_FIFO_FILL, &val);
	if (ret)
		return ret;

	return val & ICP20100_FIFO_LEVEL_MASK;
}

static int icp20100_read_fifo(struct icp20100_state *st, u8 *buf)
{
	return icp20100_read_block(st, ICP20100_REG_FIFO_BASE, buf,
				ICP20100_FIFO_PACKET_LEN);
}

static int icp20100_wait_data(struct icp20100_state *st)
{
	unsigned int retries;
	int ret;

	for (retries = 0; retries < ICP20100_DATA_READY_RETRIES; retries++) {
		ret = icp20100_get_fifo_level(st);
		if (ret < 0)
			return ret;
		if (ret > 0)
			return 0;
		usleep_range(ICP20100_DATA_POLL_US,
			    ICP20100_DATA_POLL_US + 500);
	}

	return -ETIMEDOUT;
}

static int icp20100_read_measurement(struct icp20100_state *st,
				       int *pressure_pa, int *temperature_mdeg)
{
	s32 raw_pressure, raw_temp;
	s64 pressure;
	s64 temperature;
	u8 data[ICP20100_FIFO_PACKET_LEN];
	int ret;

	ret = icp20100_trigger_measurement(st);
	if (ret)
		return ret;

	usleep_range(ICP20100_MEAS_DELAY_US,
		    ICP20100_MEAS_DELAY_US + 2000);

	ret = icp20100_wait_data(st);
	if (ret)
		return ret;

	ret = icp20100_read_fifo(st, data);
	if (ret)
		return ret;

	raw_pressure = data[0] | (data[1] << 8) |
			((data[2] & 0x0F) << 16);
	if (raw_pressure & BIT(19))
		raw_pressure |= GENMASK(31, 20);

	raw_temp = data[3] | (data[4] << 8) |
		  ((data[5] & 0x0F) << 16);
	if (raw_temp & BIT(19))
		raw_temp |= GENMASK(31, 20);

	pressure = div64_s64((s64)raw_pressure * 40000LL, 131072);
	pressure += 70000LL;
	*pressure_pa = pressure;

	temperature = div64_s64((s64)raw_temp * 65000LL, 262144);
	temperature += 25000LL;
	*temperature_mdeg = temperature;

	icp20100_clear_interrupts(st);
	icp20100_flush_fifo(st);

	return 0;
}

static int icp20100_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct icp20100_state *st = iio_priv(indio_dev);
	int pressure_pa, temperature_mdeg;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		mutex_lock(&st->lock);
		ret = icp20100_read_measurement(st, &pressure_pa,
					     &temperature_mdeg);
		mutex_unlock(&st->lock);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;

		if (chan->type == IIO_PRESSURE) {
			*val = pressure_pa / 1000;
			*val2 = (pressure_pa % 1000) * 1000;
			return IIO_VAL_INT_PLUS_MICRO;
		}

		if (chan->type == IIO_TEMP) {
			*val = temperature_mdeg / 1000;
			*val2 = (temperature_mdeg % 1000) * 1000;
			return IIO_VAL_INT_PLUS_MICRO;
		}

		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info icp20100_info = {
	.read_raw = icp20100_read_raw,
};

static const struct iio_chan_spec icp20100_channels[] = {
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int icp20100_enable_regulator(struct icp20100_state *st)
{
	int ret;

	ret = regulator_enable(st->vdd);
	if (ret)
		return ret;

	msleep(100);
	return 0;
}

static void icp20100_disable_regulator_action(void *data)
{
	struct icp20100_state *st = data;
	int ret;

	ret = regulator_disable(st->vdd);
	if (ret)
		dev_warn(&st->client->dev,
			 "failed to disable regulator: %d\n", ret);
}

static int icp20100_probe(struct i2c_client *client,
			 const struct i2c_device_id *id_entry)
{
	struct iio_dev *indio_dev;
	struct icp20100_state *st;
	int ret;
	u8 chip_id;

	dev_info(&client->dev, "probe start\n");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	mutex_init(&st->lock);

	st->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(st->vdd)) {
		dev_err(&client->dev, "failed to get vdd regulator: %ld\n",
			PTR_ERR(st->vdd));
		return PTR_ERR(st->vdd);
	}

	ret = icp20100_enable_regulator(st);
	if (ret) {
		dev_err(&client->dev, "failed to enable regulator: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(&client->dev,
					 icp20100_disable_regulator_action, st);
	if (ret)
		return ret;

	ret = icp20100_read_reg(st, ICP20100_REG_DEVICE_ID, &chip_id);
	if (ret) {
		dev_err(&client->dev, "failed to read device id: %d\n", ret);
		return ret;
	}

	if (chip_id != ICP20100_DEVICE_ID) {
		dev_err(&client->dev, "unexpected device id %#x (expected %#x)\n",
			chip_id, ICP20100_DEVICE_ID);
		return -ENODEV;
	}

	dev_info(&client->dev, "device id %#x verified\n", chip_id);

	/* silence unused variable warning when built without legacy id table */
	if (id_entry)
		dev_dbg(&client->dev, "matched via legacy id %s\n",
			 id_entry->name);

	ret = icp20100_config(st);
	if (ret) {
		dev_err(&client->dev, "failed to config device: %d\n", ret);
		return ret;
	}

	indio_dev->info = &icp20100_info;
	indio_dev->channels = icp20100_channels;
	indio_dev->num_channels = ARRAY_SIZE(icp20100_channels);
	indio_dev->name = "icp20100";
	indio_dev->modes = INDIO_DIRECT_MODE;

	pm_runtime_enable(&client->dev);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register IIO device: %d\n", ret);
		pm_runtime_disable(&client->dev);
		return ret;
	}

	dev_info(&client->dev, "probe successful\n");
	return 0;
}

static int icp20100_remove(struct i2c_client *client)
{
	pm_runtime_disable(&client->dev);
	return 0;
}

static const struct of_device_id icp20100_of_match[] = {
	{ .compatible = "invensense,icp20100" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, icp20100_of_match);

static const struct i2c_device_id icp20100_id[] = {
	{ "icp20100", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, icp20100_id);

static struct i2c_driver icp20100_driver = {
	.driver = {
		.name = "icp20100",
		.of_match_table = icp20100_of_match,
	},
	.probe = icp20100_probe,
	.remove = icp20100_remove,
	.id_table = icp20100_id,
};
module_i2c_driver(icp20100_driver);

MODULE_AUTHOR("InvenSense porters");
MODULE_DESCRIPTION("TDK InvenSense ICP-20100 pressure sensor driver");
MODULE_LICENSE("GPL");
