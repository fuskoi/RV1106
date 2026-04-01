// SPDX-License-Identifier: GPL-2.0-only
/*
 * Touchkey driver for Freescale MPR121 Controllor
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 * Author: Zhang Jiejing <jiejing.zhang@freescale.com>
 *
 * Based on mcs_touchkey.c
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/* Register definitions */
#define ELE_TOUCH_STATUS_0_ADDR	0x0
#define ELE_TOUCH_STATUS_1_ADDR	0X1
#define MHD_RISING_ADDR		0x2b
#define NHD_RISING_ADDR		0x2c
#define NCL_RISING_ADDR		0x2d
#define FDL_RISING_ADDR		0x2e
#define MHD_FALLING_ADDR	0x2f
#define NHD_FALLING_ADDR	0x30
#define NCL_FALLING_ADDR	0x31
#define FDL_FALLING_ADDR	0x32
#define ELE0_TOUCH_THRESHOLD_ADDR	0x41
#define ELE0_RELEASE_THRESHOLD_ADDR	0x42
#define AFE_CONF_ADDR			0x5c
#define FILTER_CONF_ADDR		0x5d

/*
 * ELECTRODE_CONF_ADDR: This register configures the number of
 * enabled capacitance sensing inputs and its run/suspend mode.
 */
#define ELECTRODE_CONF_ADDR		0x5e
#define ELECTRODE_CONF_QUICK_CHARGE	0x80
#define AUTO_CONFIG_CTRL_ADDR		0x7b
#define AUTO_CONFIG_USL_ADDR		0x7d
#define AUTO_CONFIG_LSL_ADDR		0x7e
#define AUTO_CONFIG_TL_ADDR		0x7f

/* Threshold of touch/release trigger */
#define TOUCH_THRESHOLD			0x14
#define RELEASE_THRESHOLD		0x0A
/* Masks for touch and release triggers */
#define TOUCH_STATUS_MASK		0xfff
/* MPR121 has 12 keys */
#define MPR121_MAX_KEY_COUNT		12

#define MPR121_MIN_POLL_INTERVAL	10
#define MPR121_MAX_POLL_INTERVAL	200

#define MPR121_STABLE_CNT_TH   1   /* 1=最灵敏，2=平衡，3=更稳 */

struct mpr121_touchkey {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	unsigned int		statusbits;
	unsigned int		keycount;
	u32			keycodes[MPR121_MAX_KEY_COUNT];
	u16 raw_statusbits;                  /* 原始状态（未去抖） */
	u8  stable_cnt[MPR121_MAX_KEY_COUNT];/* 连续稳定计数 */
	u8  last_raw[MPR121_MAX_KEY_COUNT];  /* 上次原始按下(0/1) */
};

struct mpr121_init_register {
	int addr;
	u8 val;
};

static const struct mpr121_init_register init_reg_table[] = {
	{ MHD_RISING_ADDR,	0x1 },
	{ NHD_RISING_ADDR,	0x1 },
	{ MHD_FALLING_ADDR,	0x1 },
	{ NHD_FALLING_ADDR,	0x1 },
	{ NCL_FALLING_ADDR,	0xff },
	{ FDL_FALLING_ADDR,	0x02 },
	{ FILTER_CONF_ADDR,	0x01 },  /* 减少滤波延迟，提高响应速度 */
	{ AFE_CONF_ADDR,	0x09 },  /* 8ms采样间隔，提高响应速度（0x0b=16ms） */
	{ AUTO_CONFIG_CTRL_ADDR, 0x0b },
};

static void mpr121_vdd_supply_disable(void *data)
{
	struct regulator *vdd_supply = data;

	regulator_disable(vdd_supply);
}

static struct regulator *mpr121_vdd_supply_init(struct device *dev)
{
	struct regulator *vdd_supply;
	int err;

	vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(vdd_supply)) {
		dev_err(dev, "failed to get vdd regulator: %ld\n",
			PTR_ERR(vdd_supply));
		return vdd_supply;
	}

	err = regulator_enable(vdd_supply);
	if (err) {
		dev_err(dev, "failed to enable vdd regulator: %d\n", err);
		return ERR_PTR(err);
	}

	err = devm_add_action(dev, mpr121_vdd_supply_disable, vdd_supply);
	if (err) {
		regulator_disable(vdd_supply);
		dev_err(dev, "failed to add disable regulator action: %d\n",
			err);
		return ERR_PTR(err);
	}

	return vdd_supply;
}

static void mpr_touchkey_report(struct input_dev *dev)
{
	struct mpr121_touchkey *mpr121 = input_get_drvdata(dev);
	struct input_dev *input = mpr121->input_dev;
	struct i2c_client *client = mpr121->client;
	unsigned int key_num;
	int reg;

	reg = i2c_smbus_read_byte_data(client, ELE_TOUCH_STATUS_1_ADDR);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		return;
	}

	reg <<= 8;
	reg |= i2c_smbus_read_byte_data(client, ELE_TOUCH_STATUS_0_ADDR);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		return;
	}

	reg &= TOUCH_STATUS_MASK;

	/* 保存原始状态（可用于调试） */
	mpr121->raw_statusbits = reg;

	/* 对每个键做"连续稳定 N 次"过滤 */
	for (key_num = 0; key_num < mpr121->keycount; key_num++) {
		u8 raw_pressed = !!(reg & BIT(key_num));
		u8 stable_pressed = !!(mpr121->statusbits & BIT(key_num));

		/* 状态变化时重置计数器 */
		if (raw_pressed != mpr121->last_raw[key_num]) {
			mpr121->last_raw[key_num] = raw_pressed;
			mpr121->stable_cnt[key_num] = 1;
		} else if (mpr121->stable_cnt[key_num] < MPR121_STABLE_CNT_TH) {
			/* 状态未变化，增加稳定计数 */
			mpr121->stable_cnt[key_num]++;
		}

		/* 状态连续稳定到阈值，且与已上报状态不同时才上报 */
		if (mpr121->stable_cnt[key_num] >= MPR121_STABLE_CNT_TH &&
		    raw_pressed != stable_pressed) {
			unsigned int key_val = mpr121->keycodes[key_num];

			if (raw_pressed)
				mpr121->statusbits |= BIT(key_num);
			else
				mpr121->statusbits &= ~BIT(key_num);

			input_event(input, EV_MSC, MSC_SCAN, key_num);
			input_report_key(input, key_val, raw_pressed);

			dev_dbg(&client->dev, "key %u %u %s (debounced)\n",
				key_num, key_val,
				raw_pressed ? "pressed" : "released");
		}
	}

	input_sync(input);
}


static irqreturn_t mpr_touchkey_interrupt(int irq, void *dev_id)
{
	struct mpr121_touchkey *mpr121 = dev_id;

	mpr_touchkey_report(mpr121->input_dev);

	return IRQ_HANDLED;
}

static int mpr121_phys_init(struct mpr121_touchkey *mpr121,
			    struct i2c_client *client, int vdd_uv)
{
	const struct mpr121_init_register *reg;
	unsigned char usl, lsl, tl, eleconf;
	int i, t, vdd, ret;

	/* Set up touch/release threshold for ele0-ele11 */
	for (i = 0; i < mpr121->keycount; i++) {
		t = ELE0_TOUCH_THRESHOLD_ADDR + (i * 2);
		ret = i2c_smbus_write_byte_data(client, t, TOUCH_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
		ret = i2c_smbus_write_byte_data(client, t + 1, RELEASE_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
	}


	/* Set up init register */
	for (i = 0; i < ARRAY_SIZE(init_reg_table); i++) {
		reg = &init_reg_table[i];
		ret = i2c_smbus_write_byte_data(client, reg->addr, reg->val);
		if (ret < 0)
			goto err_i2c_write;
	}


	/*
	 * Capacitance on sensing input varies and needs to be compensated.
	 * The internal MPR121-auto-configuration can do this if it's
	 * registers are set properly (based on vdd_uv).
	 */
	vdd = vdd_uv / 1000;
	usl = ((vdd - 700) * 256) / vdd;
	lsl = (usl * 65) / 100;
	tl = (usl * 90) / 100;
	ret = i2c_smbus_write_byte_data(client, AUTO_CONFIG_USL_ADDR, usl);
	ret |= i2c_smbus_write_byte_data(client, AUTO_CONFIG_LSL_ADDR, lsl);
	ret |= i2c_smbus_write_byte_data(client, AUTO_CONFIG_TL_ADDR, tl);

	/*
	 * Quick charge bit will let the capacitive charge to ready
	 * state quickly, or the buttons may not function after system
	 * boot.
	 */
	 eleconf = (2 << 5) | ELECTRODE_CONF_QUICK_CHARGE | mpr121->keycount;
	 ret = i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, eleconf);
	 if (ret)
		 dev_err(&client->dev, "Failed to set electrode config with fast baseline load\n");
	if (ret != 0)
		goto err_i2c_write;

	dev_dbg(&client->dev, "set up with %x keys.\n", mpr121->keycount);

	return 0;

err_i2c_write:
	dev_err(&client->dev, "i2c write error: %d\n", ret);
	return ret;
}

static int mpr_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct regulator *vdd_supply;
	int vdd_uv;
	struct mpr121_touchkey *mpr121;
	struct input_dev *input_dev;
	u32 poll_interval = 0;
	int error;
	int i;

	vdd_supply = mpr121_vdd_supply_init(dev);
	if (IS_ERR(vdd_supply))
		return PTR_ERR(vdd_supply);

	vdd_uv = regulator_get_voltage(vdd_supply);

	mpr121 = devm_kzalloc(dev, sizeof(*mpr121), GFP_KERNEL);
	if (!mpr121)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;

	mpr121->client = client;
	mpr121->input_dev = input_dev;
	mpr121->keycount = device_property_count_u32(dev, "linux,keycodes");
	if (mpr121->keycount > MPR121_MAX_KEY_COUNT) {
		dev_err(dev, "too many keys defined (%d)\n", mpr121->keycount);
		return -EINVAL;
	}

	error = device_property_read_u32_array(dev, "linux,keycodes",
					       mpr121->keycodes,
					       mpr121->keycount);
	if (error) {
		dev_err(dev,
			"failed to read linux,keycode property: %d\n", error);
		return error;
	}

	input_dev->name = "Freescale MPR121 Touchkey";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, mpr121);

	input_dev->keycode = mpr121->keycodes;
	input_dev->keycodesize = sizeof(mpr121->keycodes[0]);
	input_dev->keycodemax = mpr121->keycount;

	for (i = 0; i < mpr121->keycount; i++)
		input_set_capability(input_dev, EV_KEY, mpr121->keycodes[i]);

	error = mpr121_phys_init(mpr121, client, vdd_uv);
	if (error) {
		dev_err(dev, "Failed to init register\n");
		return error;
	}

	device_property_read_u32(dev, "poll-interval", &poll_interval);

	if (client->irq) {
		error = devm_request_threaded_irq(dev, client->irq, NULL,
						  mpr_touchkey_interrupt,
						  IRQF_TRIGGER_FALLING |
						  IRQF_ONESHOT,
						  dev->driver->name, mpr121);
		if (error) {
			dev_err(dev, "Failed to register interrupt\n");
			return error;
		}
	} else if (poll_interval) {
		if (poll_interval < MPR121_MIN_POLL_INTERVAL)
			return -EINVAL;

		if (poll_interval > MPR121_MAX_POLL_INTERVAL)
			return -EINVAL;

		error = input_setup_polling(input_dev, mpr_touchkey_report);
		if (error) {
			dev_err(dev, "Failed to setup polling\n");
			return error;
		}

		input_set_poll_interval(input_dev, poll_interval);
		input_set_min_poll_interval(input_dev,
					    MPR121_MIN_POLL_INTERVAL);
		input_set_max_poll_interval(input_dev,
					    MPR121_MAX_POLL_INTERVAL);
	} else {
		dev_err(dev,
			"invalid IRQ number and polling not configured\n");
		return -EINVAL;
	}

	error = input_register_device(input_dev);
	if (error)
		return error;

	i2c_set_clientdata(client, mpr121);
	device_init_wakeup(dev,
			device_property_read_bool(dev, "wakeup-source"));

	return 0;
}

static int __maybe_unused mpr_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, 0x00);

	return 0;
}

static int __maybe_unused mpr_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpr121_touchkey *mpr121 = i2c_get_clientdata(client);
	u8 eleconf;

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	/* 恢复电极配置，需要包含QUICK_CHARGE位 */
	eleconf = mpr121->keycount | ELECTRODE_CONF_QUICK_CHARGE;
	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, eleconf);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mpr121_touchkey_pm_ops, mpr_suspend, mpr_resume);

static const struct i2c_device_id mpr121_id[] = {
	{ "mpr121_touchkey", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpr121_id);

#ifdef CONFIG_OF
static const struct of_device_id mpr121_touchkey_dt_match_table[] = {
	{ .compatible = "fsl,mpr121-touchkey" },
	{ },
};
MODULE_DEVICE_TABLE(of, mpr121_touchkey_dt_match_table);
#endif

static struct i2c_driver mpr_touchkey_driver = {
	.driver = {
		.name	= "mpr121",
		.pm	= &mpr121_touchkey_pm_ops,
		.of_match_table = of_match_ptr(mpr121_touchkey_dt_match_table),
	},
	.id_table	= mpr121_id,
	.probe		= mpr_touchkey_probe,
};

module_i2c_driver(mpr_touchkey_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Jiejing <jiejing.zhang@freescale.com>");
MODULE_DESCRIPTION("Touch Key driver for Freescale MPR121 Chip");
