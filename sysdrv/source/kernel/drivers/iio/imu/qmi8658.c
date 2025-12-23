
#include<linux/module.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>
#include<linux/slab.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#include<linux/of_device.h>
#include<linux/of_address.h>
#include<linux/of_gpio.h>

#include<linux/wakelock.h>
#include<linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/device.h>
#include "qmi8658.h"


struct Qmi8658_Data
{
	unsigned char				chip_id;
	struct i2c_client 			*client;
	atomic_t 					position;

	atomic_t 					acc_enable;
	atomic_t 					acc_delay;	
	atomic_t 					gyro_enable;
	atomic_t 					gyro_delay;
	Qmi8658_Acc					acc_out;
	Qmi8658_Gyro				gyro_out;

	struct input_dev 			*acc_input;
	struct input_dev 			*gyro_input;
	#if 0
	struct sensors_classdev 	accel_cdev;
	struct sensors_classdev 	gyro_cdev;
	#endif

	struct delayed_work 		acc_work;
	struct delayed_work 		gyro_work;
	
	struct mutex 				op_mutex;
	
	char						calibrate_buf[QMI8658_CAL_NUM];
	int							cal_params[3];
	bool						use_cal;
	atomic_t					cal_status;
	
	// 陀螺仪零点偏移（原始值）
	short						gyro_bias[3];
	bool						gyro_calibrated;

	short 						acc_lsb;
	short						acc_scale;
	enum Qmi8658_AccRange		acc_range;
	enum Qmi8658_AccOdr			acc_odr;
	enum Qmi8658_AccUnit		acc_uint;
	short 						gyro_lsb;
	short						gyro_scale;
	enum Qmi8658_GyrRange		gyro_range;	
	enum Qmi8658_GyrOdr			gyro_odr;
	enum Qmi8658_GyrUnit		gyro_uint;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_drv;
#endif 
};

#if defined(ACC_USE_CALI)
#define ACC_CALI_FILE		"/productinfo/acc_cali.conf"
#define ACC_LSB_1G			1000			// mg
#define ACC_CALI_NUM		20    
static int acc_cali[3]={0, 0, 0};
static char acc_cali_flag = 0;
static void acc_read_file(char * filename, char *data, int len);
static void acc_write_file(char * filename, char *data, int len);
#endif

static struct Qmi8658_convert g_map;
static struct Qmi8658_Data *g_qmi8658=NULL;

#if 0
static struct sensors_classdev qmi8658_acc_cdev = {
	.name = "accelerometer",
	.vendor = "QST Corporation",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,	
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "2048",
	.resolution = "0.6",
	.sensor_power = "0.2",
	.min_delay = 10 * 1000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev qmi8658_gyro_cdev = {
	.name = "gyroscope",
	.vendor = "QST Corporation",
	.version = 1,	
	.handle = SENSORS_GYROSCOPE_HANDLE,	
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "65536",	
	.resolution = "1",	
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = 10 * 1000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

void qmi8658_initialize(struct i2c_client *client);


/**
 * 写入QMI8658寄存器
 * @param reg 寄存器地址
 * @param value 写入值
 * @return 0成功，负数失败
 */
static int qmi8658_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
	
	if (!g_qmi8658 || !g_qmi8658->client) {
		QMI8658_ERR("qmi8658_write_reg: g_qmi8658 or client is NULL\n");
		return -EINVAL;
	}
	
	mutex_lock(&g_qmi8658->op_mutex);
	ret = i2c_smbus_write_byte_data(g_qmi8658->client, reg, value);
	mutex_unlock(&g_qmi8658->op_mutex);
	if(ret < 0)
	{
		QMI8658_ERR("write reg 0x%02x value 0x%02x failed: %d\n", reg, value, ret);
		return ret;
	}
	
	return 0;
}

/**
 * 读取QMI8658寄存器
 * @param reg 寄存器地址
 * @param buf 数据缓冲区
 * @param len 读取长度
 * @return 0成功，负数失败
 */
static int qmi8658_read_reg(unsigned char reg, unsigned char *buf, unsigned char len)
{
	int ret = 0;
	
	if (!g_qmi8658 || !g_qmi8658->client) {
		QMI8658_ERR("qmi8658_read_reg: g_qmi8658 or client is NULL\n");
		return -EINVAL;
	}
	
	mutex_lock(&g_qmi8658->op_mutex);
	if(len == 1) {
		ret = i2c_smbus_read_byte_data(g_qmi8658->client, reg);
		if (ret < 0) {
			mutex_unlock(&g_qmi8658->op_mutex);
			QMI8658_ERR("read reg 0x%02x failed: %d\n", reg, ret);
			return ret;
		}
		*buf = (unsigned char)ret;
		ret = 0;
	} else {
		ret = i2c_smbus_read_i2c_block_data(g_qmi8658->client, reg, len, buf);
		if (ret < 0) {
			mutex_unlock(&g_qmi8658->op_mutex);
			QMI8658_ERR("read reg 0x%02x block failed: %d\n", reg, ret);
			return ret;
		}
	}
	mutex_unlock(&g_qmi8658->op_mutex);
	
	return 0;
}


static int qmi8658_read_chip_id(struct i2c_client *client)
{
	int res = 0;
	unsigned char chip_id;
	unsigned char i2c_addr[2] = {QMI8658_I2C_SLAVE_ADDR_L, QMI8658_I2C_SLAVE_ADDR_H};
	int i = 0;

	QMI8658_FUN();
	while(i < 2)
	{
		client->addr = i2c_addr[i];
		res = qmi8658_read_reg(Qmi8658Register_WhoAmI, &chip_id, 1);
		if(res)
		{
			QMI8658_ERR("qmi8658_read_chip_id error!");
		}
		else
		{
			QMI8658_ERR("qmi8658_read_chip_id OK chip_id=0x%x ", chip_id);
			g_qmi8658->chip_id = chip_id;
		}
		if(g_qmi8658->chip_id == QMI8658_CHIP_ID)
		{
			break;
		}
		i++;
	}

	if(g_qmi8658->chip_id == QMI8658_CHIP_ID)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60);
		return 0;
	}
	else
	{
		return -1;
	}
}

/**
 * 配置加速度计参数
 * @param range 量程
 * @param odr 输出数据率
 * @param lpf 低通滤波器配置
 * @param stEnable 自检使能
 * @return 0成功，负数失败
 */
static int qmi8658_config_acc(enum Qmi8658_AccRange range, enum Qmi8658_AccOdr odr, enum Qmi8658_LpfConfig lpf, enum Qmi8658_StConfig stEnable)
{
	u8 acc_setting = 0;
	int err=0;

	switch(range)
	{
		case Qmi8658AccRange_2g:
			// 2g量程：±2g对应±32768，1g=16384 LSB，转换公式：mg = (raw * 2000) / 16384
			g_qmi8658->acc_lsb = 16384;  // 1g对应的LSB数
			g_qmi8658->acc_scale = 2000;  // 2g量程，单位mg
			break;
		case Qmi8658AccRange_4g:
			// 4g量程：±4g对应±32768，1g=8192 LSB
			g_qmi8658->acc_lsb = 8192;   // 1g对应的LSB数
			g_qmi8658->acc_scale = 4000;  // 4g量程，单位mg
			break;
		case Qmi8658AccRange_8g:
			// 8g量程：±8g对应±32768，1g=4096 LSB
			g_qmi8658->acc_lsb = 4096;   // 1g对应的LSB数
			g_qmi8658->acc_scale = 8000;  // 8g量程，单位mg
			break;
		case Qmi8658AccRange_16g:
			// 16g量程：±16g对应±32768，1g=2048 LSB
			g_qmi8658->acc_lsb = 2048;   // 1g对应的LSB数
			g_qmi8658->acc_scale = 16000; // 16g量程，单位mg
			break;
		default:			
			g_qmi8658->acc_lsb = 16384;
			g_qmi8658->acc_scale = 2000;
			break;
	}
	g_qmi8658->acc_range = range;
	g_qmi8658->acc_odr = odr;
// range & odr
	if(stEnable == Qmi8658St_Enable)
		acc_setting = (u8)range|(u8)odr|0x80;
	else
		acc_setting = (u8)range|(u8)odr;

	err = qmi8658_write_reg(Qmi8658Register_Ctrl2, acc_setting);
// lpf & St
	err = qmi8658_read_reg(Qmi8658Register_Ctrl5, &acc_setting, 1);
	acc_setting &= 0xfc;
	if(lpf == Qmi8658Lpf_Enable)
	{
		acc_setting |= A_LSP_MODE_3;
		acc_setting |= 0x01;
	}
	else
	{
		acc_setting &= ~0x01;
	}
	err = qmi8658_write_reg(Qmi8658Register_Ctrl5, acc_setting);

	return err;
}

/**
 * 配置陀螺仪参数
 * @param range 量程
 * @param odr 输出数据率
 * @param lpf 低通滤波器配置
 * @param stEnable 自检使能
 * @return 0成功，负数失败
 */
static int qmi8658_config_gyro(enum Qmi8658_GyrRange range, enum Qmi8658_GyrOdr odr, enum Qmi8658_LpfConfig lpf, enum Qmi8658_StConfig stEnable)
{
	u8 gyro_setting = 0;
	int err=0;

	switch(range)
	{
		case Qmi8658GyrRange_32dps:
			// 32dps量程：±32dps对应±32768，1dps=1024 LSB
			g_qmi8658->gyro_lsb = 1024;   // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 32;    // 32dps量程，单位dps
			break;
		case Qmi8658GyrRange_64dps:
			// 64dps量程：±64dps对应±32768，1dps=512 LSB
			g_qmi8658->gyro_lsb = 512;    // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 64;    // 64dps量程，单位dps
			break;
		case Qmi8658GyrRange_128dps:
			// 128dps量程：±128dps对应±32768，1dps=256 LSB
			g_qmi8658->gyro_lsb = 256;    // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 128;   // 128dps量程，单位dps
			break;
		case Qmi8658GyrRange_256dps:
			// 256dps量程：±256dps对应±32768，1dps=128 LSB
			g_qmi8658->gyro_lsb = 128;    // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 256;   // 256dps量程，单位dps
			break;
		case Qmi8658GyrRange_512dps:
			// 512dps量程：±512dps对应±32768，1dps=64 LSB
			g_qmi8658->gyro_lsb = 64;     // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 512;   // 512dps量程，单位dps
			break;
		case Qmi8658GyrRange_1024dps:
			// 1024dps量程：±1024dps对应±32768，1dps=32 LSB
			g_qmi8658->gyro_lsb = 32;     // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 1024;  // 1024dps量程，单位dps
			break;
		case Qmi8658GyrRange_2048dps:
			// 2048dps量程：±2048dps对应±32768，1dps=16 LSB
			g_qmi8658->gyro_lsb = 16;     // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 2048;  // 2048dps量程，单位dps
			break;
		case Qmi8658GyrRange_4096dps:
			// 4096dps量程：±4096dps对应±32768，1dps=8 LSB
			g_qmi8658->gyro_lsb = 8;      // 1dps对应的LSB数
			g_qmi8658->gyro_scale = 4096; // 4096dps量程，单位dps
			break;
		default:
			g_qmi8658->gyro_lsb = 32;
			g_qmi8658->gyro_scale = 1024;
	}
	g_qmi8658->gyro_range = range;
	g_qmi8658->gyro_odr = odr;

	// range & odr
	if(stEnable == Qmi8658St_Enable)
		gyro_setting = (uint8_t)range|(uint8_t)odr|0x80;
	else
		gyro_setting = (uint8_t)range|(uint8_t)odr;
	err = qmi8658_write_reg(Qmi8658Register_Ctrl3, gyro_setting);

	// lpf & hpf
	err = qmi8658_read_reg(Qmi8658Register_Ctrl5, &gyro_setting, 1);
	gyro_setting &= 0x0f;
	if(lpf == Qmi8658Lpf_Enable)
	{
		gyro_setting |= G_LSP_MODE_3;
		gyro_setting |= 0x10;
	}
	else
	{
		gyro_setting &= ~0x10;
	}
	err = qmi8658_write_reg(Qmi8658Register_Ctrl5, gyro_setting);

	return err;
}

/**
 * 使能或禁用传感器
 * @param type 传感器类型（ACC、GYRO或组合）
 * @param enableFlags 1使能，0禁用
 */
void qmi8658_enable_sensors(enum Qmi8658_type type, u8 enableFlags)
{
	unsigned char ctrl7_reg = 0;
	int ret;

	if (!g_qmi8658) {
		QMI8658_ERR("qmi8658_enable_sensors: g_qmi8658 is NULL\n");
		return;
	}

	ret = qmi8658_read_reg(Qmi8658Register_Ctrl7, &ctrl7_reg, 1);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_enable_sensors: read Ctrl7 failed\n");
		return;
	}

	QMI8658_LOG("qmi8658_enable_sensors type=0x%02x enableFlags=%d, ctrl7_reg=0x%02x\n", 
	            type, enableFlags, ctrl7_reg);

	if(type & QMI8658_TYPE_ACC)
	{
		if(enableFlags)
			ctrl7_reg |= QMI8658_CTRL7_ACC_ENABLE;
		else
			ctrl7_reg &= ~QMI8658_CTRL7_ACC_ENABLE;
	}
	if(type & QMI8658_TYPE_GYRO)
	{
		if(enableFlags)
			ctrl7_reg |= QMI8658_CTRL7_GYR_ENABLE;
		else
			ctrl7_reg &= ~QMI8658_CTRL7_GYR_ENABLE;
	}
	
	ret = qmi8658_write_reg(Qmi8658Register_Ctrl7, ctrl7_reg);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_enable_sensors: write Ctrl7 failed\n");
		return;
	}

	QMI8658_LOG("qmi8658_enable_sensors: new ctrl7_reg=0x%02x\n", ctrl7_reg);

	if(enableFlags)
	{
		// 使能后需要等待传感器稳定
		mdelay(100);
	}
}

#if 0
static int qmi8658_set_mode(enum Qmi8658_mode mode)
{
	if(mode == QMI8658_MODE_LOW_POWER)
	{
		qmi8658_config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_LowPower_3Hz, Lpf_Disable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 0);
		mdelay(5);
	}
	else if(mode == QMI8658_MODE_POWER_DOWN)
	{
		qmi8658_config_acc(Qmi8658AccRange_2g, Qmi8658AccOdr_250Hz, Lpf_Disable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 0);		
		mdelay(5);
		qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x01);
		mdelay(5);
	}
	else if(mode == QMI8658_MODE_POWERON_DEFAULT)
	{		
		qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 0);
		mdelay(5);
	}
	else if(mode == QMI8658_MODE_ACCGYRO_ENABLE)
	{
		//qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x00);
		//mdelay(5);
		qmi8658_config_acc(g_qmi8658->acc_range, g_qmi8658->acc_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_config_gyro(g_qmi8658->gyro_range, g_qmi8658->gyro_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 1);		
		mdelay(100);
	}
	else if(mode == QMI8658_MODE_ACC_ENABLE)
	{	
		qmi8658_config_acc(g_qmi8658->acc_range, g_qmi8658->acc_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_config_gyro(g_qmi8658->gyro_range, g_qmi8658->gyro_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_ACC, 1);		
		mdelay(5);
	}
	else if(mode == QMI8658_MODE_GYRO_ENABLE)
	{	
		qmi8658_config_acc(g_qmi8658->acc_range, g_qmi8658->acc_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_config_gyro(g_qmi8658->gyro_range, g_qmi8658->gyro_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_GYRO, 1);		
		mdelay(5);
	}

	return 0;
}
#endif

void qmi8658_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 1)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 2)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 3)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}	
	else if(layout == 4)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 5)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 6)
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
	else if(layout == 7)
	{
		g_map.sign[axis_x] = -1;
		g_map.sign[axis_y] = -1;
		g_map.sign[axis_z] = -1;
		g_map.map[axis_x] = axis_y;
		g_map.map[axis_y] = axis_x;
		g_map.map[axis_z] = axis_z;
	}
	else		
	{
		g_map.sign[axis_x] = 1;
		g_map.sign[axis_y] = 1;
		g_map.sign[axis_z] = 1;
		g_map.map[axis_x] = axis_x;
		g_map.map[axis_y] = axis_y;
		g_map.map[axis_z] = axis_z;
	}
}

#if 0
static void qmi8658_set_delay(enum Qmi8658_type type, int delay)
{
	int delay_last;
	int is_enable;

	QMI8658_FUN();
	if(type == QMI8658_TYPE_ACC)
	{
		delay_last = atomic_read(&g_qmi8658->acc_delay);
		is_enable = atomic_read(&g_qmi8658->acc_enable);
		atomic_set(&g_qmi8658->acc_delay, delay);
		if((delay_last != delay)&&(is_enable))
		{
			cancel_delayed_work_sync(&g_qmi8658->acc_work);
			schedule_delayed_work(&g_qmi8658->acc_work, msecs_to_jiffies(delay)+1);
		}
	}
	else if(type == QMI8658_TYPE_GYRO)
	{
		delay_last = atomic_read(&g_qmi8658->gyro_delay);
		is_enable = atomic_read(&g_qmi8658->gyro_enable);
		atomic_set(&g_qmi8658->gyro_delay, delay);
		if((delay_last != delay)&&(is_enable))
		{
			cancel_delayed_work_sync(&g_qmi8658->gyro_work);
			schedule_delayed_work(&g_qmi8658->gyro_work, msecs_to_jiffies(delay)+1);
		}
	}
}
#endif

/**
 * 读取加速度计原始数据
 * @param raw_xyz 输出原始数据数组
 * @return 0成功，负数失败
 */
static int qmi8658_acc_read_raw(short raw_xyz[3])
{
	unsigned char buf_reg[6];
	unsigned char status0 = 0;
	int ret = 0;
	int i;

	// 检查状态寄存器，看数据是否准备好（可选，某些芯片需要）
	ret = qmi8658_read_reg(Qmi8658Register_Status0, &status0, 1);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_acc_read_raw: read Status0 failed: %d\n", ret);
		// 继续尝试读取数据
	}

	ret = qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 6);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_acc_read_raw failed: %d\n", ret);
		return ret;
	}

	// 打印原始寄存器值用于调试
	QMI8658_LOG("qmi8658_acc_read_raw reg[0x%02x-0x%02x]: ", 
	            Qmi8658Register_Ax_L, Qmi8658Register_Az_H);
	for (i = 0; i < 6; i++) {
		QMI8658_LOG("0x%02x ", buf_reg[i]);
	}
	QMI8658_LOG("status0=0x%02x\n", status0);

	// 小端格式：低字节在前，高字节在后
	raw_xyz[0] = (short)((buf_reg[1] << 8) | buf_reg[0]);
	raw_xyz[1] = (short)((buf_reg[3] << 8) | buf_reg[2]);
	raw_xyz[2] = (short)((buf_reg[5] << 8) | buf_reg[4]);
	
	QMI8658_LOG("qmi8658_acc_read_raw: %d %d %d\n", raw_xyz[0], raw_xyz[1], raw_xyz[2]);

	return 0;
}


/**
 * 读取加速度计数据并转换为物理单位
 * @param acc 输出加速度数据（单位：mg）
 * @return 0成功，负数失败
 */
static int qmi8658_read_accel_xyz(Qmi8658_Acc *acc)
{
	int res = 0;
	short raw_xyz[3];
	int acc_xyz[3];

	if (!acc) {
		QMI8658_ERR("qmi8658_read_accel_xyz: acc is NULL\n");
		return -EINVAL;
	}

	res = qmi8658_acc_read_raw(raw_xyz);
	if (res < 0) {
		return res;
	}

	// 坐标重映射
	acc_xyz[g_map.map[0]] = g_map.sign[0] * raw_xyz[0];
	acc_xyz[g_map.map[1]] = g_map.sign[1] * raw_xyz[1];
	acc_xyz[g_map.map[2]] = g_map.sign[2] * raw_xyz[2];
	
	// 转换为物理单位（mg）
	// 公式：物理值 = (原始值 * 量程) / 32768
	// 对于8g量程：mg = (raw * 8000) / 32768
	if (g_qmi8658->acc_lsb > 0) {
		// 使用标准16位满量程32768进行转换
		acc->x = (acc_xyz[0] * g_qmi8658->acc_scale) / 32768;
		acc->y = (acc_xyz[1] * g_qmi8658->acc_scale) / 32768;
		acc->z = (acc_xyz[2] * g_qmi8658->acc_scale) / 32768;
	} else {
		QMI8658_ERR("acc_lsb is zero!\n");
		return -EINVAL;
	}

	return 0;
}


/**
 * 读取陀螺仪原始数据
 * @param raw_xyz 输出原始数据数组
 * @return 0成功，负数失败
 */
static int qmi8658_gyro_read_raw(short raw_xyz[3])
{
	unsigned char buf_reg[6];
	unsigned char status0 = 0;
	int ret = 0;
	int i;

	// 检查状态寄存器，看数据是否准备好（可选，某些芯片需要）
	ret = qmi8658_read_reg(Qmi8658Register_Status0, &status0, 1);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_gyro_read_raw: read Status0 failed: %d\n", ret);
		// 继续尝试读取数据
	}

	ret = qmi8658_read_reg(Qmi8658Register_Gx_L, buf_reg, 6);
	if (ret < 0) {
		QMI8658_ERR("qmi8658_gyro_read_raw failed: %d\n", ret);
		return ret;
	}

	// 打印原始寄存器值用于调试
	QMI8658_LOG("qmi8658_gyro_read_raw reg[0x%02x-0x%02x]: ", 
	            Qmi8658Register_Gx_L, Qmi8658Register_Gz_H);
	for (i = 0; i < 6; i++) {
		QMI8658_LOG("0x%02x ", buf_reg[i]);
	}
	QMI8658_LOG("status0=0x%02x\n", status0);

	// 小端格式：低字节在前，高字节在后
	raw_xyz[0] = (short)((buf_reg[1] << 8) | buf_reg[0]);
	raw_xyz[1] = (short)((buf_reg[3] << 8) | buf_reg[2]);
	raw_xyz[2] = (short)((buf_reg[5] << 8) | buf_reg[4]);

	QMI8658_LOG("qmi8658_gyro_read_raw: %d %d %d\n", raw_xyz[0], raw_xyz[1], raw_xyz[2]);

	return 0;
}

/**
 * 读取陀螺仪数据并转换为物理单位
 * @param gyro 输出陀螺仪数据（单位：dps）
 * @return 0成功，负数失败
 */
static int qmi8658_read_gyro_xyz(Qmi8658_Gyro *gyro)
{
	int res = 0;
	short raw_xyz[3];
	int gyro_xyz[3];

	if (!gyro) {
		QMI8658_ERR("qmi8658_read_gyro_xyz: gyro is NULL\n");
		return -EINVAL;
	}

	res = qmi8658_gyro_read_raw(raw_xyz);
	if (res < 0) {
		return res;
	}

	// 减去零点偏移（在原始数据上应用，如果已校准）
	if (g_qmi8658->gyro_calibrated) {
		raw_xyz[0] -= g_qmi8658->gyro_bias[0];
		raw_xyz[1] -= g_qmi8658->gyro_bias[1];
		raw_xyz[2] -= g_qmi8658->gyro_bias[2];
	}

	// 坐标重映射（在减去偏移后进行）
	gyro_xyz[g_map.map[0]] = g_map.sign[0] * raw_xyz[0];
	gyro_xyz[g_map.map[1]] = g_map.sign[1] * raw_xyz[1];
	gyro_xyz[g_map.map[2]] = g_map.sign[2] * raw_xyz[2];
	
	// 转换为物理单位（dps）
	// 公式：物理值 = (原始值 * 量程) / 32768
	// 对于1024dps量程：dps = (raw * 1024) / 32768
	if (g_qmi8658->gyro_lsb > 0) {
		// 使用标准16位满量程32768进行转换
		gyro->x = (gyro_xyz[0] * g_qmi8658->gyro_scale) / 32768;
		gyro->y = (gyro_xyz[1] * g_qmi8658->gyro_scale) / 32768;
		gyro->z = (gyro_xyz[2] * g_qmi8658->gyro_scale) / 32768;
	} else {
		QMI8658_ERR("gyro_lsb is zero!\n");
		return -EINVAL;
	}

	return 0;
}


/**
 * 加速度计工作队列处理函数
 * 周期性读取加速度计数据并上报到input子系统
 */
static void acc_work_func(struct work_struct *work)
{
    Qmi8658_Acc acc;
    int comres;
    int retry;
    int enable;

    QMI8658_FUN();

    if (!g_qmi8658 || !g_qmi8658->acc_input) {
        QMI8658_ERR("acc_work_func: g_qmi8658 or acc_input is NULL\n");
        return;
    }

    enable = atomic_read(&g_qmi8658->acc_enable);
    if (!enable) {
        // 如果加速度计未使能，取消调度
        return;
    }

    /* 初始化变量 */
    acc.x = 0;
    acc.y = 0;
    acc.z = 0;
    comres = -1;
    retry = 0;

    // 重试读取数据，最多3次
    while (comres && (retry++ < 3)) {
        comres = qmi8658_read_accel_xyz(&acc);
        if (comres) {
            msleep(1);  // 读取失败时稍作延迟
        }
    }

    if (comres) {
        // 读取失败，使用上次的数据
        acc.x = g_qmi8658->acc_out.x;
        acc.y = g_qmi8658->acc_out.y;
        acc.z = g_qmi8658->acc_out.z;
        QMI8658_ERR("acc_work_func read data fail, retry=%d\n", retry);
    } else {
        // 读取成功，更新缓存
        g_qmi8658->acc_out.x = acc.x;
        g_qmi8658->acc_out.y = acc.y;
        g_qmi8658->acc_out.z = acc.z;
    }

    // 上报数据到input子系统
    input_report_abs(g_qmi8658->acc_input, ABS_X, acc.x);
    input_report_abs(g_qmi8658->acc_input, ABS_Y, acc.y);
    input_report_abs(g_qmi8658->acc_input, ABS_Z, acc.z);
    input_report_abs(g_qmi8658->acc_input, ABS_THROTTLE, 3);
    input_sync(g_qmi8658->acc_input);

    QMI8658_LOG("%s: [%d %d %d] mg\n", __func__, acc.x, acc.y, acc.z);

    // 继续调度下一次读取
    queue_delayed_work(system_wq, &g_qmi8658->acc_work, msecs_to_jiffies(atomic_read(&g_qmi8658->acc_delay)));

}



/**
 * 陀螺仪工作队列处理函数
 * 周期性读取陀螺仪数据并上报到input子系统
 */
static void gyro_work_func(struct work_struct *work)
{
    Qmi8658_Gyro gyro;
    int comres;
    int retry;
    int enable;

    QMI8658_FUN();

    if (!g_qmi8658 || !g_qmi8658->gyro_input) {
        QMI8658_ERR("gyro_work_func: g_qmi8658 or gyro_input is NULL\n");
        return;
    }

    enable = atomic_read(&g_qmi8658->gyro_enable);
    if (!enable) {
        // 如果陀螺仪未使能，取消调度
        return;
    }

    gyro.x = 0;
    gyro.y = 0;
    gyro.z = 0;
    comres = -1;
    retry = 0;

    // 重试读取数据，最多3次
    while (comres && (retry++ < 3)) {
        comres = qmi8658_read_gyro_xyz(&gyro);
        if (comres) {
            msleep(1);  // 读取失败时稍作延迟
        }
    }

    if (comres) {
        // 读取失败，使用上次的数据
        gyro.x = g_qmi8658->gyro_out.x;
        gyro.y = g_qmi8658->gyro_out.y;
        gyro.z = g_qmi8658->gyro_out.z;
        QMI8658_ERR("gyro_work_func read data fail, retry=%d\n", retry);
    } else {
        // 读取成功，更新缓存
        g_qmi8658->gyro_out.x = gyro.x;
        g_qmi8658->gyro_out.y = gyro.y;
        g_qmi8658->gyro_out.z = gyro.z;
    }

    // 上报数据到input子系统
    input_report_abs(g_qmi8658->gyro_input, ABS_X, gyro.x);
    input_report_abs(g_qmi8658->gyro_input, ABS_Y, gyro.y);
    input_report_abs(g_qmi8658->gyro_input, ABS_Z, gyro.z);
    input_report_abs(g_qmi8658->gyro_input, ABS_THROTTLE, 3);
    input_sync(g_qmi8658->gyro_input);

    QMI8658_LOG("%s: [%d %d %d] dps\n", __func__, gyro.x, gyro.y, gyro.z);

    // 继续调度下一次读取
    queue_delayed_work(system_wq, &g_qmi8658->gyro_work, msecs_to_jiffies(atomic_read(&g_qmi8658->gyro_delay)));

}



static int qmi8658_input_init(struct Qmi8658_Data *qmi8658)
{
	struct input_dev *dev = NULL;
	int err = 0;

	QMI8658_LOG("acc input init\n");
	dev = input_allocate_device();
	if(!dev) {
		QMI8658_ERR("acc input can't allocate device!\n");
		return -ENOMEM;
	}

	dev->name = QMI8658_ACC_INPUT_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(dev, ABS_X, QMI8658_ACC_8G_MIN, QMI8658_ACC_8G_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, QMI8658_ACC_8G_MIN, QMI8658_ACC_8G_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, QMI8658_ACC_8G_MIN, QMI8658_ACC_8G_MAX, 0, 0);
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	input_set_drvdata(dev, (void*)qmi8658);
	err = input_register_device(dev);
	if(err < 0) {
		QMI8658_ERR("acc input can't register device!\n");
		input_free_device(dev);
		return err;
	}
	qmi8658->acc_input = dev;


	QMI8658_LOG("gyro input init\n");
	dev = input_allocate_device();
	if(!dev) {
		QMI8658_ERR("gyro input can't allocate device!\n");
		return -ENOMEM;
	}
	dev->name = QMI8658_GYRO_INPUT_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(dev, ABS_X, QMI8658_GYRO_DPS_MIN, QMI8658_GYRO_DPS_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, QMI8658_GYRO_DPS_MIN, QMI8658_GYRO_DPS_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, QMI8658_GYRO_DPS_MIN, QMI8658_GYRO_DPS_MAX, 0, 0);
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	input_set_drvdata(dev, (void*)qmi8658);
	err = input_register_device(dev);
	if(err < 0) {
		QMI8658_ERR("gyro input can't register device!\n");
		input_free_device(dev);
		return err;
	}
	qmi8658->gyro_input = dev;

	return 0;
}


static void qmi8658_input_deinit(struct Qmi8658_Data *qmi8658)
{
	QMI8658_LOG("%s called\n", __func__);
	if(qmi8658->acc_input)
	{
		input_unregister_device(qmi8658->acc_input);
		input_free_device(qmi8658->acc_input);
		qmi8658->acc_input = NULL;
	}
	
	if(qmi8658->gyro_input)
	{
		input_unregister_device(qmi8658->gyro_input);
		input_free_device(qmi8658->gyro_input);
		qmi8658->gyro_input = NULL;
	}
}


//static int qmi8658_get_enable(enum Qmi8658_type type)
//{
//	QMI8658_FUN();

//	if(type == QMI8658_TYPE_ACC)
//		return atomic_read(&g_qmi8658->acc_enable);
//	else if(type == QMI8658_TYPE_GYRO)
//		return atomic_read(&g_qmi8658->gyro_enable);
//	else
//		return 0;
//}

#if 0
static int qmi8658_set_enable(enum Qmi8658_type type, int enable)
{
	int acc_enable, gyro_enable;

	QMI8658_LOG("%s: type:%d--enable :%d\n",__func__,type, enable);
	if(type == QMI8658_TYPE_ACC)
	{
		atomic_set(&g_qmi8658->acc_enable, enable);
		if(enable) 
		{
			schedule_delayed_work(&g_qmi8658->acc_work,msecs_to_jiffies(atomic_read(&g_qmi8658->acc_delay))+1);
		}	
		else
		{
			cancel_delayed_work_sync(&g_qmi8658->acc_work);		
		}
	}
	else if(type == QMI8658_TYPE_GYRO)
	{
		atomic_set(&g_qmi8658->gyro_enable, enable);
		if(enable) 
		{
			schedule_delayed_work(&g_qmi8658->gyro_work,msecs_to_jiffies(atomic_read(&g_qmi8658->gyro_delay))+1);
		}
		else
		{
			cancel_delayed_work_sync(&g_qmi8658->gyro_work);		
		}
	}
	acc_enable = atomic_read(&g_qmi8658->acc_enable);
	gyro_enable = atomic_read(&g_qmi8658->gyro_enable);

	if(acc_enable)
	{
		qmi8658_config_acc(g_qmi8658->acc_range, g_qmi8658->acc_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_ACC, 1);
	}
	else
	{
		qmi8658_enable_sensors(QMI8658_TYPE_ACC, 0);
	}

	if(gyro_enable)
	{
		qmi8658_config_gyro(g_qmi8658->gyro_range, g_qmi8658->gyro_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		qmi8658_enable_sensors(QMI8658_TYPE_GYRO, 1);
	}
	else
	{
		qmi8658_enable_sensors(QMI8658_TYPE_GYRO, 0);
	}

	return 0;
}
#endif

#if 0
static ssize_t show_init_imu_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = qmi8658_read_chip_id(g_qmi8658->client);
	if(err < 0)
	{
		QMI8658_ERR("%s: g_qmi8658 read id fail!\n", __func__);
	}
	qmi8658_initialize(g_qmi8658->client);	
	qmi8658_config_acc(g_qmi8658->acc_range, g_qmi8658->acc_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	qmi8658_config_gyro(g_qmi8658->gyro_range, g_qmi8658->gyro_odr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 1);

	return sprintf(buf, "init done!\n");
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "QMI8658\n");
}

static ssize_t show_sensordata_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	qmi8658_read_accel_xyz(&g_qmi8658->acc_out);
	qmi8658_read_gyro_xyz(&g_qmi8658->gyro_out);
	return sprintf(buf, "%d %d %d	%d %d %d\n",g_qmi8658->acc_out.x,g_qmi8658->acc_out.y,g_qmi8658->acc_out.z,
												g_qmi8658->gyro_out.x,g_qmi8658->gyro_out.y,g_qmi8658->gyro_out.z);
}
		
static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[600];
	char tempstrbuf[24];
	unsigned char databuf[2]={0};
	int length=0;

	QMI8658_FUN();
	/* Check status register for data availability */
	for(i =0;i<=50;i++)
	{
		databuf[0] = i;
		res = qmi8658_read_reg(databuf[0], &databuf[1], 1);
		if(res < 0)
			QMI8658_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "0x%2x=0x%2x\n",databuf[0], databuf[1]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMI8658_FUN();

	return sprintf(buf, "%d\n", atomic_read(&g_qmi8658->position));
}

static ssize_t store_layout_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int position = 0;

	QMI8658_FUN();

	if(1 == sscanf(buf, "%d", &position))
	{
		if((position >= 0) && (position <= 7))
		{
			atomic_set(&g_qmi8658->position, position);
			qmi8658_set_layout(position);
		}
	}
	else
	{
		QMI8658_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}


static ssize_t qmi8658_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMI8658_FUN();

	return sprintf(buf, "acc:%d gyro:%d\n", atomic_read(&g_qmi8658->acc_enable),atomic_read(&g_qmi8658->gyro_enable));
}

static ssize_t qmi8658_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int type, enable=0;

	QMI8658_FUN();
	
	if(2 == sscanf(buf, "%d %d", &type, &enable))
	{
		if((type==QMI8658_TYPE_ACC)||(type==QMI8658_TYPE_GYRO))
		{
			if(enable)
			{
				qmi8658_set_enable(type, 1);
			}
			else
			{
				qmi8658_set_enable(type, 0);
			}
		}
	}
	else
	{	
		QMI8658_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t qmi8658_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMI8658_FUN();
	return sprintf(buf, "acc:%d gyro:%d\n", atomic_read(&g_qmi8658->acc_delay),atomic_read(&g_qmi8658->gyro_delay));
}

static ssize_t qmi8658_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int type, delay;

	QMI8658_FUN();
	if(2 == sscanf(buf, "%d %d", &type, &delay))
	{
		if(delay > QMI8658_MAX_DELAY)
			delay = QMI8658_MAX_DELAY;
		else if(delay <= 1)
			delay = 1;

		if((type==QMI8658_TYPE_ACC)||(type==QMI8658_TYPE_GYRO))
		{
			qmi8658_set_delay(type, delay);
		}
	}
	else
	{
		QMI8658_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}

static unsigned char qmi8658_debug_reg_addr=0x00;
static ssize_t qmi8658_getreg(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res = 0;
	unsigned char data=0xff;

	QMI8658_FUN();
	res = qmi8658_read_reg(qmi8658_debug_reg_addr, &data, 1);

	return sprintf(buf, "0x%x\n",  data);
}

static ssize_t qmi8658_setreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr, value;
	unsigned char data;	
	int res = 0;	
	
	QMI8658_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);	
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))	
	{		
		QMI8658_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);
		qmi8658_debug_reg_addr = (unsigned char)addr;
		data = (unsigned char)value;
		res = qmi8658_write_reg(qmi8658_debug_reg_addr, data);
		if(res)
		{
			QMI8658_ERR("write reg 0x%02x fail\n", addr);
		}
	}
	else
	{
		QMI8658_ERR("store_reg get para error\n");
	}

	return count;
}
#endif

#if defined(ACC_USE_CALI)
static void acc_write_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp))
	{
		printk("acc_write_file open file error\n");
	}
	else
	{
		//printk("acc_write_file data=0x%x len=%d\n", data, len);
		//snprintf();
		fp->f_op->write(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
	}

	set_fs(fs);
}

static void acc_read_file(char * filename, char *data, int len)
{
	struct file *fp;
	mm_segment_t fs;

	if(acc_cali_flag == 1)
	{
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp))
	{
		acc_cali_flag = 1;
		printk("qmi8658_read_file open file error\n");
	}
	else
	{
		//printk("qmi8658_read_file data=0x%x len=%d\n", data, len);
		fp->f_op->read(fp, data, len , &fp->f_pos);
		filp_close(fp, NULL);
		acc_cali_flag = 1;
	}

	set_fs(fs);
}

static ssize_t acc_cali_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_cali[0], acc_cali[1], acc_cali[2]);
}

static ssize_t acc_cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int en = 0;
	int data[3], data_avg[3];
	int icount, z_max, z_min;	
	struct qmi8658_acc acc;
	//struct Qmi8658_Data *g_qmi8658 = i2c_get_clientdata(this_client);
#if 0
	if (strict_strtoul(buf, 10, &en))
		return -EINVAL;
#endif
	if(1 == sscanf(buf, "%d", &en))
	{
		
	}
	else
	{
		QMI8658_ERR("invalid format = '%s'\n", buf);
		return count;
	}
	en = en ? 1 : 0;

	if(en)
	{	
		data_avg[0] = 0;
		data_avg[1] = 0;
		data_avg[2] = 0;
		for(icount=0; icount<QMA6981_CALI_NUM; icount++)
		{
			//qmi8658_read_raw_xyz(acc_qst, data);			
			qmi8658_read_accel_xyz(&acc);
			data_avg[0] += acc.x;	//data[0];
			data_avg[1] += acc.y;	//data[1];
			data_avg[2] += acc.z;	//data[2];
			// add by yangzhiqiang check vibrate
			if(icount == 0)
			{
				z_max = acc.z;
				z_min = acc.z;
			}
			else
			{
				z_max = (acc.z>z_max)?acc.z:z_max;
				z_min = (acc.z<z_min)?acc.z:z_min;
			}
			// add by yangzhiqiang check vibrate
			mdelay(5);
		}
		// add by yangzhiqiang check vibrate
		if((z_max-z_min)>(QMA6981_LSB_1G*3/10))
		{
			printk("acc_cali_store check vibrate cali ingore!\n");
			return count;
		}
		// add by yangzhiqiang check vibrate

		data_avg[0] = data_avg[0]/QMA6981_CALI_NUM;
		data_avg[1] = data_avg[1]/QMA6981_CALI_NUM;
		data_avg[2] = data_avg[2]/QMA6981_CALI_NUM;
		printk("acc_cali_store data_avg[%d %d %d]\n", data_avg[0], data_avg[1], data_avg[2]);
		// add by yangzhiqiang check offset range
#if 0
		if(QMA6981_ABS(data_avg[2]-QMA6981_LSB_1G)>(QMA6981_LSB_1G*5/10))
		{
			printk("acc_cali_store check offset range cali ingore!\n");
			return count;
		}
#endif
		// add by yangzhiqiang check offset range
		data[0] = 0-data_avg[0];
		data[1] = 0-data_avg[1];
		data[2] = QMA6981_LSB_1G-data_avg[2];
		acc_cali[0] += data[0];
		acc_cali[1] += data[1];
		acc_cali[2] += data[2];
		printk("acc_cali_store offset[%d %d %d]\n", data[0], data[1], data[2]);
		printk("acc_cali_store acc_cali[%d %d %d]\n", acc_cali[0], acc_cali[1], acc_cali[2]);
		acc_write_file(QMA6981_CALI_FILE, (char *)acc_cali, sizeof(acc_cali));
		
	}
	else
	{
	}
	
	return count;
}
#endif

#if 0
static DEVICE_ATTR(init_imu,		QMI8658_ATTR_R, show_init_imu_value, NULL);
static DEVICE_ATTR(chipinfo,		QMI8658_ATTR_R, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata,	QMI8658_ATTR_R, show_sensordata_value, NULL);
static DEVICE_ATTR(dumpallreg,	QMI8658_ATTR_R, show_dumpallreg_value, NULL);
static DEVICE_ATTR(layout,		QMI8658_ATTR_WR, show_layout_value, store_layout_value);
static DEVICE_ATTR(enable_imu,	QMI8658_ATTR_WR, qmi8658_enable_show, qmi8658_enable_store);
static DEVICE_ATTR(delay_imu,		QMI8658_ATTR_WR, qmi8658_delay_show, qmi8658_delay_store);
static DEVICE_ATTR(setreg,		QMI8658_ATTR_WR, qmi8658_getreg, qmi8658_setreg);
#endif

#if defined(ACC_USE_CALI)
static DEVICE_ATTR(cali,			QMI8658_ATTR_WR, acc_cali_show, acc_cali_store);
#endif


#if 0
static struct attribute *qmi8658_attributes[] = {
	&dev_attr_init_imu.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_enable_imu.attr,
	&dev_attr_delay_imu.attr,
	&dev_attr_setreg.attr,
#if defined(ACC_USE_CALI)
	&dev_attr_cali.attr,
#endif
	NULL
};

static struct attribute_group qmi8658_attribute_group = {
	.name = "qmi8658",
	.attrs = qmi8658_attributes
};

static long qmi8658_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	//void __user *argp = (void __user *)arg;
	//int temp = 0;

	QMI8658_LOG("%s: cmd %x\n",__func__, cmd);
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if(err)
	{
		QMI8658_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == g_qmi8658->client)
	{
		QMI8658_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}
	
	switch(cmd)
	{
	default:
		QMI8658_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}
	
    return 0;
}
#endif

#if 0
static int qmi8658_open(struct inode *inode, struct file *file)
{
	int err = 0;

	QMI8658_FUN();

	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		QMI8658_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(g_qmi8658->client);

	return 0;
}

static int qmi8658_release(struct inode *inode, struct file *file)
{
	QMI8658_FUN();
	file->private_data = NULL;
	return 0;
}
#endif

#if 0
static const struct file_operations qmi8658_misc_fops = {
	.owner = THIS_MODULE,
	.open = qmi8658_open,
	.release = qmi8658_release,
	.unlocked_ioctl = qmi8658_unlocked_ioctl,
};
#endif

#if 0
static struct miscdevice qmi8658_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMI8658_DEV_NAME,
	.fops = &qmi8658_misc_fops,
};
#endif

#if 0
static int qmi8658_acc_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	ret = qmi8658_set_enable(QMI8658_TYPE_ACC, enable);

	return ret;
}

static int qmi8658_acc_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	qmi8658_set_delay(QMI8658_TYPE_ACC, delay_msec);
	return 0;
}

static int qmi8658_gyro_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int ret = 0;

	ret = qmi8658_set_enable(QMI8658_TYPE_GYRO, enable);

	return ret;
}

static int qmi8658_gyro_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	qmi8658_set_delay(QMI8658_TYPE_GYRO, delay_msec);
	return 0;
}
#endif

#if 0
static int qmi8658_acc_axis_calibrate(int *cal_xyz)
{
	short xyz[3] = { 0 };
	int arry[3] = { 0 };
	int err;
	int i;

	for(i = 0; i < QMI8658_CAL_MAX; i++) 
	{
		msleep(100);
		err = qmi8658_acc_read_raw(xyz);
		if (err < 0) {
			printk("get_acceleration_data failed\n");
			return err;
		}
		if (i < QMI8658_CAL_SKIP_COUNT)
			continue;
		arry[0] += xyz[0];
		arry[1] += xyz[1];
		arry[2] += xyz[2];
	}
	cal_xyz[0] = arry[0] / (QMI8658_CAL_MAX - QMI8658_CAL_SKIP_COUNT);
	cal_xyz[1] = arry[1] / (QMI8658_CAL_MAX - QMI8658_CAL_SKIP_COUNT);
	cal_xyz[2] = arry[2] / (QMI8658_CAL_MAX - QMI8658_CAL_SKIP_COUNT);

	return 0;
}

static int qmi8658_acc_calibrate(struct sensors_classdev *sensors_cdev,int axis, int apply_now)
{
	int err;
	int xyz[3] = { 0 };
#if 0
	if (acc_qst->enable_flag == false)
	{
		err = qmaX981_power_set(acc_qst, true);
		if (err) {
			MSE_ERR("Fail to power on the device!\n");
			return err;
		}
		err = qma6981_initialize(acc_qst->i2c);
		if (err < 0)
			return err;
	}
#endif
	err = qmi8658_acc_axis_calibrate(xyz);
	if(err)
	{
		QMI8658_ERR("qmi8658_acc_calibrate fail!\n");
		return err;
	}

	switch (axis) {
	case axis_x:
		xyz[1] = 0;
		xyz[2] = 0;
		break;
	case axis_y:
		xyz[0] = 0;
		xyz[2] = 0;
		break;
	case axis_z:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = xyz[2] - g_qmi8658->acc_lsb;
		break;
	case axis_total:
		xyz[2] = xyz[2] - g_qmi8658->acc_lsb;
		break;
	default:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = 0;
		QMI8658_ERR( "can not calibrate accel\n");
		break;
	}
	memset(g_qmi8658->calibrate_buf, 0, sizeof(g_qmi8658->calibrate_buf));
	snprintf(g_qmi8658->calibrate_buf, sizeof(g_qmi8658->calibrate_buf),"%d,%d,%d", xyz[0], xyz[1], xyz[2]);
	if(apply_now) {
		g_qmi8658->cal_params[0] = xyz[0];
		g_qmi8658->cal_params[1] = xyz[1];
		g_qmi8658->cal_params[2] = xyz[2];
		g_qmi8658->use_cal = true;
	}
	
	return 0;
}


static int qmi8658_acc_write_cal_params(struct sensors_classdev *sensors_cdev,struct cal_result_t *cal_result)
{
	g_qmi8658->cal_params[0] = cal_result->offset_x;
	g_qmi8658->cal_params[1] = cal_result->offset_y;
	g_qmi8658->cal_params[2] = cal_result->offset_z;

	snprintf(g_qmi8658->calibrate_buf, sizeof(g_qmi8658->calibrate_buf),
			"%d,%d,%d", g_qmi8658->cal_params[0], g_qmi8658->cal_params[1],g_qmi8658->cal_params[2]);
	g_qmi8658->use_cal = true;
	QMI8658_LOG( "read accel calibrate bias %d,%d,%d\n",g_qmi8658->cal_params[0], g_qmi8658->cal_params[1], g_qmi8658->cal_params[2]);

	return 0;
}
#endif


#ifdef CONFIG_OF
static int qmi8658_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	unsigned int position = 0;
	
	ret = of_property_read_u32(np, "qst,layout", &position);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		return 0;
	}
	atomic_set(&g_qmi8658->position, (int)position);
	return 0;
}
#endif

/**
 * 初始化QMI8658传感器
 * @param client I2C客户端
 */
void qmi8658_initialize(struct i2c_client *client)
{
    int ret;
    u8 reg;
    u8 chip_id = 0;

    QMI8658_LOG("qmi8658: initialize start\n");

    if (!g_qmi8658) {
        QMI8658_ERR("qmi8658_initialize: g_qmi8658 is NULL\n");
        return;
    }

    /* 1. 读取芯片ID确认通信正常 */
    ret = qmi8658_read_reg(Qmi8658Register_WhoAmI, &chip_id, 1);
    if (ret < 0 || chip_id != QMI8658_CHIP_ID) {
        QMI8658_ERR("qmi8658_initialize: chip_id check failed, read=0x%02x, expected=0x%02x\n", 
                    chip_id, QMI8658_CHIP_ID);
        return;
    }
    QMI8658_LOG("qmi8658: chip_id=0x%02x OK\n", chip_id);

    /* 2. soft reset */
    reg = 0xB0;
    ret = qmi8658_write_reg(Qmi8658Register_Reset, reg);
    if (ret < 0) {
        QMI8658_ERR("qmi8658_initialize: soft reset failed\n");
        return;
    }
    msleep(50);  // 复位后等待更长时间
    
    // 复位后再次读取芯片ID确认
    ret = qmi8658_read_reg(Qmi8658Register_WhoAmI, &chip_id, 1);
    if (ret < 0 || chip_id != QMI8658_CHIP_ID) {
        QMI8658_ERR("qmi8658_initialize: chip_id check after reset failed, read=0x%02x\n", chip_id);
        return;
    }
    QMI8658_LOG("qmi8658: chip_id after reset=0x%02x OK\n", chip_id);

    /* 3. ACC config */
    ret = qmi8658_config_acc(
        Qmi8658AccRange_8g,
        Qmi8658AccOdr_250Hz,
        Qmi8658Lpf_Enable,
        Qmi8658St_Disable
    );
    if (ret < 0) {
        QMI8658_ERR("qmi8658_initialize: config_acc failed\n");
        return;
    }
    QMI8658_LOG("qmi8658: acc config done, range=8g, odr=250Hz\n");

    /* 4. Gyro config */
    ret = qmi8658_config_gyro(
        Qmi8658GyrRange_1024dps,
        Qmi8658GyrOdr_250Hz,
        Qmi8658Lpf_Enable,
        Qmi8658St_Disable
    );
    if (ret < 0) {
        QMI8658_ERR("qmi8658_initialize: config_gyro failed\n");
        return;
    }
    QMI8658_LOG("qmi8658: gyro config done, range=1024dps, odr=250Hz\n");

    /* 5. set Ctrl1 (根据原始代码，设置为0x60用于正常模式) */
    ret = qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60);
    if (ret < 0) {
        QMI8658_ERR("qmi8658_initialize: set Ctrl1 failed\n");
        return;
    }
    QMI8658_LOG("qmi8658: Ctrl1 set to 0x60\n");

    /* 6. enable ACC + GYRO */
    qmi8658_enable_sensors(QMI8658_TYPE_ACC | QMI8658_TYPE_GYRO, 1);

    msleep(100);  // 增加等待时间，让传感器稳定

    /* 7. 验证配置：读取关键寄存器确认配置成功 */
    {
        u8 ctrl1_val = 0, ctrl2_val = 0, ctrl3_val = 0, ctrl7_val = 0, status0_val = 0;
        qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1_val, 1);
        qmi8658_read_reg(Qmi8658Register_Ctrl2, &ctrl2_val, 1);
        qmi8658_read_reg(Qmi8658Register_Ctrl3, &ctrl3_val, 1);
        qmi8658_read_reg(Qmi8658Register_Ctrl7, &ctrl7_val, 1);
        qmi8658_read_reg(Qmi8658Register_Status0, &status0_val, 1);
        QMI8658_LOG("qmi8658: Ctrl1=0x%02x, Ctrl2=0x%02x, Ctrl3=0x%02x, Ctrl7=0x%02x, Status0=0x%02x\n",
                    ctrl1_val, ctrl2_val, ctrl3_val, ctrl7_val, status0_val);
    }

    /* 8. 陀螺仪零点校准（在静止状态下采集样本计算偏移） */
    {
        short raw_xyz[3];
        long sum[3] = {0, 0, 0};
        int i, sample_count = 50;  // 采集50个样本
        
        QMI8658_LOG("qmi8658: starting gyro calibration, collecting %d samples...\n", sample_count);
        msleep(200);  // 等待传感器完全稳定
        
        for (i = 0; i < sample_count; i++) {
            ret = qmi8658_gyro_read_raw(raw_xyz);
            if (ret == 0) {
                sum[0] += raw_xyz[0];
                sum[1] += raw_xyz[1];
                sum[2] += raw_xyz[2];
            }
            msleep(10);  // 每个样本间隔10ms
        }
        
        // 计算平均值作为零点偏移
        g_qmi8658->gyro_bias[0] = (short)(sum[0] / sample_count);
        g_qmi8658->gyro_bias[1] = (short)(sum[1] / sample_count);
        g_qmi8658->gyro_bias[2] = (short)(sum[2] / sample_count);
        g_qmi8658->gyro_calibrated = true;
        
        QMI8658_LOG("qmi8658: gyro calibration done, bias=[%d %d %d]\n",
                    g_qmi8658->gyro_bias[0], g_qmi8658->gyro_bias[1], g_qmi8658->gyro_bias[2]);
    }

    QMI8658_LOG("qmi8658 initialize done.\n");
}



static int qmi8658_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static struct Qmi8658_Data *qmi8658;
	int err = 0;
	int layout = 0;

	QMI8658_LOG("%s: start\n",__func__);
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C|I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA))
	{
		QMI8658_ERR("%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit;
	}
	qmi8658 = kzalloc(sizeof(struct Qmi8658_Data), GFP_KERNEL);
	if(!qmi8658) {
		QMI8658_ERR("%s: can't allocate memory for Qmi8658_Data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	g_qmi8658 = qmi8658;
	atomic_set(&g_qmi8658->position, 0);
#ifdef CONFIG_OF
	if(client->dev.of_node)
	{
		qmi8658_parse_dt(&client->dev);
	}
#endif
	layout = atomic_read(&g_qmi8658->position);
	qmi8658_set_layout(layout);

	mutex_init(&g_qmi8658->op_mutex);
	atomic_set(&g_qmi8658->acc_delay, 200);
	atomic_set(&g_qmi8658->gyro_delay, 200);
	
	// 初始化默认单位
	g_qmi8658->acc_uint = Qmi8658AccUnit_g;
	g_qmi8658->gyro_uint = Qmi8658GyrUnit_dps;
	
	// 初始化输出数据缓存
	g_qmi8658->acc_out.x = 0;
	g_qmi8658->acc_out.y = 0;
	g_qmi8658->acc_out.z = 0;
	g_qmi8658->gyro_out.x = 0;
	g_qmi8658->gyro_out.y = 0;
	g_qmi8658->gyro_out.z = 0;
	
	// 初始化陀螺仪校准状态
	g_qmi8658->gyro_bias[0] = 0;
	g_qmi8658->gyro_bias[1] = 0;
	g_qmi8658->gyro_bias[2] = 0;
	g_qmi8658->gyro_calibrated = false;
	
	g_qmi8658->client = client;
	i2c_set_clientdata(client, g_qmi8658);
    //wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
	err = qmi8658_read_chip_id(client);
	if(err < 0)
	{
		QMI8658_ERR("%s: g_qmi8658 read id fail!\n", __func__);
		goto exit1;
	}
	qmi8658_initialize(client);
	

	INIT_DELAYED_WORK(&g_qmi8658->acc_work, acc_work_func);
	INIT_DELAYED_WORK(&g_qmi8658->gyro_work, gyro_work_func);

	err = qmi8658_input_init(g_qmi8658);
	if(err < 0) {
		QMI8658_ERR("input init fail!\n");
		goto exit1;
	}

	atomic_set(&g_qmi8658->acc_enable, 1);
	atomic_set(&g_qmi8658->gyro_enable, 1);


    /* 设置一个合理的采样周期（单位：ms），比如 20ms≈50Hz */
    atomic_set(&g_qmi8658->acc_delay,  1000);
    atomic_set(&g_qmi8658->gyro_delay, 1000);

    /* 第一次调度，后面会在 work_func 里自己循环调度 */
    queue_delayed_work(system_wq, &g_qmi8658->acc_work, msecs_to_jiffies(20));
	queue_delayed_work(system_wq, &g_qmi8658->gyro_work, msecs_to_jiffies(20));


    QMI8658_LOG("qmi8658: work started\n");

	#if 0
	err = misc_register(&qmi8658_device);
	if(err) {
		QMI8658_ERR("%s: create register fail!\n", __func__);
		goto exit2;
	}

	//err = sysfs_create_group(&qmi8658->acc_input->dev.kobj, &qmi8658_attribute_group);
	err = sysfs_create_group(&qmi8658_device.this_device->kobj, &qmi8658_attribute_group);
	if(err < 0) {
		QMI8658_ERR("%s: create group fail!\n", __func__);
		goto exit3;
	}

	g_qmi8658->accel_cdev = qmi8658_acc_cdev;
	g_qmi8658->accel_cdev.delay_msec = 200;
	g_qmi8658->accel_cdev.sensors_enable = qmi8658_acc_enable_set;
	g_qmi8658->accel_cdev.sensors_poll_delay = qmi8658_acc_delay_set;
	g_qmi8658->accel_cdev.sensors_calibrate = qmi8658_acc_calibrate;
	g_qmi8658->accel_cdev.sensors_write_cal_params = qmi8658_acc_write_cal_params;
	memset(&g_qmi8658->accel_cdev.cal_result, 0, sizeof(g_qmi8658->accel_cdev.cal_result));
	g_qmi8658->accel_cdev.params = g_qmi8658->calibrate_buf;
	err = sensors_classdev_register(&g_qmi8658->acc_input->dev, &g_qmi8658->accel_cdev);
	if(err)
	{
		QMI8658_ERR("class device create failed: %d\n",err);
		goto exit4;
	}
	
	g_qmi8658->gyro_cdev = qmi8658_gyro_cdev;
	g_qmi8658->gyro_cdev.delay_msec = 200;
	g_qmi8658->gyro_cdev.sensors_enable = qmi8658_gyro_enable_set;
	g_qmi8658->gyro_cdev.sensors_poll_delay = qmi8658_gyro_delay_set;
	err = sensors_classdev_register(&g_qmi8658->gyro_input->dev, &g_qmi8658->gyro_cdev);
	if(err)
	{
		QMI8658_ERR("create stepcount class device file failed!\n");
		err = -EINVAL;
		goto exit5;
	}
	#endif

	//atomic_set(&g_qmi8658->acc_enable, 1);
    //atomic_set(&g_qmi8658->gyro_enable, 1);
//
    ///* 设置采样周期，单位毫秒，看你驱动里 acc_delay/gyro_delay 的语义 */
    //atomic_set(&g_qmi8658->acc_delay, 20);   // 20ms ≈ 50Hz
    //atomic_set(&g_qmi8658->gyro_delay, 20);
//
    ///* 启动 workqueue（名字按你驱动里的来，有的是 schedule_delayed_work，
    //   有的是 queue_delayed_work(g_qmi8658->workqueue, ...)） */
    //schedule_delayed_work(&g_qmi8658->acc_work,
    //                      msecs_to_jiffies(20));
    //schedule_delayed_work(&g_qmi8658->gyro_work,
    //                      msecs_to_jiffies(20));
	
	return 0;
	#if 0
exit5:
	sensors_classdev_unregister(&g_qmi8658->accel_cdev);
exit4:
	//sysfs_remove_group(&g_qmi8658->acc_input->dev.kobj, &qmi8658_attribute_group);
	sysfs_remove_group(&qmi8658_device.this_device->kobj, &qmi8658_attribute_group);
exit3:

	misc_deregister(&qmi8658_device);

exit2:
	qmi8658_input_deinit(g_qmi8658);	
	#endif
exit1:
	if(g_qmi8658)
	{
		kfree(g_qmi8658);
		g_qmi8658 = NULL;
	}
exit:
	return err;	
}

/**
 * 移除设备时的清理函数
 * @param client I2C客户端
 * @return 0成功
 */
static int qmi8658_remove(struct i2c_client *client)
{
	QMI8658_FUN();
	if(g_qmi8658)
	{
		// 取消工作队列
		cancel_delayed_work_sync(&g_qmi8658->acc_work);
		cancel_delayed_work_sync(&g_qmi8658->gyro_work);
		
		// 禁用传感器
		qmi8658_enable_sensors(QMI8658_TYPE_ACC | QMI8658_TYPE_GYRO, 0);
		
	#if 0
		sensors_classdev_unregister(&g_qmi8658->gyro_cdev);
		sensors_classdev_unregister(&g_qmi8658->accel_cdev);
		//sysfs_remove_group(&g_qmi8658->acc_input->dev.kobj, &qmi8658_attribute_group);
		sysfs_remove_group(&qmi8658_device.this_device->kobj, &qmi8658_attribute_group);
	#endif
		qmi8658_input_deinit(g_qmi8658);
		kfree(g_qmi8658);
		g_qmi8658 = NULL;
	}
	return 0;
}

static int qmi8658_i2c_remove(struct i2c_client *client)
{
	return qmi8658_remove(client);
}

#if 0
static int qmi8658_suspend(struct i2c_client *client, pm_message_t mesg)
{
	QMI8658_FUN();

	cancel_delayed_work_sync(&g_qmi8658->acc_work);
	cancel_delayed_work_sync(&g_qmi8658->gyro_work);
	//qmi8658_set_mode(QMI8658_MODE_POWERON_DEFAULT);
	qmi8658_enable_sensors(QMI8658_TYPE_ACC|QMI8658_TYPE_GYRO, 0);

	return 0;
}

static int qmi8658_resume(struct i2c_client *client)
{
#if 0
	int delay = atomic_read(&g_qmi8658->acc_delay);

	if(qmi8658_get_enable(QMI8658_TYPE_ACC))
	{
		qmi8658_set_mode(FIS_MODE_NOMAL);
		schedule_delayed_work(&g_qmi8658->acc_work,msecs_to_jiffies(delay));
	}
	if(qmi8658_get_enable(QMI8658_TYPE_GYRO))
	{
		qmi8658_set_mode(FIS_MODE_NOMAL);
		schedule_delayed_work(&g_qmi8658->gyro_work,msecs_to_jiffies(delay));
	}
#endif
	QMI8658_FUN();

	return 0;
}
#endif

static const struct i2c_device_id qmi8658_id[] = {
	{QMI8658_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, qmi8658_id);

#ifdef CONFIG_OF
static const struct of_device_id qmi8658_of_match[] = {
       { .compatible = "qst,qmi8658", },
       { }
};
#endif

MODULE_DEVICE_TABLE(of, qmi8658_of_match);
static struct i2c_driver qmi8658_driver = 
{
    .driver = {
        .name  = QMI8658_DEV_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = qmi8658_of_match,
#endif
    },
    .probe    = qmi8658_i2c_probe,
    .remove   = qmi8658_i2c_remove,
    .id_table = qmi8658_id,
};


static int __init qmi8658_i2c_init(void)
{
	QMI8658_LOG("%s A+G driver: init\n", QMI8658_DEV_NAME);

	return i2c_add_driver(&qmi8658_driver);
}

static void __exit qmi8658_i2c_exit(void)
{
	QMI8658_LOG("%s A+G driver exit\n", QMI8658_DEV_NAME);

	i2c_del_driver(&qmi8658_driver);
}

module_init(qmi8658_i2c_init);			//late_initcall(qmi8658_i2c_init);
module_exit(qmi8658_i2c_exit);


MODULE_DESCRIPTION("qmi8658 IMU driver");
MODULE_AUTHOR("zhiqiang_yang@qstcorp.com");
MODULE_LICENSE("GPL");
MODULE_VERSION(QMI8658_DEV_VERSION);

