/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef QMI8658_H
#define QMI8658_H

#include <linux/ioctl.h>

//#define ACC_USE_CALI

#define QMI8658_ABS(X) 			((X) < 0 ? (-1 * (X)) : (X))

//#define QMI8658_DEBUG
#if defined(QMI8658_DEBUG)
#define QMI8658_TAG				       "qmi8658"
#define QMI8658_FUN(f)               	printk(QMI8658_TAG"%s\n", __FUNCTION__)
#define QMI8658_ERR(fmt, args...)    	printk(QMI8658_TAG"%s %d" fmt, __FUNCTION__, __LINE__, ##args)
#define QMI8658_LOG(fmt, args...)    	printk(QMI8658_TAG"%s %d" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define QMI8658_FUN()        
#define QMI8658_LOG(fmt, args...)
#define QMI8658_ERR(fmt, args...)
#endif

#define QMI8658_DEV_NAME				"qmi8658"
#define QMI8658_DEV_VERSION				"1.0.1"
#define QMI8658_ACC_INPUT_NAME			"accelerometer"
#define QMI8658_GYRO_INPUT_NAME			"gyrocope"
#define QMI8658_ACC_8G_MIN				(-8 * 1024)
#define QMI8658_ACC_8G_MAX				(8 * 1024)
#define QMI8658_GYRO_DPS_MIN			(-2048)
#define QMI8658_GYRO_DPS_MAX			(2048)

#define QMI8658_MAX_DELAY				2000

#define QMI8658_ATTR_WR					S_IRUSR|S_IRGRP|S_IWUSR		//444 | 200
#define QMI8658_ATTR_R					S_IRUSR|S_IRGRP				//S_IRUGO		//444
#define QMI8658_ATTR_W					S_IWUSR						//200


#define QMI8658_I2C_SLAVE_ADDR_L		0x6a
#define QMI8658_I2C_SLAVE_ADDR_H		0x6b
#define QMI8658_CHIP_ID					0x05

#define QMI8658_CTRL5_ACC_HPF_ENABLE	0x01
#define QMI8658_CTRL5_ACC_LPF_ENABLE	0x02
#define QMI8658_CTRL5_GYR_HPF_ENABLE	0x04
#define QMI8658_CTRL5_GYR_LPF_ENABLE	0x08

#define QMI8658_CTRL7_DISABLE_ALL		0x0
#define QMI8658_CTRL7_ACC_ENABLE		0x1
#define QMI8658_CTRL7_GYR_ENABLE		0x2
#define QMI8658_CTRL7_MAG_ENABLE		0x4
#define QMI8658_CTRL7_AE_ENABLE			0x8
#define QMI8658_CTRL7_ENABLE_MASK		0xF

#define QMI8658_CAL_SKIP_COUNT 		5
#define QMI8658_CAL_MAX				(10 + QMI8658_CAL_SKIP_COUNT)
#define QMI8658_CAL_NUM				99

enum Qmi8658_type
{
	QMI8658_TYPE_NONE = 0x00,
	QMI8658_TYPE_ACC = 0x01,
	QMI8658_TYPE_GYRO = 0x02,

	QMI8658_TYPE_MAX = 0xff
};

enum axis 
{
	axis_x = 0,
	axis_y,
	axis_z,
	axis_total
};

typedef struct
{
	int x;
	int y;
	int z;
}Qmi8658_Acc;

typedef struct
{
	int x;
	int y;
	int z;
}Qmi8658_Gyro;

struct Qmi8658_convert
{
	signed char sign[3];
	unsigned char map[3];
};

enum Qmi8658_mode
{
	QMI8658_MODE_POWERON_DEFAULT,
	QMI8658_MODE_LOW_POWER,
	QMI8658_MODE_POWER_DOWN,
	QMI8658_MODE_ACC_ENABLE,
	QMI8658_MODE_GYRO_ENABLE,
	QMI8658_MODE_ACCGYRO_ENABLE
};

enum Qmi8658_LpfConfig
{
	Qmi8658Lpf_Disable, /*!< \brief Disable low pass filter. */
	Qmi8658Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum Qmi8658_HpfConfig
{
	Qmi8658Hpf_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_StConfig
{
	Qmi8658St_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658St_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_LpfMode
{
	A_LSP_MODE_0 = 0x00<<1,
	A_LSP_MODE_1 = 0x01<<1,
	A_LSP_MODE_2 = 0x02<<1,
	A_LSP_MODE_3 = 0x03<<1,

	G_LSP_MODE_0 = 0x00<<5,
	G_LSP_MODE_1 = 0x01<<5,
	G_LSP_MODE_2 = 0x02<<5,
	G_LSP_MODE_3 = 0x03<<5
};

enum Qmi8658_AccRange
{
	Qmi8658AccRange_2g = 0x00 << 4, /*!< \brief +/- 2g range */
	Qmi8658AccRange_4g = 0x01 << 4, /*!< \brief +/- 4g range */
	Qmi8658AccRange_8g = 0x02 << 4, /*!< \brief +/- 8g range */
	Qmi8658AccRange_16g = 0x03 << 4 /*!< \brief +/- 16g range */
};


enum Qmi8658_AccOdr
{
	Qmi8658AccOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658AccOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658AccOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658AccOdr_1000Hz = 0x03,  /*!< \brief High resolution 1000Hz output rate. */
	Qmi8658AccOdr_500Hz = 0x04,  /*!< \brief High resolution 500Hz output rate. */
	Qmi8658AccOdr_250Hz = 0x05, /*!< \brief High resolution 250Hz output rate. */
	Qmi8658AccOdr_125Hz = 0x06, /*!< \brief High resolution 125Hz output rate. */
	Qmi8658AccOdr_62_5Hz = 0x07, /*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658AccOdr_31_25Hz = 0x08,  /*!< \brief High resolution 31.25Hz output rate. */
	Qmi8658AccOdr_LowPower_128Hz = 0x0c, /*!< \brief Low power 128Hz output rate. */
	Qmi8658AccOdr_LowPower_21Hz = 0x0d,  /*!< \brief Low power 21Hz output rate. */
	Qmi8658AccOdr_LowPower_11Hz = 0x0e,  /*!< \brief Low power 11Hz output rate. */
	Qmi8658AccOdr_LowPower_3Hz = 0x0f    /*!< \brief Low power 3Hz output rate. */
};

enum Qmi8658_GyrRange
{
	Qmi8658GyrRange_32dps = 0 << 4,   /*!< \brief +-32 degrees per second. */
	Qmi8658GyrRange_64dps = 1 << 4,   /*!< \brief +-64 degrees per second. */
	Qmi8658GyrRange_128dps = 2 << 4,  /*!< \brief +-128 degrees per second. */
	Qmi8658GyrRange_256dps = 3 << 4,  /*!< \brief +-256 degrees per second. */
	Qmi8658GyrRange_512dps = 4 << 4,  /*!< \brief +-512 degrees per second. */
	Qmi8658GyrRange_1024dps = 5 << 4, /*!< \brief +-1024 degrees per second. */
	Qmi8658GyrRange_2048dps = 6 << 4, /*!< \brief +-2048 degrees per second. */
	Qmi8658GyrRange_4096dps = 7 << 4  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum Qmi8658_GyrOdr
{
	Qmi8658GyrOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658GyrOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658GyrOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658GyrOdr_1000Hz = 0x03,	/*!< \brief High resolution 1000Hz output rate. */
	Qmi8658GyrOdr_500Hz	= 0x04,	/*!< \brief High resolution 500Hz output rate. */
	Qmi8658GyrOdr_250Hz	= 0x05,	/*!< \brief High resolution 250Hz output rate. */
	Qmi8658GyrOdr_125Hz	= 0x06,	/*!< \brief High resolution 125Hz output rate. */
	Qmi8658GyrOdr_62_5Hz = 0x07,	/*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658GyrOdr_31_25Hz= 0x08	/*!< \brief High resolution 31.25Hz output rate. */
};

enum Qmi8658_AeOdr
{
	Qmi8658AeOdr_1Hz = 0x00,  /*!< \brief 1Hz output rate. */
	Qmi8658AeOdr_2Hz = 0x01,  /*!< \brief 2Hz output rate. */
	Qmi8658AeOdr_4Hz = 0x02,  /*!< \brief 4Hz output rate. */
	Qmi8658AeOdr_8Hz = 0x03,  /*!< \brief 8Hz output rate. */
	Qmi8658AeOdr_16Hz = 0x04, /*!< \brief 16Hz output rate. */
	Qmi8658AeOdr_32Hz = 0x05, /*!< \brief 32Hz output rate. */
	Qmi8658AeOdr_64Hz = 0x06,  /*!< \brief 64Hz output rate. */
	Qmi8658AeOdr_128Hz = 0x07,  /*!< \brief 128Hz output rate. */
	Qmi8658AeOdr_motionOnDemand = 128
};

enum Qmi8658_MagOdr
{
	Qmi8658MagOdr_1000Hz = 0x00,   /*!< \brief 1000Hz output rate. */
	Qmi8658MagOdr_500Hz = 0x01,	/*!< \brief 500Hz output rate. */
	Qmi8658MagOdr_250Hz = 0x02,	/*!< \brief 250Hz output rate. */
	Qmi8658MagOdr_125Hz = 0x03,	/*!< \brief 125Hz output rate. */
	Qmi8658MagOdr_62_5Hz = 0x04,	/*!< \brief 62.5Hz output rate. */
	Qmi8658MagOdr_31_25Hz = 0x05	/*!< \brief 31.25Hz output rate. */
};
	
enum Qmi8658_MagDev
{
	MagDev_AKM09918 = (0 << 3), /*!< \brief AKM09918. */
};

enum Qmi8658_AccUnit
{
	Qmi8658AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	Qmi8658AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum Qmi8658_GyrUnit
{
	Qmi8658GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	Qmi8658GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};


enum Qmi8658Register
{
	Qmi8658Register_WhoAmI = 0, // 0
	Qmi8658Register_Revision, // 1
	Qmi8658Register_Ctrl1, // 2
	Qmi8658Register_Ctrl2, // 3
	Qmi8658Register_Ctrl3, // 4
	Qmi8658Register_Ctrl4, // 5
	Qmi8658Register_Ctrl5, // 6
	Qmi8658Register_Ctrl6, // 7
	Qmi8658Register_Ctrl7, // 8
	Qmi8658Register_Ctrl8, // 9
	Qmi8658Register_Ctrl9,	// 10
	Qmi8658Register_Cal1_L = 11,
	Qmi8658Register_Cal1_H,
	Qmi8658Register_Cal2_L,
	Qmi8658Register_Cal2_H,
	Qmi8658Register_Cal3_L,
	Qmi8658Register_Cal3_H,
	Qmi8658Register_Cal4_L,
	Qmi8658Register_Cal4_H,
	Qmi8658Register_FifoCtrl = 19,
	Qmi8658Register_FifoData,	// 20
	Qmi8658Register_FifoStatus,	// 21	
	Qmi8658Register_StatusInt = 45,
	Qmi8658Register_Status0,
	Qmi8658Register_Status1,
	Qmi8658Register_Timestamp_L = 48,
	Qmi8658Register_Timestamp_M,
	Qmi8658Register_Timestamp_H,
	Qmi8658Register_Tempearture_L = 51,
	Qmi8658Register_Tempearture_H,
	Qmi8658Register_Ax_L = 53,
	Qmi8658Register_Ax_H,
	Qmi8658Register_Ay_L,
	Qmi8658Register_Ay_H,
	Qmi8658Register_Az_L,
	Qmi8658Register_Az_H,
	Qmi8658Register_Gx_L = 59,
	Qmi8658Register_Gx_H,
	Qmi8658Register_Gy_L,
	Qmi8658Register_Gy_H,
	Qmi8658Register_Gz_L,
	Qmi8658Register_Gz_H,
	Qmi8658Register_Mx_L = 65,
	Qmi8658Register_Mx_H,
	Qmi8658Register_My_L,
	Qmi8658Register_My_H,
	Qmi8658Register_Mz_L,
	Qmi8658Register_Mz_H,
	Qmi8658Register_Q1_L = 73,
	Qmi8658Register_Q1_H,
	Qmi8658Register_Q2_L,
	Qmi8658Register_Q2_H,
	Qmi8658Register_Q3_L,
	Qmi8658Register_Q3_H,
	Qmi8658Register_Q4_L,
	Qmi8658Register_Q4_H,
	Qmi8658Register_Dvx_L = 81,
	Qmi8658Register_Dvx_H,
	Qmi8658Register_Dvy_L,
	Qmi8658Register_Dvy_H,
	Qmi8658Register_Dvz_L,
	Qmi8658Register_Dvz_H,
	Qmi8658Register_AeReg1 = 87,
	Qmi8658Register_AeOverflow,

	Qmi8658Register_Reset = 0x60,	// 96, 复位寄存器

	Qmi8658Register_I2CM_STATUS = 110
};


#endif
