/*****************************************************************************
 *
 * File : CMP20X.h
 *
 * Date : 2022/06/09
 *
 ****************************************************************************/

#ifndef __QMP6990_H__
#define __QMP6990_H__

//#include "bsp_hardware.h"
#include "type_support_qmp.h"

//5 SA0/SDO 
//Serial data output 
//-I2C slave address selection pin, 0 for 0x3A, 1 for 0x3B 
//-Serial Data Output (SPI 4-Wire)/ NC (SPI 3-Wire).
/* -----  I2C Slave Address  ----- */
//#define QMP6990_7BIT_I2C_ADDR	0x3A
#define QMP6990_7BIT_I2C_ADDR	0x3B
#define QMP6990_8BIT_I2C_ADDR	0x3B<<1
#define I2C_SUCCESS		1

//#define SENSOR_FIFO_FUNC

/* --------------------------------------------------------------------- */
//
#define debug_mode		1 	// debug mode (0: disable, 1: enable)
#define polling_time	40	// sensor data polling time (unit: msec)

/* -----  Sensor Dara Resolution  ----- */
#define QMP6990_PRESS_SENSITIVITY 	 (64)  // (unit : count/Pa)
#define QMP6990_TEMP_SENSITIVITY 	 (65536)  // (unit : count/Celsius)

#define QMP6990_P_CODE_TO_PA(pCode)				((float)(pCode)/QMP6990_PRESS_SENSITIVITY)
#define QMP6990_T_CODE_TO_CELSIUS(tCode)		((float)(tCode)/QMP6990_TEMP_SENSITIVITY)

/* -----  Registers Address (RAM) ----- */
#define QMP6990_REG_CHIP_ID		     0x00
#define QMP6990_REG_REV_ID	 		 0x01
#define QMP6990_REG_ERR_MSG		     0x02
#define QMP6990_REG_STATUS		     0x03
#define QMP6990_REG_PRESS_XLSB	     0x04
#define QMP6990_REG_PRESS_LSB	     0x05
#define QMP6990_REG_PRESS_MSB	     0x06
#define QMP6990_REG_TEMP_XLSB	     0x07
#define QMP6990_REG_TEMP_LSB 	     0x08
#define QMP6990_REG_TEMP_MSB 	     0x09
#define QMP6990_REG_SENSOR_TIME_0    0x0C
#define QMP6990_REG_SENSOR_TIME_1    0x0D
#define QMP6990_REG_SENSOR_TIME_2    0x0E
#define QMP6990_REG_INT_STATUS       0x11
#define QMP6990_REG_FIFO_LENGTH_0    0x12
#define QMP6990_REG_FIFO_LENGTH_1    0x13
#define QMP6990_REG_FIFO_DATA        0x14
#define QMP6990_REG_FIFO_WM_0        0x15
#define QMP6990_REG_FIFO_WM_1        0x16
#define QMP6990_REG_FIFO_CONFIG_0    0x17
#define QMP6990_REG_FIFO_CONFIG_1    0x18
#define QMP6990_REG_INT_CTRL         0x19
#define QMP6990_REG_CONFIG           0x1A
#define QMP6990_REG_PWR_CTRL         0x1B
#define QMP6990_REG_OSR 	         0x1C
#define QMP6990_REG_ODR 	         0x1D
#define QMP6990_REG_FILTER           0x1F
#define QMP6990_REG_PRIMIF           0x22
#define QMP6990_REG_FILGAIN          0x30
#define QMP6990_REG_AGAIN            0x49
#define QMP6990_OTP_PWR 	         0x7A
#define QMP6990_OTP_ADDR	         0x7B
#define QMP6990_OTP_DATA	         0x7C
#define QMP6990_OTP_TRIG	         0x7D
#define QMP6990_REG_RESET            0x7E

/* -----  Registers Address (OTP) ----- */
#define QMP6990_SERIAL_NUMBER        0x00	// 0x00 ~ 0x0C

/* -----  Sensor PID  ----- */
#define QMP6990_PID		 	  0xA0

/* -----  Sensor Version  ----- */
#define QMP6990_Ver		 	  0x80

/* CONF ERR bit */
#define QMP6990_CONF_ERR__REG    QMP6990_REG_ERR_MSG
#define QMP6990_CONF_ERR__MSK	0x04
#define QMP6990_CONF_ERR__POS	2

/* DRDY TEMP bit */
#define QMP6990_DRDY_TEMP__REG	 QMP6990_REG_STATUS
#define QMP6990_DRDY_TEMP__MSK	 0x40
#define QMP6990_DRDY_TEMP__POS	 6

/* DRDY PRESS bit */
#define QMP6990_DRDY_PRESS__REG	 QMP6990_REG_STATUS
#define QMP6990_DRDY_PRESS__MSK	 0x20
#define QMP6990_DRDY_PRESS__POS	 5

/* DRDY INT bit */
#define QMP6990_DRDY_INT__REG	 QMP6990_REG_INT_STATUS
#define QMP6990_DRDY_INT__MSK	 0x10
#define QMP6990_DRDY_INT__POS	 4

/* FFULL INT bit */
#define QMP6990_FFULL_INT__REG	 QMP6990_REG_INT_STATUS
#define QMP6990_FFULL_INT__MSK	 0x02
#define QMP6990_FFULL_INT__POS	 1

/* FWM INT bit */
#define QMP6990_FWM_INT__REG	     QMP6990_REG_INT_STATUS
#define QMP6990_FWM_INT__MSK	     0x01
#define QMP6990_FWM_INT__POS	     0

/* FIFO TEMP EN bit */
#define QMP6990_FIFO_TEMP_EN__REG    QMP6990_REG_FIFO_CONFIG_0
#define QMP6990_FIFO_TEMP_EN__MSK    0x10
#define QMP6990_FIFO_TEMP_EN__POS    4

/* FIFO PRESS EN bit */
#define QMP6990_FIFO_PRESS_EN__REG    QMP6990_REG_FIFO_CONFIG_0
#define QMP6990_FIFO_PRESS_EN__MSK    0x08
#define QMP6990_FIFO_PRESS_EN__POS    3

/* FIFO MODE bit */
#define QMP6990_FIFO_MODE__REG    QMP6990_REG_FIFO_CONFIG_0
#define QMP6990_FIFO_MODE__MSK    0x02
#define QMP6990_FIFO_MODE__POS    1

/* FIFO EN bit */
#define QMP6990_FIFO_EN__REG    QMP6990_REG_FIFO_CONFIG_0
#define QMP6990_FIFO_EN__MSK    0x01
#define QMP6990_FIFO_EN__POS    0

/* DATA SEL bit */
#define QMP6990_DATA_SEL__REG    QMP6990_REG_FIFO_CONFIG_1
#define QMP6990_DATA_SEL__MSK    0x08
#define QMP6990_DATA_SEL__POS    3

/* FIFO INT TYPE bit */
#define QMP6990_FIFO_INT_TYPE__REG    QMP6990_REG_INT_CTRL
#define QMP6990_FIFO_INT_TYPE__MSK    0x80
#define QMP6990_FIFO_INT_TYPE__POS    7

/* DRDY EN bit */
#define QMP6990_DRDY_EN__REG    QMP6990_REG_INT_CTRL
#define QMP6990_DRDY_EN__MSK    0x40
#define QMP6990_DRDY_EN__POS    6

/* FFULL EN bit */
#define QMP6990_FFULL_EN__REG    QMP6990_REG_INT_CTRL
#define QMP6990_FFULL_EN__MSK    0x10
#define QMP6990_FFULL_EN__POS    4

/* FWTM EN bit */
#define QMP6990_FWTM_EN__REG    QMP6990_REG_INT_CTRL
#define QMP6990_FWTM_EN__MSK    0x08
#define QMP6990_FWTM_EN__POS    3

/* INT LV bit */
#define QMP6990_INT_LV__REG    QMP6990_REG_INT_CTRL
#define QMP6990_INT_LV__MSK    0x02
#define QMP6990_INT_LV__POS    1

/* INT OD bit */
#define QMP6990_INT_OD__REG    QMP6990_REG_INT_CTRL
#define QMP6990_INT_OD__MSK    0x01
#define QMP6990_INT_OD__POS    0

/* I2C WDT SEL bit */
#define QMP6990_I2C_WDT_SEL__REG    QMP6990_REG_CONFIG
#define QMP6990_I2C_WDT_SEL__MSK    0x04
#define QMP6990_I2C_WDT_SEL__POS    2

/* I2C WDT EN bit */
#define QMP6990_I2C_WDT_EN__REG    QMP6990_REG_CONFIG
#define QMP6990_I2C_WDT_EN__MSK    0x02
#define QMP6990_I2C_WDT_EN__POS    1

/* SPI3 EN bit */
#define QMP6990_SPI3_EN__REG    QMP6990_REG_CONFIG
#define QMP6990_SPI3_EN__MSK    0x01
#define QMP6990_SPI3_EN__POS    0

/* PWR MODE bit */
#define QMP6990_PWR_MODE__REG    QMP6990_REG_PWR_CTRL
#define QMP6990_PWR_MODE__MSK    0x30
#define QMP6990_PWR_MODE__POS    4

/* TEMP EN bit */
#define QMP6990_TEMP_EN__REG    QMP6990_REG_PWR_CTRL
#define QMP6990_TEMP_EN__MSK    0x02
#define QMP6990_TEMP_EN__POS    1

/* PRESS EN bit */
#define QMP6990_PRESS_EN__REG    QMP6990_REG_PWR_CTRL
#define QMP6990_PRESS_EN__MSK    0x01
#define QMP6990_PRESS_EN__POS    0

typedef enum {
  QMP6990_SLEEP_MODE 	= 0,
  QMP6990_FORCED_MODE 	= 1,
  QMP6990_NORMAL_MODE 	= 3,
} QMP6990_PWR_MODE_Type;

typedef enum {
  QMP6990_OSR_x1		= 0,
  QMP6990_OSR_x2		= 1,
  QMP6990_OSR_x4		= 2,
  QMP6990_OSR_x8		= 3,
  QMP6990_OSR_x16	= 4,
  QMP6990_OSR_x32	= 5,
  QMP6990_OSR_x64	= 6,
  QMP6990_OSR_x128	= 7,
} QMP6990_OSR_PT_Type;

typedef enum {
  QMP6990_ODR_4ms	    = 0x00,
  QMP6990_ODR_4_5ms	    = 0x01,
  QMP6990_ODR_5ms	    = 0x02,
  QMP6990_ODR_5_556ms    = 0x03,
  QMP6990_ODR_6_25ms     = 0x04,
  QMP6990_ODR_7ms	    = 0x05,
  QMP6990_ODR_8ms	    = 0x06,
  QMP6990_ODR_9ms	    = 0x07,
  QMP6990_ODR_10ms	    = 0x08,
  QMP6990_ODR_11_1ms	    = 0x09,
  QMP6990_ODR_12_5ms	    = 0x0A,
  QMP6990_ODR_14_286ms	= 0x0B,
  QMP6990_ODR_16ms	    = 0x0C,
  QMP6990_ODR_20ms	    = 0x0D,
  QMP6990_ODR_25ms	    = 0x0E,
  QMP6990_ODR_31_25ms    = 0x0F,
  QMP6990_ODR_40ms       = 0x10,
  QMP6990_ODR_50ms       = 0x11,
  QMP6990_ODR_62_5ms     = 0x12,
  QMP6990_ODR_80ms       = 0x13,
  QMP6990_ODR_100ms      = 0x14,
  QMP6990_ODR_125ms      = 0x15,
  QMP6990_ODR_156_25ms   = 0x16,
  QMP6990_ODR_200ms      = 0x17,
  QMP6990_ODR_250ms      = 0x18,
  QMP6990_ODR_312_5ms    = 0x19,
  QMP6990_ODR_400ms      = 0x1A,
  QMP6990_ODR_500ms      = 0x1B,
  QMP6990_ODR_1000ms     = 0x1C,
  QMP6990_ODR_2000ms     = 0x1D,
  QMP6990_ODR_4000ms     = 0x1E,
  QMP6990_ODR_8000ms     = 0x1F,
} QMP6990_ODR_SEL_Type;

typedef enum {
  QMP6990_FILTER_COE_T_0	     = 0,
  QMP6990_FILTER_COE_T_1	     = 1,
  QMP6990_FILTER_COE_T_3	     = 2,
  QMP6990_FILTER_COE_T_7	     = 3,
  QMP6990_FILTER_COE_T_15     = 4,
  QMP6990_FILTER_COE_T_31     = 5,
  QMP6990_FILTER_COE_T_63     = 6,
  QMP6990_FILTER_COE_T_127    = 7,
} QMP6990_IIR_COEF_T_Type;

typedef enum {
  QMP6990_FILTER_COE_P_0	     = 0,
  QMP6990_FILTER_COE_P_1	     = 1,
  QMP6990_FILTER_COE_P_3	     = 2,
  QMP6990_FILTER_COE_P_7	     = 3,
  QMP6990_FILTER_COE_P_15     = 4,
  QMP6990_FILTER_COE_P_31     = 5,
  QMP6990_FILTER_COE_P_63     = 6,
  QMP6990_FILTER_COE_P_127    = 7,
} QMP6990_IIR_COEF_P_Type;

#define qmp6990_get_bitslice(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define qmp6990_set_bitslice(regvar, bitname, val)	\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS) & bitname##__MSK))

/* --------------------------------------------------------------------- */
//

#ifdef __cplusplus
extern "C" {
#endif

  u8 qmp6990_burst_read(u8 u8Addr, u8 *pu8Data, u8 u8Len);
  u8 qmp6990_burst_write(u8 u8Addr, u8 *pu8Data, u8 u8Len);

#ifdef SENSOR_FIFO_FUNC

  u8 qmp6990_fifo_flush_process(void);  
  u8 qmp6990_read_fifo_data(u8* fifo, int* fifo_byte);
#endif  
  u8 qmp6990_into_sleep_mode(void);
  u8 qmp6990_software_reset_process(void);
  u8 qmp6990_enable_PT_meas(void);
  u8 qmp6990_disable_PT_meas(void);
  u8 qmp6990_initialization(void);
  u8 qmp6990_MEMS_T_init(void);
  u8 qmp6990_measure_PT_data(s32* s32P, s32* s32T);

  u8 qmp6990_setup(void);
  void qmp6990_measure_data(float *pa, float *temp);
#ifdef __cplusplus
}   // extern "C"
#endif

#endif // __QMP6990_H__
