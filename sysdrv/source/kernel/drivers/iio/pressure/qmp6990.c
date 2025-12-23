/*****************************************************************************
 *
 * File : qmp6990_test.cpp
 *
 * Date : 2022/06/09
 *
 ****************************************************************************/
//
#include "qmp6990.h"
#include "qst_sw_i2c.h"
/* --------------------------------------------------------------------- */


#if debug_mode
#define QMP6990_LOG		printf
#else
#define QMP6990_LOG(fmt, args...)	
#endif

static s32 s32P, s32T;
static float fT_Celsius, fP_Pa;

#ifdef SENSOR_FIFO_FUNC
u8 fifo[576] = {0};
static u16 fifo_cnt;
#endif

static void delayms(u32 time)//ms
{
	extern void delay_ms(unsigned int n_ms);

	delay_ms(time);
}

/*****************************************************************************
 *
 * Function : qmp6990_burst_read
 *
 * @input : [u8Addr] -> register address
 *          [u8Len]  -> data length
 *
 * @output : [*pu8Data] -> data buffer 
 *
 * @return : 0 -> success
 *           other numbers(rc) -> receive wrong data
 *
 ****************************************************************************/
//
u8 qmp6990_burst_read(u8 u8Addr, u8 *pu8Data, u8 u8Len)
{
	s8 rc;

	/* ----- modify i2c read function at here ----- */ 
	rc = qst_sw_readreg(QMP6990_8BIT_I2C_ADDR, u8Addr, pu8Data, u8Len);

	if(I2C_SUCCESS == rc)
	{
#if 0//debug_mode
		for(int i = 0;i<u8Len;i++)
		{		
			QMP6990_LOG("qmp6990_burst_read rc = %d, u8Addr = 0x%x,pu8Data[%d] = 0x%x \n\r",rc,u8Addr+i,i,pu8Data[i]);
		}
#endif
		return 0;
	}
	else
	{	
		return u8Len;
	}	
}

/*****************************************************************************
 *
 * Function : qmp6990_burst_write
 *
 * @input : [u8Addr]   -> register address 
 *          [*pu8Data] -> data buffer 
 *          [u8Len]    -> data length
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           other numbers(rc) -> receive wrong data
 *
 ****************************************************************************/
//
u8 qmp6990_burst_write(u8 u8Addr, u8 *pu8Data, u8 u8Len)
{
	s8 rc,i;

	/* ----- modify i2c write function at here ----- */ 	
	for(i = 0;i<u8Len;i++)
	{		
		rc = qst_sw_writereg(QMP6990_8BIT_I2C_ADDR, u8Addr+i, pu8Data[i]);
#if 0 //debug_mode
		QMP6990_LOG("qmp6990_burst_write rc = %d, u8Addr = 0x%x,pu8Data[%d] = 0x%x \n\r",rc,u8Addr+i,i,pu8Data[i]);
#endif
	}
	if(I2C_SUCCESS == rc)
	{
		return 0;
	}
	else
	{	
		return u8Len;
	}

}


void qmp6990_dump_reg(void)
{
	u8 rc, u8Data;

	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_FIFO_CONFIG_1, &u8Data, 1);
	if (rc != 0)	return;
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_FIFO_CONFIG_1, u8Data);

	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_INT_CTRL, &u8Data, 1);
	if (rc != 0)	return;  
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_INT_CTRL, u8Data);

	/*****  (Reg)OSR, oversampling ratio (P & T)	*****/
	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_OSR, &u8Data, 1);
	if (rc != 0)	return;  
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_OSR, u8Data);

	/*****  (Reg)ODR, output data rate (25Hz) *****/
	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_ODR, &u8Data, 1);
	if (rc != 0)	return;  
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_ODR, u8Data);

	/*****  (Reg)FILTER, IIR-filter_coe_P: 0, IIR-filter_coe_T: 0,  *****/
	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_FILTER, &u8Data, 1);
	if (rc != 0)	return;
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_FILTER, u8Data);

	u8Data = 0x00;
	rc = qmp6990_burst_read(QMP6990_REG_PWR_CTRL, &u8Data, 1);
	if (rc != 0)	return;
	QMP6990_LOG("dump 0x%02x = 0x%02x\r\n", QMP6990_REG_PWR_CTRL, u8Data);	
}

#ifdef SENSOR_FIFO_FUNC
/*****************************************************************************
 *
 * Function : gmp108a_fifo_flush_process
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_fifo_flush_process(void)
{
  u8 rc, u8Data;

  u8Data = 0xB0;
  rc = qmp6990_burst_write(QMP6990_REG_RESET, &u8Data, 1);
  
  return (rc);
}
#endif

/*****************************************************************************
 *
 * Function : qmp6990_into_sleep_mode
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_into_sleep_mode(void)
{
  u8 rc, u8Data;

  /*****  Disable pressure & temperature sensor *****/
  u8Data = 0x00;
  rc = qmp6990_burst_write(QMP6990_REG_PWR_CTRL, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_software_reset_process
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_software_reset_process(void)
{
  u8 rc, u8Data;

  u8Data = 0xB6;
  rc = qmp6990_burst_write(QMP6990_REG_RESET, &u8Data, 1);
  delayms(100);	// wait 100msec
  
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_enable_PT_meas
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_enable_PT_meas(void)
{
  u8 rc, u8Data;

  /*****  Enable pressure & temperature sensor *****/
  u8Data = 0x33;
  rc = qmp6990_burst_write(QMP6990_REG_PWR_CTRL, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_enable_T_meas
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_enable_T_meas(void)
{
  u8 rc, u8Data;

  /*****  Enable temperature sensor *****/
  u8Data = 0x32;
  rc = qmp6990_burst_write(QMP6990_REG_PWR_CTRL, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_disable_PT_meas
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_disable_PT_meas(void)
{
  u8 rc, u8Data;

  /*****  Disable pressure & temperature sensor *****/
  u8Data = 0x30;
  rc = qmp6990_burst_write(QMP6990_REG_PWR_CTRL, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_initialization
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_initialization(void)
{
  u8 rc, u8Data;
  
  /*****  check sensor version *****/
  rc = qmp6990_burst_read(QMP6990_REG_REV_ID, &u8Data, 1);
  if (rc != 0)  return (rc);
  QMP6990_LOG("QMP6990_REG_REV_ID = 0x%x\n\r",u8Data);
  
  /*****  check sensor ID *****/
  rc = qmp6990_burst_read(QMP6990_REG_CHIP_ID, &u8Data, 1);
  if (rc != 0)  return (rc);
  QMP6990_LOG("QMP6990_REG_CHIP_ID = 0x%x\n\r",u8Data);
    
  /*****  (Reg)FIFO_CONFIG_1, enable filtered data, subsampling interval: 0,  *****/
  u8Data = 0x00;
  rc = qmp6990_burst_write(QMP6990_REG_FIFO_CONFIG_1, &u8Data, 1);
  if (rc != 0)  return (rc);  

#ifdef SENSOR_FIFO_FUNC
  /*****  (Reg)INT_CTRL, enable Watermark & FIFO full interrupt,  *****/
  u8Data = 0x30;
#else 
  /*****  (Reg)INT_CTRL, enable data ready interrupt,  *****/
  u8Data = 0x40;
#endif
  rc = qmp6990_burst_write(QMP6990_REG_INT_CTRL, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  /*****  (Reg)OSR, oversampling ratio (P & T)  *****/
  u8Data = 0x33;
  rc = qmp6990_burst_write(QMP6990_REG_OSR, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  /*****  (Reg)ODR, output data rate (25Hz) *****/
  u8Data = 0x16;
  rc = qmp6990_burst_write(QMP6990_REG_ODR, &u8Data, 1);
  if (rc != 0)  return (rc);  
  
  /*****  (Reg)FILTER, IIR-filter_coe_P: 0, IIR-filter_coe_T: 0,  *****/
  u8Data = 0x22;	// filter-coes: 0
  rc = qmp6990_burst_write(QMP6990_REG_FILTER, &u8Data, 1);
  if (rc != 0)  return (rc);
  
  QMP6990_LOG("qmp6990_initialization OK\n\r");
  
  /*****  (Reg)FILGAIN, CIC filter Gain x16,  *****/
/*															   
  u8Data = 0x40;
  rc = qmp6990_burst_write(QMP6990_REG_FILGAIN, &u8Data, 1);
  if (rc != 0)  return (rc);
  
  u8Data = 0xA0;  // ENG (CP trim)
  rc = qmp6990_burst_write(0x4A, &u8Data, 1);
  if (rc != 0)  return (rc);
*/
#ifdef SENSOR_FIFO_FUNC
   /*****  (Reg)FIFO_WM_0, watermark level: 0x00FF (255 bytes)	*****/
	u8Data = 0xFF;
	rc = qmp6990_burst_write(QMP6990_REG_FIFO_WM_0, &u8Data, 1);
	if (rc != 0)  return (rc);
	u8Data = 0x00;
	rc = qmp6990_burst_write(QMP6990_REG_FIFO_WM_1, &u8Data, 1);
	if (rc != 0)  return (rc);
  
  /*****  (Reg)FIFO_CONFIG_0, store: temp/pressure, FIFO normal mode, FIFO enable  *****/
	u8Data = 0x1B;
	rc = qmp6990_burst_write(QMP6990_REG_FIFO_CONFIG_0, &u8Data, 1);
	if (rc != 0)  return (rc); 
#endif

  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_MEMS_T_init
 *
 * @input : N/A
 *
 * @output : N/A
 *
 * @return : 0 -> success
 *           others number -> error 
 *
 ****************************************************************************/
//
u8 qmp6990_MEMS_T_init(void)
{
  u8 rc, u8Data;
  
  u8Data = 0x00;  // Choose T0 TRIM config
  rc = qmp6990_burst_write(0x33, &u8Data, 1);
  if (rc != 0)  return (rc);
   
  u8Data = 0x00;  // disable BG temperature sensor
  rc = qmp6990_burst_write(0x46, &u8Data, 1);
  if (rc != 0)  return (rc);
  
  /*****  (Reg)AGAIN, Tch ADC gain x2, Pch ADC gain x1, PGA gain x16,  *****/
  u8Data = 0x18;
  rc = qmp6990_burst_write(QMP6990_REG_AGAIN, &u8Data, 1);
  if (rc != 0)  return (rc);
  
  u8Data = 0x27;  // Choose MEMS temperature sensor / enable ADC
  rc = qmp6990_burst_write(0x4B, &u8Data, 1);
  
  u8Data = 0x34;  // RT = 5Kohm
  rc = qmp6990_burst_write(0x4C, &u8Data, 1);
  if (rc != 0)  return (rc);
 
  return (rc);
}

/*****************************************************************************
 *
 * Function : qmp6990_measure_PT_data
 *
 * @input : N/A
 *
 * @output : [s32P] -> pressure value
 *           [s32T] -> temperature value
 *
 * @return : 0 -> success
 *          -1 -> I2C transmission error
 *          -2 -> avoid command
 *
 ****************************************************************************/
//
u8 qmp6990_measure_PT_data(s32* s32P, s32* s32T)
{
  u8 rc, cnt1;
  u8 u8Data[6] = {0};
   
  /*****  Read pressure & temperature data *****/
  cnt1 = 0;
  
  do {
	cnt1++;
	delayms(polling_time);
    rc = qmp6990_burst_read(QMP6990_REG_STATUS, u8Data, 1);
	if (rc != 0)  return (rc);
  } while (((u8Data[0] & 0x60) != 0x60) && (cnt1 <= 10));
    
  if (cnt1 > 10) {
	QMP6990_LOG("qmp6990 measure timeout = %d\n\r",cnt1);
    return (rc);
  }
	
  rc = qmp6990_burst_read(QMP6990_REG_PRESS_XLSB, u8Data, 6);
  if (rc != 0)  return (rc);
  *s32P = ((u32)(u8Data[2]) << 16) + ((u16)(u8Data[1]) << 8) + u8Data[0]; 
  if (*s32P >= 0x800000) {
    *s32P = ((s64) (*s32P)) - 0x01000000;
  }
 
  *s32T = ((u32)(u8Data[5]) << 16) + ((u16)(u8Data[4]) << 8) + u8Data[3];
  if (*s32T >= 0x800000) {
    *s32T = ((s64) (*s32T)) - 0x01000000;
  }

  return (rc);
}

#ifdef SENSOR_FIFO_FUNC
/*****************************************************************************
 *
 * Function : gmp108a_read_fifo_data
 *
 * @input : N/A
 *
 * @output : [fifo] -> FIFO data
 *           [fifo_byte] -> FIFO data counts
 *
 * @return : 0 -> success
 *          -1 -> I2C transmission error
 *          -2 -> avoid command
 *
 ****************************************************************************/
//
u8 qmp6990_read_fifo_data(u8* fifo, int* fifo_byte)
{
  u8 rc, cnt1 = 0;
  u8 u8Data[2] = {0};
  
  rc = qmp6990_burst_read(QMP6990_REG_FIFO_LENGTH_0, u8Data, 2);
  if (rc != 0)  return (rc);
   
  *fifo_byte = ((u16)(u8Data[1] & 0x03) << 8) + u8Data[0];

  if (*fifo_byte) {
    rc = qmp6990_burst_read(QMP6990_REG_FIFO_DATA, fifo, *fifo_byte);
  }

  return (rc);
}
#endif

void qmp6990_measure_data(float *pa, float *temp) 
{
  u8 rc;

#ifdef SENSOR_FIFO_FUNC
	rc = qmp6990_read_fifo_data(fifo, &fifo_cnt);
#else
	/*****  read pressure and temperature data  *****/   
	rc = qmp6990_measure_PT_data(&s32P, &s32T);	

	/*****	Code to Pa and Celsius	*****/	 
	fP_Pa = QMP6990_P_CODE_TO_PA(s32P);
	fT_Celsius = QMP6990_T_CODE_TO_CELSIUS(s32T);
   *pa = fP_Pa;
   *temp = fT_Celsius;
	//QMP6990_LOG("qmp6990 fP_Pa = %f\n\r",fP_Pa);
	//QMP6990_LOG("qmp6990 fT_Celsius = %f\n\r",fT_Celsius);

#endif

  if (rc != 0)
  {
	QMP6990_LOG("qmp6990 measure_sensor_PT_data rc = %d\n\r",rc);
    return;
  }	 

#ifdef SENSOR_FIFO_FUNC
  if (fifo_cnt)
  {
	u8 i=0;
	for (i = 0; i < (fifo_cnt / 6); i++) 
	{
      s32T = ( ((u32)(fifo[i * 6 + 2]) << 16) | 
	           ((u16)(fifo[i * 6 + 1]) << 8) | 
			          fifo[i * 6] );
      s32P = ( ((u32)(fifo[i * 6 + 5]) << 16) | 
	           ((u16)(fifo[i * 6 + 4]) << 8) | 
			          fifo[i * 6 + 3] );
					  
      if (s32T >= 0x800000)
	  {
        s32T = ((s64) (s32T)) - 0x01000000;
      }
	  if (s32P >= 0x800000)
	  {
        s32P = ((s64) (s32P)) - 0x01000000;
      }
	  

	  /*****  Code to Pa and Celsius  *****/   
	  fP_Pa = QMP6990_P_CODE_TO_PA(s32P);
	  fT_Celsius = QMP6990_T_CODE_TO_CELSIUS(s32T);
	  QMP6990_LOG("FIFO :qmp6990 fP_Pa[%d] = %f\n\r",i,fP_Pa);
	  QMP6990_LOG("FIFO :qmp6990 fT_Celsius[%d] = %f\n\r",i,fT_Celsius);
    }  
  } 
#endif  
}  


u8 qmp6990_setup(void) 
{
  u8 rc;

  QMP6990_LOG(" qmp6990 initialization begin\n\r");
  /***** qmp6990 initialization setup *****/
  rc = qmp6990_software_reset_process();
  // if (rc != 0)  goto err_init;
  
  rc = qmp6990_initialization();
  if (rc != 0)  goto err_init;

#ifdef SENSOR_FIFO_FUNC
	rc = qmp6990_fifo_flush_process();
	if (rc != 0) goto err_init;
#endif

#if  0
  /***** qmp6990 temperature setup *****/
  rc = qmp6990_MEMS_T_init();
  if (rc != 0)  goto err_init;
#endif   
  /***** qmp6990 enable PT func  *****/
  rc = qmp6990_enable_PT_meas();
  if (rc != 0)  goto err_init;
#if debug_mode 
  qmp6990_dump_reg();
#endif
  return 1;

err_init:    
  QMP6990_LOG("qmp6990 initial process erro.\n\r");
	return 0;
}


