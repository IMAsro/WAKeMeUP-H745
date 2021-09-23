/*
 * sensor.h
 *
 *  Created on: Nov 13, 2020
 *      Author: MICHAL_MOUCKA
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "stm32h7xx_hal.h"


//HDC1080 - Digital Humidity Sensor with Temperature Sensor
static const uint8_t  HDC1080_AD 		 									__attribute__((unused))	= (0x40<<1);	//base address
static const uint8_t  HDC1080_RG_TEMPERATURE 					__attribute__((unused))	= 0x00;				//temperature register
static const uint8_t	HDC1080_RG_HUMIDITY 						__attribute__((unused))	= 0x01;				//humidity register
static const uint8_t	HDC1080_RG_CONFIG								__attribute__((unused)) = 0x02;				//configuration register
static const uint8_t	HDC1080_RG_SERIAL_ID_F 					__attribute__((unused))	= 0xFB; 			//serial ID device dependent First 2 bytes of the serial ID of the part
static const uint8_t	HDC1080_RG_SERIAL_ID_M					__attribute__((unused))	= 0xFC;				//serial ID device dependent Mid 2 bytes of the serial ID of the part
static const uint8_t	HDC1080_RG_SERIAL_ID_L					__attribute__((unused))	= 0xFD; 			//serial ID device dependent Last byte bit of the serial ID of the part
static const uint8_t	HDC1080_RG_MANUFAC_ID						__attribute__((unused))	= 0xFE; 			//manufacturer ID
static const uint8_t	HDC1080_RG_PART_ID							__attribute__((unused))	= 0xFF; 			//ID 0x1050 ID of the device

//LPS25HB - Pressure sensor absolute
static const uint8_t  LPS25HB_AD 		 									__attribute__((unused))	= (0x5C<<1);	//base address
static const uint8_t	LPS25HB_RG_REF_P_XL 						__attribute__((unused))	= 0x08;				//reference pressure (LSB)
static const uint8_t	LPS25HB_RG_REF_P_L 							__attribute__((unused))	= 0x09;				//reference pressure (middle part)
static const uint8_t	LPS25HB_RG_REF_P_H 							__attribute__((unused))	= 0x0A;				//reference pressure (MSB data)
static const uint8_t	LPS25HB_RG_WHO_AM_I 						__attribute__((unused))	= 0x0F;				//device who am I
static const uint8_t	LPS25HB_RG_RES_CONF 						__attribute__((unused))	= 0x10;				//pressure and temperature resolution
static const uint8_t	LPS25HB_RG_CTRL_REG1 						__attribute__((unused))	= 0x20;				//control register 1
static const uint8_t	LPS25HB_RG_CTRL_REG2 						__attribute__((unused))	= 0x21;				//control register 2
static const uint8_t	LPS25HB_RG_CTRL_REG3 						__attribute__((unused))	= 0x22;				//control register 3
static const uint8_t	LPS25HB_RG_CTRL_REG4 						__attribute__((unused))	= 0x23;				//control register 4
static const uint8_t	LPS25HB_RG_INTERRUPT_CFG 				__attribute__((unused))	= 0x24;				//interrupt configuration
static const uint8_t	LPS25HB_RG_INT_SOURCE 					__attribute__((unused))	= 0x25;				//interrupt source
static const uint8_t	LPS25HB_RG_STATUS_REG 					__attribute__((unused))	= 0x27;				//status register
static const uint8_t	LPS25HB_RG_PRESS_OUT_XL 				__attribute__((unused))	= 0x28;				//pressure output (LSB)
static const uint8_t	LPS25HB_RG_PRESS_OUT_L 					__attribute__((unused))	= 0x29;				//pressure output value (mid part)
static const uint8_t	LPS25HB_RG_PRESS_OUT_H 					__attribute__((unused))	= 0x2A;				//pressure output value (MSB)
static const uint8_t	LPS25HB_RG_TEMP_OUT_L 					__attribute__((unused))	= 0x2B;				//temperature output value (LSB)
static const uint8_t	LPS25HB_RG_TEMP_OUT_H 					__attribute__((unused))	= 0x2C;				//temperature output value (LSB)
static const uint8_t	LPS25HB_RG_FIFO_CTRL 						__attribute__((unused))	= 0x2E;				//FIFO control
static const uint8_t	LPS25HB_RG_FIFO_STATUS 					__attribute__((unused))	= 0x2F;				//FIFO status
static const uint8_t	LPS25HB_RG_THS_P_L 							__attribute__((unused))	= 0x30;				//least significant bits of the threshold value for pressure interrupt generation
static const uint8_t	LPS25HB_RG_THS_P_H 							__attribute__((unused))	= 0x31;				//most significant bits of the threshold value for pressure interrupt generation
static const uint8_t	LPS25HB_RG_RPDS_L 							__attribute__((unused))	= 0x39;				//pressure offset (LSB data)
static const uint8_t	LPS25HB_RG_RPDS_H 							__attribute__((unused))	= 0x3A;				//pressure offset (MSB data)

//LTR303ALS01 Digital Ambient Light Sensor
static const uint8_t  LTR303ALS01_AD 									__attribute__((unused))	= (0x29<<1);	//base address
static const uint8_t	LTR303ALS01_RG_ALS_CONFIG				__attribute__((unused))	= 0x80;				//operation mode control SW  reset
static const uint8_t  LTR303ALS01_RG_ALS_MEAS_RATEALS __attribute__((unused))	=	0x85; 			//measurement rate in active mode
static const uint8_t  LTR303ALS01_RG_PART_ID 					__attribute__((unused))	= 0x86;				//part Number ID and Revision ID
static const uint8_t  LTR303ALS01_RG_MANUFAC_ID				__attribute__((unused))	= 0x87;				//manufacturer ID
static const uint8_t  LTR303ALS01_RG_ALS_DATA_CH1_0		__attribute__((unused))	= 0x88;				//ALS measurement CH1 data, lower byte
static const uint8_t  LTR303ALS01_RG_ALS_DATA_CH1_1		__attribute__((unused))	= 0x89;				//ALS measurement CH1 data, upper byte
static const uint8_t  LTR303ALS01_RG_ALS_DATA_CH0_0		__attribute__((unused))	= 0x8A;				//ALS measurement CH0 data, lower byte
static const uint8_t  LTR303ALS01_RG_ALS_DATA_CH0_1		__attribute__((unused))	= 0x8B;				//ALS measurement CH0 data, upper byte
static const uint8_t  LTR303ALS01_RG_ALS_STATUS 			__attribute__((unused))	=	0x8C;				//ALS new data status
static const uint8_t  LTR303ALS01_RG_INTERRUPT 				__attribute__((unused))	=	0x8F; 			//interrupt settings
static const uint8_t  LTR303ALS01_RG_ALS_THRES_UP_0 	__attribute__((unused))	=	0x97; 			//ALS interrupt upper threshold, lower byte
static const uint8_t  LTR303ALS01_RG_ALS_THRES_UP_1 	__attribute__((unused))	=	0x98;				//ALS interrupt upper threshold, upper byte
static const uint8_t  LTR303ALS01_RG_ALS_THRES_LOW_0 	__attribute__((unused))	=	0x99;				//ALS interrupt lower threshold, lower byte
static const uint8_t  LTR303ALS01_RG_ALS_THRES_LOW_1 	__attribute__((unused))	=	0x9A; 			//ALS interrupt lower threshold, upper byte
static const uint8_t  LTR303ALS01_RGINTERRUPT_PERSIST	__attribute__((unused))	=	0x9E; 			//ALS	interrupt persist setting


//LSM9DS1 - 3D accelerometer, 3D gyroscope, 3D magnetometer
static const uint8_t  LSM9DS1_AG_AD 									__attribute__((unused))	= (0x6B<<1);	//accelerometer base address
static const uint8_t  LSM9DS1_MA_AD 									__attribute__((unused))	= (0x1E<<1);	//magnetometer base address
static const uint8_t  LSM9DS1_AG_RG_ACT_THS						__attribute__((unused))	=	0x04;
static const uint8_t  LSM9DS1_AG_RG_ACT_DUR						__attribute__((unused))	= 0x05;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_CFG_XL		__attribute__((unused))	= 0x06;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_X_XL	__attribute__((unused))	= 0x07;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_Y_XL 	__attribute__((unused))	= 0x08;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_Z_XL 	__attribute__((unused))	= 0x09;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_DUR_XL		__attribute__((unused))	= 0x0A;
static const uint8_t  LSM9DS1_AG_RG_REFERENCE_G 			__attribute__((unused))	= 0x0B;
static const uint8_t  LSM9DS1_AG_RG_INT1_CTRL 				__attribute__((unused))	= 0x0C;
static const uint8_t  LSM9DS1_AG_RG_INT2_CTRL 				__attribute__((unused))	= 0x0D;
static const uint8_t  LSM9DS1_AG_RG_WHO_AM_I 					__attribute__((unused))	= 0x0F;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG1_G 			__attribute__((unused))	= 0x10;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG2_G 			__attribute__((unused))	= 0x11;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG3_G				__attribute__((unused))	= 0x12;
static const uint8_t  LSM9DS1_AG_RG_ORIENT_CFG_G			__attribute__((unused))	= 0x13;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_SRC_G			__attribute__((unused))	= 0x14;
static const uint8_t  LSM9DS1_AG_RG_OUT_TEMP_L				__attribute__((unused))	= 0x15;
static const uint8_t  LSM9DS1_AG_RG_OUT_TEMP_H				__attribute__((unused))	= 0x16;
static const uint8_t  LSM9DS1_AG_RG_STATUS_REG1				__attribute__((unused))	= 0x17;
static const uint8_t  LSM9DS1_AG_RG_OUT_X_L_G					__attribute__((unused))	= 0x18;
static const uint8_t  LSM9DS1_AG_RG_OUT_X_H_G					__attribute__((unused))	= 0x19;
static const uint8_t  LSM9DS1_AG_RG_OUT_Y_L_G					__attribute__((unused))	= 0x1A;
static const uint8_t  LSM9DS1_AG_RG_OUT_Y_H_G					__attribute__((unused))	= 0x1B;
static const uint8_t  LSM9DS1_AG_RG_OUT_Z_L_G					__attribute__((unused))	= 0x1C;
static const uint8_t  LSM9DS1_AG_RG_OUT_Z_H_G					__attribute__((unused))	= 0x1D;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG4					__attribute__((unused))	= 0x1E;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG5_XL			__attribute__((unused))	= 0x1F;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG6_XL			__attribute__((unused))	= 0x20;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG7_XL			__attribute__((unused))	= 0x21;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG8					__attribute__((unused))	= 0x22;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG9					__attribute__((unused))	= 0x23;
static const uint8_t  LSM9DS1_AG_RG_CTRL_REG10				__attribute__((unused))	= 0x24;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_SRC_XL		__attribute__((unused))	= 0x26;
static const uint8_t  LSM9DS1_AG_RG_STATUS_REG2				__attribute__((unused))	= 0x27;
static const uint8_t  LSM9DS1_AG_RG_OUT_X_L_XL				__attribute__((unused))	= 0x28;
static const uint8_t  LSM9DS1_AG_RG_OUT_X_H_XL				__attribute__((unused))	= 0x29;
static const uint8_t  LSM9DS1_AG_RG_OUT_Y_L_XL				__attribute__((unused))	= 0x2A;
static const uint8_t  LSM9DS1_AG_RG_OUT_Y_H_XL				__attribute__((unused))	= 0x2B;
static const uint8_t  LSM9DS1_AG_RG_OUT_Z_L_XL				__attribute__((unused))	= 0x2C;
static const uint8_t  LSM9DS1_AG_RG_OUT_Z_H_XL				__attribute__((unused))	= 0x2D;
static const uint8_t  LSM9DS1_AG_RG_FIFO_CTRL					__attribute__((unused))	= 0x2E;
static const uint8_t  LSM9DS1_AG_RG_FIFO_SRC					__attribute__((unused))	= 0x2F;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_CFG_G			__attribute__((unused))	= 0x30;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_XH_G	__attribute__((unused))	= 0x31;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_XL_G	__attribute__((unused))	= 0x32;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_YH_G	__attribute__((unused))	= 0x33;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_YL_G	__attribute__((unused))	= 0x34;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_ZH_G	__attribute__((unused))	= 0x35;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_THS_ZL_G	__attribute__((unused))	= 0x36;
static const uint8_t  LSM9DS1_AG_RG_INT_GEN_DUR_G			__attribute__((unused))	= 0x37;
static const uint8_t  LSM9DS1_MA_RG_OFFSET_X_REG_L_M	__attribute__((unused))	= 0x05;				//offset in order to compensate environmental effects
static const uint8_t  LSM9DS1_MA_RG_OFFSET_X_REG_H_M	__attribute__((unused))	= 0x06;
static const uint8_t  LSM9DS1_MA_RG_OFFSET_Y_REG_H_M	__attribute__((unused))	= 0x08;
static const uint8_t  LSM9DS1_MA_RG_OFFSET_Z_REG_L_M	__attribute__((unused))	= 0x09;
static const uint8_t  LSM9DS1_MA_RG_OFFSET_Z_REG_H_M	__attribute__((unused))	= 0x0A;
static const uint8_t  LSM9DS1_MA_RG_WHO_AM_I_M				__attribute__((unused))	= 0x0F;				//magnetic Who I am ID
static const uint8_t  LSM9DS1_MA_RG_CTRL_REG1_M				__attribute__((unused))	= 0x20; 			//magnetic control registers
static const uint8_t  LSM9DS1_MA_RG_CTRL_REG2_M				__attribute__((unused))	= 0x21;
static const uint8_t  LSM9DS1_MA_RG_CTRL_REG3_M				__attribute__((unused))	= 0x22;
static const uint8_t  LSM9DS1_MA_RG_CTRL_REG4_M				__attribute__((unused))	= 0x23;
static const uint8_t  LSM9DS1_MA_RG_CTRL_REG5_M				__attribute__((unused))	= 0x24;
static const uint8_t  LSM9DS1_MA_RG_STATUS_REG_M			__attribute__((unused))	= 0x27; 			//magnetic output registers
static const uint8_t  LSM9DS1_MA_RG_OUT_X_L_M					__attribute__((unused))	= 0x28;
static const uint8_t  LSM9DS1_MA_RG_OUT_X_H_M					__attribute__((unused))	= 0x29;
static const uint8_t  LSM9DS1_MA_RG_OUT_Y_L_M					__attribute__((unused))	= 0x2A;
static const uint8_t  LSM9DS1_MA_RG_OUT_Y_H_M					__attribute__((unused))	= 0x2B;
static const uint8_t  LSM9DS1_MA_RG_OUT_Z_L_M					__attribute__((unused))	= 0x2C;
static const uint8_t  LSM9DS1_MA_RG_OUT_Z_H_M					__attribute__((unused))	= 0x2D;
static const uint8_t  LSM9DS1_MA_RG_INT_CFG_M					__attribute__((unused))	= 0x30; 			//magnetic interrupt configuration register
static const uint8_t  LSM9DS1_MA_RG_INT_SRC_M 				__attribute__((unused))	= 0x31; 			//magnetic interrupt generator status register
static const uint8_t  LSM9DS1_MA_RG_INT_THS_L_M				__attribute__((unused))	= 0x32; 			//magnetic interrupt generator
static const uint8_t  LSM9DS1_MA_RG_INT_THS_H_M				__attribute__((unused))	= 0x33; 			//threshold

extern void Error_Handler(void);

void HDC1090_Read( I2C_HandleTypeDef * phi2c, float *pt, float *ph );
void LSM9DS1_Init( I2C_HandleTypeDef * phi2c );
void LSM9DS1_ReadGyr( I2C_HandleTypeDef * phi2c, float *pgx, float *pgy, float *pgz );
void LSM9DS1_ReadAcc( I2C_HandleTypeDef * phi2c, float *pax, float *pay, float *paz );
void LSM9DS1_ReadMag( I2C_HandleTypeDef * phi2c, float *pmx, float *pmy, float *mz	);
void LSMD_Gyr2Ori( float gx, float gy, float gz, float *ppitch, float *proll );


#endif /* INC_SENSOR_H_ */
