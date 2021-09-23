#include "sensor.h"
#include "math.h"

#include "hdc1080.h"

int16_t getSignedVal(uint8_t val_l, uint8_t val_h){
	uint16_t value = (((val_h << 8) & 0xff00) | (val_l & 0xff) );
	return *(int16_t*) &value;
}

void hdc1080_init(I2C_HandleTypeDef* hi2c_x,Temp_Reso Temperature_Resolution_x_bit,Humi_Reso Humidity_Resolution_x_bit)
{
	/* Temperature and Humidity are acquired in sequence, Temperature first
	 * Default:   Temperature resolution = 14 bit,
	 *            Humidity resolution = 14 bit
	 */

	/* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
	uint16_t config_reg_value=0x1000;
	uint8_t data_send[2];

	if(Temperature_Resolution_x_bit == Temperature_Resolution_11_bit)
	{
		config_reg_value |= (1<<10); //11 bit
	}

	switch(Humidity_Resolution_x_bit)
	{
	case Humidity_Resolution_11_bit:
		config_reg_value|= (1<<8);
		break;
	case Humidity_Resolution_8_bit:
		config_reg_value|= (1<<9);
		break;
	}

	data_send[0]= (config_reg_value>>8);
	data_send[1]= (config_reg_value&0x00ff);

	HAL_I2C_Mem_Write(hi2c_x,HDC_1080_ADD<<1,Configuration_register_add,I2C_MEMADD_SIZE_8BIT,data_send,2,1000);
}


uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x,float* temperature, float* humidity)
{
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;
	uint8_t send_data = Temperature_register_add;

	HAL_I2C_Master_Transmit(hi2c_x,HDC_1080_ADD<<1,&send_data,1,1000);

	/* Delay here 15ms for conversion compelete.
	 * Note: datasheet say maximum is 7ms, but when delay=7ms, the read value is not correct
	 */
	HAL_Delay(15);

	/* Read temperature and humidity */
	HAL_I2C_Master_Receive(hi2c_x,HDC_1080_ADD<<1,receive_data,4,1000);

	temp_x =((receive_data[0]<<8)|receive_data[1]);
	humi_x =((receive_data[2]<<8)|receive_data[3]);

	*temperature=(((temp_x/65535.0)*165.0)-40.0)-6;
	*humidity=(float)((humi_x/65535.0)*100.0);

	return 0;

}

void LSM9DS1_Init( I2C_HandleTypeDef * phi2c ) {

		uint8_t		i2c_buf[2];

	  //reset
	  i2c_buf[0] = LSM9DS1_AG_RG_CTRL_REG8;
	  i2c_buf[1] = 0x05;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

	  i2c_buf[0] = LSM9DS1_MA_RG_CTRL_REG2_M;
	  i2c_buf[1] = 0x0C;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();
	  HAL_Delay(10);

	  //detection
	  i2c_buf[0] = LSM9DS1_AG_RG_WHO_AM_I;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,1,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();
	  if( ( HAL_I2C_Master_Receive(phi2c,LSM9DS1_AG_AD,i2c_buf,1,HAL_MAX_DELAY ) != HAL_OK ) || ( i2c_buf[0] != 0x68 ) )
	  	Error_Handler();

	  i2c_buf[0] = LSM9DS1_MA_RG_WHO_AM_I_M;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,1,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();
	  if( ( HAL_I2C_Master_Receive(phi2c,LSM9DS1_MA_AD,i2c_buf,1,HAL_MAX_DELAY ) != HAL_OK ) || ( i2c_buf[0] != 0x3D ) )
	  	Error_Handler();

	  // 119 Hz, 2000 dps, 16 Hz BW
	  i2c_buf[0] = LSM9DS1_AG_RG_CTRL_REG1_G;
	  i2c_buf[1] = 0x78;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

	  // 119 Hz, 4g
	  i2c_buf[0] = LSM9DS1_AG_RG_CTRL_REG6_XL;
	  i2c_buf[1] = 0x70;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

	  //Temperature compensation enable, medium performance, 20 Hz
	  i2c_buf[0] = LSM9DS1_MA_RG_CTRL_REG1_M;
	  i2c_buf[1] = 0xB4;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

	  //4 gauss
	  i2c_buf[0] = LSM9DS1_MA_RG_CTRL_REG2_M;
	  i2c_buf[1] = 0x00;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

	  // Continuous conversion mode
	  i2c_buf[0] = LSM9DS1_MA_RG_CTRL_REG3_M;
	  i2c_buf[1] = 0x00;
	  if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,2,HAL_MAX_DELAY) != HAL_OK )
	  	Error_Handler();

}



void LSM9DS1_ReadGyr( I2C_HandleTypeDef * phi2c, float *pgx, float *pgy, float *pgz ){

		uint8_t		i2c_buf[6];

		i2c_buf[0] = LSM9DS1_AG_RG_OUT_X_L_G;
		if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,1,HAL_MAX_DELAY) != HAL_OK )
			Error_Handler();

		if( HAL_I2C_Master_Receive(phi2c,LSM9DS1_AG_AD,i2c_buf,6,HAL_MAX_DELAY ) != HAL_OK )
			Error_Handler();

	*pgx = getSignedVal(i2c_buf[0], i2c_buf[1]) *  0.07000 * 0.017453293;
    *pgy = getSignedVal(i2c_buf[2], i2c_buf[3]) *  0.07000 * 0.017453293;
    *pgz = getSignedVal(i2c_buf[1], i2c_buf[5]) *  0.07000 * 0.017453293;

   // *pgz = ( (i2c_buf[5] << 8) | i2c_buf[4] ) * 2000./ 32768.;

}

void LSMD_Gyr2Ori( float gx, float gy, float gz, float *ppitch, float *proll ){

  *ppitch  = atan2(gx, sqrt(gy * gy) + (gz * gz));
  *proll   = atan2(gy, sqrt(gx * gx) + (gz * gz));
  *ppitch *= 180.0 / M_PI;
  *proll  *= 180.0 / M_PI;

}

//[g]
void LSM9DS1_ReadAcc( I2C_HandleTypeDef * phi2c, float *pax, float *pay, float *paz ){

	uint8_t		i2c_buf[6];

	i2c_buf[0] = LSM9DS1_AG_RG_OUT_X_L_XL;
	if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_AG_AD,i2c_buf,1,HAL_MAX_DELAY) != HAL_OK )
		Error_Handler();

	if( HAL_I2C_Master_Receive(phi2c,LSM9DS1_AG_AD,i2c_buf,6,HAL_MAX_DELAY ) != HAL_OK )
		Error_Handler();

	*pax = (float)getSignedVal(i2c_buf[0], i2c_buf[1]);
	*pax = *pax * 4.0f / 32767.0f * 9.80665f;
	*pay = (float)getSignedVal(i2c_buf[2], i2c_buf[3]);
	*pay = *pay * 4.0f / 32767.0f * 9.80665f;
	*paz = (float)getSignedVal(i2c_buf[4], i2c_buf[5]);
	*paz = *paz * 4.0f / 32767.0f * 9.80665f;
}

//[uT]
void LSM9DS1_ReadMag( I2C_HandleTypeDef * phi2c, float *pmx, float *pmy, float *pmz	){

	uint8_t i2c_buf[6];

	i2c_buf[0] = LSM9DS1_MA_RG_OUT_X_L_M;
	if( HAL_I2C_Master_Transmit(phi2c,LSM9DS1_MA_AD,i2c_buf,1,HAL_MAX_DELAY) != HAL_OK )
		Error_Handler();

	if( HAL_I2C_Master_Receive(phi2c,LSM9DS1_MA_AD,i2c_buf,6,HAL_MAX_DELAY ) != HAL_OK )
		Error_Handler();

//	*pmx = ( (i2c_buf[1] << 8) | i2c_buf[0] ) * 4.0 * 100.0 / 32768.0;
//	*pmy = ( (i2c_buf[3] << 8) | i2c_buf[2] ) * 4.0 * 100.0 / 32768.0;
//	*pmz = ( (i2c_buf[5] << 8) | i2c_buf[4] ) * 4.0 * 100.0 / 32768.0;


	*pmx = getSignedVal(i2c_buf[0], i2c_buf[1]) * 100.0 / 6842.0;
	*pmy = getSignedVal(i2c_buf[2], i2c_buf[3]) * 100.0 / 6842.0;
	*pmz = getSignedVal(i2c_buf[4], i2c_buf[5]) * 100.0 / 6842.0;


}
