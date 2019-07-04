#include "hdc1080.h"
#include "util_console.h"
#include "stm32l0xx_hal_i2c.h"

#if (0)
extern uint8_t set_i2c_hdc1080_for_tx_rx_timing(void);
#endif

uint8_t hdc1080_init(I2C_HandleTypeDef* hi2c_x, Temp_Reso Temperature_Resolution_x_bit, Humi_Reso Humidity_Resolution_x_bit)
{
	/* Temperature and Humidity are acquired in sequence, Temperature first
	 * Default:   Temperature resolution = 14 bit,
	 *            Humidity resolution = 14 bit
	 */

	/* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
	uint16_t config_reg_value=0x1000;
	uint8_t data_send[2];
	HAL_StatusTypeDef status = HAL_OK;

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

	status = HAL_I2C_Mem_Write(hi2c_x, HDC_1080_ADD<<1, Configuration_register_add, I2C_MEMADD_SIZE_8BIT, data_send, 2, 1000); //1000
	
	//HAL_Delay(10);
	HAL_Delay(15);
	
	if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_EXPBD_Error( Addr );
		PRINTF("hdc1080_init Failed...\n\r");
    return 1;
  }
  else
  {
		PRINTF("hdc1080_init Success...\n\r");
    return 0;
  }

#if (0)	
	set_i2c_hdc1080_for_tx_rx_timing();
#endif
}


uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x, float* temperature, uint8_t* humidity)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;
	uint8_t send_data = Temperature_register_add;

	//HAL_Delay(15);
	status = HAL_I2C_IsDeviceReady(hi2c_x, HDC_1080_ADD<<1, 10, 1000);
	if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_EXPBD_Error( Addr );
		PRINTF("HAL_I2C_IsDeviceReady Failed...\n\r");
    //return 1;
  }
  else
  {
		PRINTF("HAL_I2C_IsDeviceReady Success...\n\r");
    //return 0;
  }
	
	status = HAL_I2C_Master_Transmit(hi2c_x, HDC_1080_ADD<<1, &send_data, 1, 1000); //1000
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_EXPBD_Error( Addr );
		PRINTF("HAL_I2C_Master_Transmit Failed...\n\r");
    //return 1;
  }
  else
  {
		PRINTF("HAL_I2C_Master_Transmit Success...\n\r");
    //return 0;
  }
	
	/* Delay here 15ms for conversion compelete.
	 * Note: datasheet say maximum is 7ms, but when delay=7ms, the read value is not correct
	 */
	HAL_Delay(15);
	//HAL_Delay(7);

	/* Read temperature and humidity */
	status = HAL_I2C_Master_Receive(hi2c_x, HDC_1080_ADD<<1, receive_data, 4, 1000); //1000
  
	if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_EXPBD_Error( Addr );
		PRINTF("HAL_I2C_Master_Receive Failed...\n\r");
    //return 1;
  }
  else
  {
		PRINTF("HAL_I2C_Master_Receive Success...\n\r");
    //return 0;
  }
	
	temp_x =((receive_data[0]<<8)|receive_data[1]);
	humi_x =((receive_data[2]<<8)|receive_data[3]);

	*temperature=((temp_x/65536.0)*165.0)-40.0;
	*humidity=(uint8_t)((humi_x/65536.0)*100.0);

	return 0;

}
