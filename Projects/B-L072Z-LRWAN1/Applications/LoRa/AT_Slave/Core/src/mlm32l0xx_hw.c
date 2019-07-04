/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /*******************************************************************************
  * @file    mlm32l0xx_hw.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   system hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "hw.h"
#include "radio.h"
#include "debug.h"
#include "vcom.h"
#include "bsp.h"

/*!
 *  \brief Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * \brief ADC Vbat measurement constants
 */

 /* Internal voltage reference, parameter VREFINT_CAL*/
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#define LORAWAN_MAX_BAT   254


/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */

/* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at 
 *a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t) 0x1FF8007A))

/* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at 
 *a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FF8007E))

/* Vdda value with which temperature sensor has been calibrated in production 
   (+-10 mV). */
#define VDDA_TEMP_CAL                  ((uint32_t) 3000)        


#define COMPUTE_TEMPERATURE(TS_ADC_DATA, VDDA_APPLI)                           \
  ((((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / VDDA_TEMP_CAL)                  \
        - (int32_t) *TEMP30_CAL_ADDR)                                          \
     ) * (int32_t)(110 - 30)                                                   \
    )<<8) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + (30<<8)                                                                      \
  )

static ADC_HandleTypeDef hadc;
/*!
 * Flag to indicate if the ADC is Initialized
 */
static bool AdcInitialized = false;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/**
  * @brief This function initializes the hardware
  * @param None
  * @retval None
  */
void HW_Init( void )
{
  if( McuInitialized == false )
  {
#if defined( USE_BOOTLOADER )
    /* Set the Vector Table base location at 0x3000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 );
#endif

    HW_AdcInit( );

    Radio.IoInit( );
    
    HW_SPI_Init( );

    HW_RTC_Init( );
    
    TraceInit( );   

#if (1) //defined(__USE_LED_INDICATOR__)		
    BSP_LED_Init( LED1 );
    //BSP_LED_Init( LED2 ); //LED_RED1
    BSP_LED_Init( LED3 );
    BSP_LED_Init( LED4 );
#endif

#if (0)
		BSP_sensor_Init();
#endif

		//MX_I2C1_Init();
		
    McuInitialized = true;
  }
}

/**
  * @brief This function Deinitializes the hardware
  * @param None
  * @retval None
  */
void HW_DeInit( void )
{
  HW_SPI_DeInit( );
  
  Radio.IoDeInit( );
  
  vcom_DeInit( );
   
  McuInitialized = false;
}

/**
  * @brief This function Initializes the hardware Ios
  * @param None
  * @retval None
  */
static void HW_IoInit( void )
{
  HW_SPI_IoInit( );
  
  Radio.IoInit( );
  
  vcom_IoInit( );
}

/**
  * @brief This function Deinitializes the hardware Ios
  * @param None
  * @retval None
  */
static void HW_IoDeInit( void )
{
  /*  HW_SPI_IoDeInit( );*/
  GPIO_InitTypeDef initStruct={0};
    
  initStruct.Mode =GPIO_MODE_ANALOG;
  initStruct.Pull =GPIO_NOPULL;
  HW_GPIO_Init ( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct ); 
  HW_GPIO_Init ( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct ); 
  HW_GPIO_Init ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct ); 
  HW_GPIO_Init ( RADIO_NSS_PORT, RADIO_NSS_PIN , &initStruct ); 

  
  Radio.IoDeInit( );
  
// vcom_IoDeInit( );
}


void HW_GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  /* All GPIOs except debug pins (SWCLK and SWD) */
  GPIO_InitStruct.Pin = GPIO_PIN_All & (~( GPIO_PIN_13 | GPIO_PIN_14) );
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* All GPIOs */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */

void SystemClock_Config( void )
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief This function return a random seed
  * @note based on the device unique ID
  * @param None
  * @retval see
  */
uint32_t HW_GetRandomSeed( void )
{
  return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

/**
  * @brief This function return a unique ID
  * @param unique ID
  * @retval none
  */
void HW_GetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint16_t HW_GetTemperatureLevel( void ) 
{
  uint16_t measuredLevel =0; 
  uint32_t batteryLevelmV;
  uint16_t temperatureDegreeC;

  measuredLevel = HW_AdcReadChannel( ADC_CHANNEL_VREFINT ); 

  if (measuredLevel ==0)
  {
    batteryLevelmV =0;
  }
  else
  {
    batteryLevelmV= (( (uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL ) )/ measuredLevel);
  }
#if (1) //hong: adc  
  PRINTF("VDDA= %d\n\r", batteryLevelmV);
#endif
  
  measuredLevel = HW_AdcReadChannel( ADC_CHANNEL_TEMPSENSOR ); //hong: analog porting 
  
  temperatureDegreeC = COMPUTE_TEMPERATURE( measuredLevel, batteryLevelmV);

#if (1) //hong: adc
  {
    uint16_t temperatureDegreeC_Int= (temperatureDegreeC)>>8;
    uint16_t temperatureDegreeC_Frac= ((temperatureDegreeC-(temperatureDegreeC_Int<<8))*100)>>8;  
    PRINTF("temp= %d, %d,%d\n\r", temperatureDegreeC, temperatureDegreeC_Int, temperatureDegreeC_Frac);
  }
#endif
  
  return (uint16_t) temperatureDegreeC;
}
/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t HW_GetBatteryLevel( void ) 
{
  uint8_t batteryLevel = 0;
  uint16_t measuredLevel = 0;
  uint32_t batteryLevelmV;

  measuredLevel = HW_AdcReadChannel( ADC_CHANNEL_VREFINT ); 

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    batteryLevelmV= (( (uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL ) )/ measuredLevel);
  }

  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (( (uint32_t) (batteryLevelmV - VDD_MIN)*LORAWAN_MAX_BAT) /(VDD_BAT-VDD_MIN) ); 
  }

#if (0) //hong: adc  
  PRINTF("batteryLevel= %d\n\r", batteryLevel);
#endif
	
  return batteryLevel;
}

#if defined(__USE_PH_TUBIDITY_SENSOR__) //hong
uint32_t avergearray(uint16_t* arr, uint8_t number){
  uint8_t i;
  uint8_t max,min;
  uint32_t avg;
  uint32_t amount=0;
  if(number<=0){
    PRINTF("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
		PRINTF("avgADC= %x, number= %d\n\r", avg, number);
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
	PRINTF("avgADC= %x, number= %d\n\r", avg, number);
  return avg;
}

#define VOLTAGE_MAX_INPUT			(5000) // -412mV~ +412mV
#define ADC12BIT_RESOLUTION				(4096) //12bit resolution

#define tArrayLenth  (6)    //times of collection
static uint16_t tArray[tArrayLenth];//Store the average value of the sensor feedback
//static uint16_t tArray[tArrayLenth]={0x4d,0x4d,0x4d,0x4d,0x4d,0x4d}; //5 NTS
static uint8_t tArrayIndex=0; 

uint16_t HW_GetTubidityLevel( void ) 
{
  uint16_t tubidityLevel = 0; //0 NTS ~ 3000 NTS
  uint16_t measuredTubidityADC = 0;
  uint32_t tubidityLevelmV = 0; // 4.2V ~ 2.5V

#if (1) //bufferd way
	tArray[tArrayIndex++] = HW_AdcReadChannel( ADC_CHANNEL_TUBIDITY );
	if(tArrayIndex == tArrayLenth) tArrayIndex=0;
  PRINTF("measuredTubidityADC reading start...tArrayIndex= %d\n\r", tArrayIndex);
	tubidityLevelmV = avergearray(tArray, tArrayLenth)*VOLTAGE_MAX_INPUT/ADC12BIT_RESOLUTION;
  //tubidityLevel = (-1714)*(tubidityLevelmV /1000) + 7285;
	tubidityLevel = (-17)* tubidityLevelmV + 4539;
	if (tubidityLevel < 0){
		tubidityLevel = 0; // NTS
	}else if (tubidityLevel > 3000){
		tubidityLevel = 3000; //NTS
	}
#else
  measuredTubidityADC = HW_AdcReadChannel( ADC_CHANNEL_TUBIDITY ); 
	tubidityLevelmV = measuredTubidityADC * VOLTAGE_MAX_INPUT / ADC12BIT_RESOLUTION;
	// wrong calculator equation
	//tubidityLevel = (-1120.4)*(tubidityLevelmV /1000)*(tubidityLevelmV /1000) + (5742.3)*(tubidityLevelmV /1000) - 4352.9; 
  tubidityLevel = (-1714)*(tubidityLevelmV /1000) + 7285;
#endif
	PRINTF("tubidityLevelmV= %d mV, tubidityLevel= %d NTU\n\r",tubidityLevelmV, tubidityLevel);
	
  return tubidityLevel;
}

#endif //#if defined(__USE_PH_TUBIDITY_SENSOR__) //hong


#if defined(__USE_PH_TUBIDITY_SENSOR__) //hong
#ifndef VOLTAGE_MAX_INPUT
#define VOLTAGE_MAX_INPUT			(5000) // -412mV~ +412mV
#endif
#ifndef ADC12BIT_RESOLUTION
#define ADC12BIT_RESOLUTION				(4096) //12bit resolution
#endif
#define pH_OFFSET								(0)

#define pHArrayLenth  (6)    //times of collection : 1 min * 6 array sample
static uint16_t pHArray[pHArrayLenth]; //Store the average value of the sensor feedback
//static uint16_t pHArray[pHArrayLenth]={0xf5,0xf5,0xf5,0xf5,0xf5,0xf5};
static uint8_t pHArrayIndex=0;  

uint16_t HW_GetPHMeterLevel( void ) 
{
  uint16_t pHMeterLevel = 0; // 0 ~ 7 ~ 14
  uint16_t measuredPHADC = 0;
  uint32_t pHMeterLevelmV; //

#if (1) //bufferd way
	pHArray[pHArrayIndex++] = HW_AdcReadChannel( ADC_CHANNEL_PHMETER );
	if(pHArrayIndex == pHArrayLenth) pHArrayIndex=0;
  PRINTF("measuredPHADC reading start...pHArrayIndex= %d\n\r", pHArrayIndex);
	pHMeterLevelmV = avergearray(pHArray, pHArrayLenth)*VOLTAGE_MAX_INPUT/ADC12BIT_RESOLUTION;
  //pHMeterLevel = 3.5*pHMeterLevelmV+pH_OFFSET;	 // 3.5 or 1.45 ???

	#if (0) //test purpose of thingplus server
	pHMeterLevel = (0.05357 * pHMeterLevelmV - 6.12473) * 1000;
	if (pHMeterLevel < 0){
		pHMeterLevel = 0; // pH
	}else if (pHMeterLevel > 14 * 1000){
		pHMeterLevel = 14*1000; //pH
	}
	#else
	pHMeterLevel = (0.1304 * pHMeterLevelmV - 16.472)* 1000;
	if (pHMeterLevel < 0){
		pHMeterLevel = 0; // pH
	}else if (pHMeterLevel > 14 * 1000){
		pHMeterLevel = 14 * 1000; //pH
	}
	#endif
#else
  measuredPHADC = HW_AdcReadChannel( ADC_CHANNEL_PHMETER );
	pHMeterLevelmV = measuredPHADC * VOLTAGE_MAX_INPUT / ADC12BIT_RESOLUTION;
	pHMeterLevel =  (3.5) * pHMeterLevelmV + pH_OFFSET;
	//pHMeterLevel =  (1.45) * pHMeterLevelmV + pH_OFFSET;
#endif
	PRINTF("pHMeterLevelmV= %d mV, pHMeterLevel= %d PH\n\r",pHMeterLevelmV, pHMeterLevel);
	
  return pHMeterLevel; // 0~ 7 ~ 14
}

#endif //#if defined(__USE_PH_TUBIDITY_SENSOR__) 

/**
  * @brief This function initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcInit( void )
{
  if( AdcInitialized == false )
  {
    AdcInitialized = true;

    
    hadc.Instance  = ADC1;
    
    hadc.Init.OversamplingMode      = DISABLE;
  
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.LowPowerFrequencyMode = ENABLE;
    hadc.Init.LowPowerAutoWait      = DISABLE;
    
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.DMAContinuousRequests = DISABLE;

    ADCCLK_ENABLE();
    

    HAL_ADC_Init( &hadc );

  }
}
/**
  * @brief This function De-initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcDeInit( void )
{
  AdcInitialized = false;
}

/**
  * @brief This function De-initializes the ADC
  * @param Channel
  * @retval Value
  */
uint16_t HW_AdcReadChannel( uint32_t Channel )
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;
  
  if( AdcInitialized == true )
  {
    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};
      
    ADCCLK_ENABLE();
    
    /*calibrate ADC if any calibraiton hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED );
    
    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE; 
    HAL_ADC_ConfigChannel( &hadc, &adcConf);
      
    /* configure adc channel */
    adcConf.Channel = Channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel( &hadc, &adcConf);

    /* Start the conversion process */
    HAL_ADC_Start( &hadc);
      
    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion( &hadc, HAL_MAX_DELAY );
      
    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue ( &hadc);

    __HAL_ADC_DISABLE( &hadc) ;

    ADCCLK_DISABLE();
  }
  return adcData;
}

/**
  * @brief Enters Low Power Stop Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void LPM_EnterStopMode( void)
{
  BACKUP_PRIMASK();

  DISABLE_IRQ( );

  HW_IoDeInit( );
  
  /*clear wake up flag*/
  SET_BIT(PWR->CR, PWR_CR_CWUF);
  
  RESTORE_PRIMASK( );

  /* Enter Stop Mode */
  HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}
/**
  * @brief Exists Low Power Stop Mode
  * @note Enable the pll at 32MHz
  * @param none
  * @retval none
  */
void LPM_ExitStopMode( void)
{
  /* Disable IRQ while the MCU is not running on HSI */

  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );

  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSI */
  __HAL_RCC_HSI_ENABLE();

  /* Wait till HSI is ready */
  while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET ) {}
  
  /* Enable PLL */
  __HAL_RCC_PLL_ENABLE();
  /* Wait till PLL is ready */
  while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET ) {}
  
  /* Select PLL as system clock source */
  __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );
  
  /* Wait till PLL is used as system clock source */ 
  while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK ) {}
    
  /*initilizes the peripherals*/
  HW_IoInit( );

  RESTORE_PRIMASK( );
}

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void LPM_EnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

