 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   this is the main!
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

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "timeServer.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "lora.h"
#include "bsp.h"
#if defined(__USE_HDC1080)
#include "x_nucleo_iks01a2.h"
#include "./Drivers/hdc1080/hdc1080.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
//
// Jaehong Park : cto@onofflab.xyz : feature only for main.c
//
////////////////////////////////////////////////////////////////////////////////
#define __USE_THINGPLUS__ //hong
//#define __USE_PH_TUBIDITY_SENSOR__
#define __WITH_LORA_JOIN__
//#define __USE_BOARD_INIT_LED_TIMER__
#define __USE_LED_INDICATOR__

////////////////////////////////////////////////////////////////////////////////

// reference to Ping-pong app.
typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

States_t State = LOWPOWER;

/**
 * @brief LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
#define APP_TX_DUTYCYCLE                            (1000*10) //unit: ms

/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64

/**
 * When fast wake up is enabled, the mcu wakes up in ~20us and
 * does not wait for the VREFINT to be settled. THis is ok for
 * most of the case except when adc must be used in this case before
 * starting the adc, you must make sure VREFINT is settled
 */
#define ENABLE_FAST_WAKEUP

#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_RED1 ) ;   \
                   LED_Off( LED_RED2 ) ;    \
                   LED_Off( LED_GREEN ) ; \
                   LED_Off( LED_BLUE ) ; \
                   } while(0) ;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* Private variables ---------------------------------------------------------*/
#if defined(__USE_HDC1080)
static I2C_HandleTypeDef hi2c1;
#endif
									 
/* load call backs*/
/*!
 * User application data
 */
#if (1)
static uint8_t AppDataBuff[256];
#else
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
#endif

/*!
 * User application data structure
 */
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

static TimerEvent_t TxTimer;

 /* Led Timers objects*/
static  TimerEvent_t timerLed;


static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LoraRxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/**
 * Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK};

                                    
                                    
/* Private functions ---------------------------------------------------------*/

																		/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/*! hong: add for LED operation
 * \brief Function executed on when led timer elapses
 */
static void OnledEvent( void );

static void ledOnTX (void );
static void ledOnRX (void );
static void ledInitOnTRX(void);
static void ledOnJoining (void );
static void ledAfterJoin (void );

#if defined(__USE_HDC1080)
static void I2C1_GPIO_Init( void );
static uint8_t I2C1_HDC1080_Init(void);
#endif

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
    /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();

  /* Configure the hardware*/
  HW_Init();

#if defined(__USE_HDC1080)
	I2C1_HDC1080_Init();
	
	hdc1080_init(&hi2c1, Temperature_Resolution_14_bit, Humidity_Resolution_14_bit);
#endif
	
  /* Configure Debug mode */
  DBG_Init();
  
  /* USER CODE BEGIN 1 */
  CMD_Init();
  /*Disable standby mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

#if defined(__USE_BOARD_INIT_LED_TIMER__)
  /* Led Timers*/
  TimerInit(&timerLed, OnledEvent);   
  TimerSetValue( &timerLed, LED_PERIOD_MS);

  TimerStart(&timerLed );
#endif
  
	PRINTF("VERSION: %X\n\r", VERSION);
	
  PRINTF("ATtention command interface\n\r");
  /* USER CODE END 1 */

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

#if defined(__WITH_LORA_JOIN__) //hong: port from end node src
	ledOnJoining();
	LORA_Join();
#endif

		LoraStartTx( TX_ON_TIMER) ;

  /* main loop*/
  while (1)
  {
    /* Handle UART commands */
    CMD_Process();
    
    LoRaMacProcess( );
    /*
     * low power section
     */
    DISABLE_IRQ();
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();

    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}


#if defined( __USE_THINGPLUS__) //Daliworks Thingplus
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
#define LPP_APP_PORT 99

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

#define SETALAB_CTRL_PORT  	111  //hong
#define THINGPLUS_CTRL_PORT				   222
#define ONOFFLAB_CTRL_PORT 	(11)

#define PWOFF 0

static void LoraRxData(lora_AppData_t *AppData)
{
	ledOnRX(); // indicate GREEN LED on TX

	if(AppData == 0)
	{
		PRINTF("LoraRxData but AppData is NULL\r\n");
		return;
	}
	
	PRINTF("AppData Port=%d AppData->BuffSize=%d\r\n", AppData->Port, AppData->BuffSize);

	switch (AppData->Port)
	{
	case ONOFFLAB_CTRL_PORT				   : //11
		{
			//AppData->Buff[0]: ?? ??. 0
			//AppData->Buff[1] == 0x00: General Device Management
			//AppData->Buff[1] == 0x80: Device Reset
			//AppData->Buff[1] == 0x81: Change the periodic time of reporting
			//AppData->Buff[1] == 0x82: reporting
			//AppData->Buff[2]: Payload Length (N)
			//AppData->Buff[3~N+3]: Payload
			if(AppData->BuffSize > 1)
			{
				switch(AppData->Buff[1])
				{
					case 0x00: //extDevMgmt
						PRINTF("0x00 extDevMgmt\n");
					break;
					case 0x80: //Reboot
						PRINTF("Reset Device!!\n");
						TimerStop( &TxTimer ); 
						HAL_Delay(1000);
						HAL_NVIC_SystemReset();
					break;
					case 0x81: // Uplink Period
					{
						uint32_t perioddata = 0;
						uint32_t periods = 0; //???
						TimerStop( &TxTimer ); 
						PRINTF("0x81 Period reset!\n");
						if(AppData->BuffSize > 2)
						{
				#if (1) //Loriot Network order
						for(int i=0; (i< AppData->Buff[2])&&i<4; i++)
						periods |=  (uint32_t)((AppData->Buff[3+i])<<(8*(3-i)));
				#else
						for(int i=0; (i< AppData->Buff[2])&&i<4; i++)
						periods |=  (uint32_t)((AppData->Buff[3+i])<<(8*i));
				#endif
						}

						if(periods == 0)
						{
							PRINTF("Uplink Period Off!\n");
						}
						else 
						{
							if(periods < 10)
							{
								PRINTF("Period ms[%u] is limited by 10sec\r\n", periods);
								periods = 10000;
							}
							// Maximum 30? ?? 
							else if(periods >= 2592000)
								periods = 2592000000;
							else
								periods *= 1000;

							PRINTF("Period ms[%u]\r\n", periods);
							TimerSetValue( &TxTimer,  periods);
							TimerStart( &TxTimer);
						}
						//perioddata is value from application server.
						perioddata = periods/1000;
					}
					break;
					case 0x82: //extDevMgmt
						PRINTF("0x82 report\n");
					break;
				}
			}
		}
	   break;

#if (0)
	case THINGPLUS_CTRL_PORT: //222
		{
			PRINTF("THINGPLUS_CTRL_PORT\n"); 
			for(int j=0; j<AppData->BuffSize; j++) 
				PRINTF("%02X ", AppData->Buff[j]); 
			PRINTF("\n"); 
			
			if(AppData->BuffSize > 4)
			{
				uint32_t utc = 0;
				for(int i=0; i<4; i++)
						utc |=  (uint32_t)((AppData->Buff[1+i])<<(8*i));
				PRINTF("status=%d utc=%d \n", AppData->Buff[0], utc);
				if(AppData->BuffSize > 7)
				{
					PRINTF("Type = %1X SeqLen=%1X Val=%d\n", AppData->Buff[5], AppData->Buff[6], AppData->Buff[7]);
					switch(AppData->Buff[5])
					{
						// Power Off
						case 0x3D:
							if(AppData->Buff[7]==PWOFF)
							{
								PRINTF("Reset Device!!\n");
								TimerStop( &TxTimer ); 
								HAL_Delay(1000);
								HAL_NVIC_SystemReset();
							}
							break;
						default:
							break;
					}
				}
			}
		}
		break;
#endif

	case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
		
	case LORAWAN_APP_PORT: // 2 ( port number used for both Tx & Rx)
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    }
    break;
		
  case LPP_APP_PORT: // 99
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      LED_On( LED_BLUE ) ; 
    }
    break;
  }
	default:
    break;
	}
	set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
}
#else //__USE_THINGPLUS__

static void LoraRxData(lora_AppData_t *AppData)
{
   set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
}
#endif 

#if defined( __USE_THINGPLUS__) //Daliworks Thingplus
#define PR_PORT 10		
static uint32_t pr_cnt = 0;
#if defined(__USE_HDC1080)
volatile float temp;
volatile uint8_t humi;
#endif

static void Send( void )
{
  /* USER CODE BEGIN 3 */
#if defined(__USE_PRESSURE_SENSOR__)
  uint16_t pressure = 0;
#endif	
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t batteryLevel;
#if !defined(__USE_HDC1080)	
  sensor_t sensor_data;
#endif
	
#if defined(__USE_PH_TUBIDITY_SENSOR__)
	uint16_t turbidity = 0;
	uint16_t pHmeter = 0;
#endif

  uint32_t i = 0;

#if (0)
#if defined(__WITH_LORA_JOIN__)
  if ( LORA_JoinStatus () != LORA_SET)
  {
		PRINTF("LORA_JoinStatus () != LORA_SET, Try to join again later! \n\r");
    /*Not joined, try again later*/
    ledOnJoining();
	//DelayMs( 1 ); //hong
	LORA_Join();
    return;
  }
#endif
#endif

#if defined(__USE_HDC1080)
	hdc1080_start_measurement(&hi2c1, (float*)&temp, (uint8_t*)&humi);
#else
  BSP_sensor_Read( &sensor_data );
#endif

#if defined(__USE_HDC1080)
	temperature = ( int16_t )( temp*100);
  humidity    = ( uint16_t )( humi );
#else	
  temperature = ( int16_t )( sensor_data.temperature*100);
#if defined(__USE_PRESSURE_SENSOR__)	
  pressure    = ( uint16_t )( sensor_data.pressure );
#endif	
  humidity    = ( uint16_t )( sensor_data.humidity ); 
#endif
	
  /* 1 (very low) to 254 (fully charged) */
  batteryLevel = HW_GetBatteryLevel( ) * 100 / 255; 
	
#if defined(__USE_PRESSURE_SENSOR__)
  PRINTF("pr_cnt=%d temperature = %d Pressure=%d Humidity=%d\n\r", pr_cnt, temperature, pressure, humidity);
#else
  PRINTF("pr_cnt=%d temperature = %d Humidity=%d batteryLevel=%d\n\r", pr_cnt, temperature, humidity, batteryLevel);
#endif

#if (1)
#if defined(__WITH_LORA_JOIN__)
  if ( LORA_JoinStatus () != LORA_SET)
  {
		PRINTF("LORA_JoinStatus () != LORA_SET, Try to join again later! \n\r");
    /*Not joined, try again later*/
    ledOnJoining();
	//DelayMs( 1 ); //hong
	LORA_Join();
    return;
  }
#endif
#endif
	
#if defined(__USE_PH_TUBIDITY_SENSOR__)
	turbidity = HW_GetTubidityLevel( );
	pHmeter = HW_GetPHMeterLevel( );
#endif	

  AppData.Port = PR_PORT;

#ifdef __USE_THINGPLUS__
	
#if defined(__USE_PH_TUBIDITY_SENSOR__)
		AppData.Buff[i++]  = 0; // status
  
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	
  AppData.Buff[i++]  = 27; // number type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
  AppData.Buff[i++] = ( turbidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = turbidity & 0xFF;
	
  AppData.Buff[i++]  = 31; // batterGauge type
	AppData.Buff[i++]  = 0x11; //  the same type num & data size
  AppData.Buff[i++] = batteryLevel & 0xFF;
	
	AppData.Buff[i++]  = 52; // pH type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
	AppData.Buff[i++] = ( pHmeter >> 8 ) & 0xFF;
	AppData.Buff[i++] = pHmeter & 0xFF;
#else
	AppData.Buff[i++]  = 0; // status
  
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 1; // temperature type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
	AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData.Buff[i++] = temperature & 0xFF;

  AppData.Buff[i++]  = 2; // humidity type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;

	AppData.Buff[i++]  = 31; // batterGauge type
	AppData.Buff[i++]  = 0x11; //  the same type num & data size
  AppData.Buff[i++] = batteryLevel & 0xFF;
	
  #if defined(__USE_PRESSURE_SENSOR__)
  	AppData.Buff[i++]  = 28; // pressure type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
  	AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  	AppData.Buff[i++] = pressure & 0xFF;
	#endif
	
#endif
#else //temp
	AppData.Buff[i++]  = ( pr_cnt >> 8 ) & 0xFF;
  	AppData.Buff[i++]  = pr_cnt & 0xFF;
  	AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  	AppData.Buff[i++] = pressure & 0xFF;
  	AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  	AppData.Buff[i++] = temperature & 0xFF;
  	AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  	AppData.Buff[i++] = humidity & 0xFF;
  	AppData.Buff[i++] = batteryLevel;
  	AppData.Buff[i++] = 0;
	pr_cnt++;
#endif

  AppData.BuffSize = i;

#if defined(__WITH_LORA_JOIN__)
  ledOnTX();
  LORA_send( &AppData, lora_config_reqack_get());

#endif

  /* USER CODE END 3 */
}
#else //#if defined( __USE_THINGPLUS__)


#define PR_PORT 10																		
static uint32_t pr_cnt = 0;
static void Send( void )
{
  /* USER CODE BEGIN 3 */
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t batteryLevel;
  sensor_t sensor_data;

#ifndef CAYENNE_LPP // reference in End-node app.
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif

  uint32_t i = 0;
	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
	
  BSP_sensor_Read( &sensor_data );
	
  temperature = ( int16_t )( sensor_data.temperature*100); 
  pressure    = ( uint16_t )( sensor_data.pressure ); 
  humidity    = ( uint16_t )( sensor_data.humidity ); 
  
	latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
	
  PRINTF("pr_cnt=%d temperature = %d Pressure=%d Humidity=%d\n", pr_cnt, temperature, pressure, humidity);
	
  /* 1 (very low) to 254 (fully charged) */
  batteryLevel = HW_GetBatteryLevel( );                     

  AppData.Port = PR_PORT;  // 10
	
  AppData.Buff[i++]  = ( pr_cnt >> 8 ) & 0xFF;
  AppData.Buff[i++]  = pr_cnt & 0xFF;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = 0;
	pr_cnt++;


/* daliworks thingplus format
  AppData.Buff[i++]  = 0; // status
  
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 1; // temperature type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
	AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData.Buff[i++] = temperature & 0xFF;
	pr_cnt++;
*/


  AppData.BuffSize = i;

  ledOnTX(); // indicate BLUE LED on TX

  LORA_send( &AppData, lora_config_reqack_get());
  
  /* USER CODE END 3 */
}
#endif //__USE_THINGPLUS__

static void OnTxTimerEvent( void )
{
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
  /*Send*/
  Send( );
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  Error_Handler();
}
#endif

static void LORA_HasJoined( void )
{
  PRINTF("JOINED\n\r");

#if (0) // reference to End-node app.
	LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
#endif
	
	ledAfterJoin();
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

#if (0)	
	/*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT; //2
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
#endif	
}

static void LORA_TxNeeded ( void )
{
  PRINTF("Network Server is asking for an uplink transmission\n\r");

#if (0)
	AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT; // 2
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
#endif
}

static void OnledEvent( void )
{ 
  //LED_Toggle( LED_RED1 ) ; //LED2 : not used 
  LED_Toggle( LED_RED2 ) ; //LED1
  LED_Toggle( LED_BLUE ) ;  //LED3
  LED_Toggle( LED_GREEN ) ;   //LED4

  TimerStart(&timerLed );
}

static void ledOnTX (void )
{
	TimerStop(&timerLed );
	// Indicates on a RED1, RED2 after joinning network
#if (1)
	//LED_On(LED_BLUE); 
#else
  //LED_On( LED_RED1);
  LED_Off( LED_RED2 ) ; 
	// Indicates on a BLUE LED on TX
  LED_Toggle( LED_BLUE );
  LED_On( LED_GREEN ) ;
#endif	
}

static void ledOnRX(void)
{
	TimerStop(&timerLed );
	// Indicates on a RED1, RED2 after joinning network
#if (1)
	//LED_On( LED_RED2 ) ;
#else
  //LED_On( LED_RED1 ) ;
  // Indicates on a RED2 LED on RX	
  LED_Toggle( LED_RED2 ) ;
	LED_Off( LED_BLUE);
  LED_On( LED_GREEN ) ;
#endif
	
}

static void ledInitOnTRX(void)
{
	LED_Off( LED_BLUE ) ;
	LED_Off( LED_RED2 ) ;
}

static void ledOnJoining (void )
{
	TimerStop(&timerLed );
	// Indicates on a RED1, RED2 when trying to join network
#if (1)
	LED_Toggle( LED_GREEN ) ;
#else
  //LED_Toggle( LED_RED1);
  LED_Off( LED_RED2 ) ; 
	LED_Off( LED_BLUE );
  LED_Toggle( LED_GREEN ) ;
#endif
}

static void ledAfterJoin (void )
{
	TimerStop(&timerLed );
	// Indicates on a RED1, RED2 after joinning network
#if (1)
	LED_On( LED_GREEN ) ;
#else	
  //LED_On( LED_RED1);
  LED_Off( LED_RED2 ) ; 
	LED_Off( LED_BLUE );
	
  LED_On( LED_GREEN ) ;
#endif	
}


#if defined(PH_SENSOR) //hong
// Not used.
#endif

#if defined(TUBIDITY_SENSOR) //hong
// Not used.
#endif

#if defined(__USE_HDC1080)
/* Timing samples for L0 with SYSCLK 32MHz set in SystemClock_Config() */
#if (defined (USE_STM32L0XX_NUCLEO)|| (defined (USE_B_L072Z_LRWAN1)))
#define HDC1080_I2C_EXPBD_TIMING_100KHZ       0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define HDC1080_I2C_EXPBD_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */
#endif /* USE_STM32L0XX_NUCLEO */

/* I2C peripheral configuration defines */
#define NUCLEO_I2C_HDC1080                            I2C1
#define NUCLEO_I2C_HDC1080_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define NUCLEO_I2C_HDC1080_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define NUCLEO_I2C_HDC1080_SCL_SDA_AF                 GPIO_AF4_I2C1
#define NUCLEO_I2C_HDC1080_SCL_SDA_GPIO_PORT          GPIOB
#define NUCLEO_I2C_HDC1080_SCL_PIN                    GPIO_PIN_8
#define NUCLEO_I2C_HDC1080_SDA_PIN                    GPIO_PIN_9

#define NUCLEO_I2C_HDC1080_FORCE_RESET()              __I2C1_FORCE_RESET()
#define NUCLEO_I2C_HDC1080_RELEASE_RESET()            __I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#if (defined (USE_STM32L0XX_NUCLEO)|| (defined (USE_B_L072Z_LRWAN1)))
#define NUCLEO_I2C_HDC1080_EV_IRQn                    I2C1_IRQn
#endif


/**
 * @brief I2C MSP Initialization
 * @param None
 * @retval None
 */
static void I2C1_GPIO_Init( void )
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  NUCLEO_I2C_HDC1080_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = NUCLEO_I2C_HDC1080_SCL_PIN | NUCLEO_I2C_HDC1080_SDA_PIN;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO))|| (defined (USE_B_L072Z_LRWAN1)))
  #if (1) // tune-debug for hdc1080
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	#else
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	#endif
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = NUCLEO_I2C_HDC1080_SCL_SDA_AF;

  HAL_GPIO_Init( NUCLEO_I2C_HDC1080_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
  NUCLEO_I2C_HDC1080_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  NUCLEO_I2C_HDC1080_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  NUCLEO_I2C_HDC1080_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(NUCLEO_I2C_HDC1080_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NUCLEO_I2C_HDC1080_EV_IRQn);
}

/**
 * @brief  Configures I2C1 interface.
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C1_HDC1080_Init(void)
{
 if(HAL_I2C_GetState( &hi2c1) == HAL_I2C_STATE_RESET )
 {
 
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif

#if (defined (USE_STM32L0XX_NUCLEO)|| (defined (USE_B_L072Z_LRWAN1)))	
	#if (0) // tune-debug for hdc1080
	hi2c1.Init.Timing = HDC1080_I2C_EXPBD_TIMING_100KHZ;// 
	#else
	hi2c1.Init.Timing = HDC1080_I2C_EXPBD_TIMING_400KHZ;// 400Khz
	#endif
#endif
	 
#if (defined (USE_STM32L4XX_NUCLEO))
	hi2c1.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ; //400Khz
#endif

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))	
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;	
#endif

	hi2c1.Init.OwnAddress1    = 0x00; //0x33;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  //hi2c1.Instance            = NUCLEO_I2C_EXPBD;
	hi2c1.Instance = I2C1; //I2C3(scl:PC0 sda:PC1) //I2C1 (scl;PB8, sda:PB9) //I2C2(scl:PB10, sda:PB11)

#if (0) // reference codes
	//hi2c1.Init.OwnAddress1 =I 0;
	//hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
#endif

#if (1) // tune-debug for hdc1080
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
#endif

	/* Init the I2C */
  I2C1_GPIO_Init();
	
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
 } //if
	
	if( HAL_I2C_GetState( &hi2c1) == HAL_I2C_STATE_READY )
  {
		PRINTF("I2C1_Init Success...\n\r");
    return 0;
  }
  else
  {
		PRINTF("I2C1_Init Failed...\n\r");
    return 1;
  }

#if (0) // tune-debug for hdc1080: work-around for failure of i2c tx/rx
	/* Enable the I2C_EXPBD peripheral clock */
  NUCLEO_I2C_HDC1080_CLK_ENABLE();
#endif
	
}

#if (0)
uint8_t set_i2c_hdc1080_for_tx_rx_timing(void)
{
// if(HAL_I2C_GetState( &hi2c1) == HAL_I2C_STATE_RESET )
// {
	#if (1) // tune-debug for hdc1080
	hi2c1.Init.Timing = HDC1080_I2C_EXPBD_TIMING_100KHZ;// 
	#endif
	
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
// }
 
	if( HAL_I2C_GetState( &hi2c1) == HAL_I2C_STATE_READY )
  {
		PRINTF("I2C1_hdc1080_txrx_timing_Init Success...\n\r");
    return 0;
  }
  else
  {
		PRINTF("I2C1_hdc1080_txrx_timing_Init Failed...\n\r");
    return 1;
  }
}
#endif//#if (0)
#endif//#if defined(__USE_HDC1080)

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
