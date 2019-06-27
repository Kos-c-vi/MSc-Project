/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Ping-Pong implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/*
********************************************************************************
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
********************************************************************************
******************************************************************************
  * @file    main.c
  * @author  Kossivi Fangbemi
  * @version V1.0.0
  * @date    SEPTEMBER-2017
  * @brief   THIS IS THE TRANSMISSION NODE (NODE 0)'S MAIN CODE
  * 		 Adopted from the Ping-Pong implementation example from Semtech
********************************************************************************
********************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power.h"
#include "vcom.h"
//#include "SD.h"

#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             20        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            10000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               				200
#define RADIO_TX_PERIOD_MS               			5000

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;

uint8_t StartKeyWord[] = "Start ";
uint8_t * qt1 = StartKeyWord;
uint8_t * Data[] = {"0.0001 ", "0.001 ", "0.01 ", "0.1 ", "0 ", "1.0 ", "10.0 ", "100.0 ", "1000.0 ", "10000.0 "};
uint8_t EndKeyWord[] =  "End";
uint8_t * qt3 = EndKeyWord;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t TxBuffer[BUFFER_SIZE];
uint8_t * pt1 = TxBuffer;
uint8_t RxBuffer[BUFFER_SIZE];
uint8_t * pt2 = RxBuffer;

States_t State = LOWPOWER;
uint16_t Txcount;
uint8_t TxState = 0;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Current time object*/
static TimerTime_t Currenttime;

/* Round trip time object*/
static TimerTime_t Roundtrip;

 /* Led Timers objects*/
static  TimerEvent_t timerLed;

/* Radio Transmission objects*/
static  TimerEvent_t timerRadioTx;

/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on when led timer elapses
 */
static void OnledEvent( void );
/**
 * Main application entry point.
 */

/*!
 * \brief Function executed on when radio transmission timer elapses
 */
static void OnRadioTransmission( void );

/*!
 * \brief Function to copy Data and Key words (Start and End) to the Payload buffer
 */
int My_Strcat(uint8_t * Destination_ptr, uint8_t * Source_ptr);

/*!
 * \brief Function to initialize USER Push Button for External Interrupt
 */
void Push_Button_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
/*!
 * \brief Function to handle the External Interrupt from the USER Push Button
 */
void EXTI2_IRQHandler(void);

/*!
 * \brief Main Function
 */
int main( void )
{
//  bool isMaster = true;
  uint8_t i;

  HAL_Init( );
  
  SystemClock_Config( );
  
  DBG_Init( );

  HW_Init( );
  //PRINTF(" INIT'G SD CSRD ");

//  SC_CARD_Init();
  //PRINTF(" SD CARD INIT'D ");

  Push_Button_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /* Led Timers*/
  TimerInit(&timerLed, OnledEvent);   
  TimerSetValue( &timerLed, LED_PERIOD_MS);
  TimerStart(&timerLed );

  /* Radio Transmission timers*/
  TimerInit(&timerRadioTx, OnRadioTransmission);
  TimerSetValue( &timerRadioTx, RADIO_TX_PERIOD_MS);
//  TimerStart(&timerRadioTx );
//  TimerStop(&timerRadioTx );

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init( &RadioEvents );

  Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000000 );
    
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000000 );
    
  Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

  /* Appending Start key word, Data and end key word to the Tx buffer */
  My_Strcat(pt1, qt1);

  for(i = 0; i < (sizeof(Data)/sizeof(Data[0])); i++ )
  {
  	My_Strcat(pt1, Data[i]);
  }

  My_Strcat(pt1, qt3);

  uint8_t PB_Previous_State = HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_EXTI_LINE);

  //PRINTF("INIT COMPLETED And PushButton reads %u\n\r", PB_Previous_State);

  while( 1 )
  {
	  switch( State )
	  {
	  	  case RX:
	  		 Radio.Rx( RX_TIMEOUT_VALUE );
	  		 State = LOWPOWER;
	  		 break;
	  	  case TX:
	  		 Radio.Rx( RX_TIMEOUT_VALUE );
	  		 State = LOWPOWER;
	  		 break;
	  	  case RX_TIMEOUT:
	  		 Radio.Rx( RX_TIMEOUT_VALUE );
	  		 State = LOWPOWER;
	  		 break;
	  	  case RX_ERROR:
//	  		 Radio.Send( TxBuffer, BufferSize );
//	  		 Currenttime = TimerGetCurrentTime();
//	  		 TimerStart(&timerRadioTx );
	  		 Radio.Rx( RX_TIMEOUT_VALUE );
	  		 State = LOWPOWER;
	  		 break;
	  	  case TX_TIMEOUT:
//	  		 Radio.Send( TxBuffer, BufferSize );
//	  		 Currenttime = TimerGetCurrentTime();
//	  		 TimerStart(&timerRadioTx );
	  		 Radio.Rx( RX_TIMEOUT_VALUE );
	  		 State = LOWPOWER;
	  		 break;
	  	  case LOWPOWER:
	  		 default:
	  			 // Set low power
	  		 break;
	  }

	  DISABLE_IRQ( );
	  /* if an interupt has occured after __disable_irq, it is kept pending
	  * and cortex will not enter low power anyway  */
	  if (State == LOWPOWER)
	  {
		  #ifndef LOW_POWER_DISABLE
	  		      LowPower_Handler( );
	  	  #endif
	  }
	  ENABLE_IRQ( );


  }
}

void OnTxDone( void )
{
	Currenttime = TimerGetCurrentTime();
    Radio.Sleep( );
    State = TX;

    PRINTF("OnTxDone\n\r");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	Roundtrip = TimerGetElapsedTime(Currenttime);
    Radio.Sleep( );
//    BufferSize = size;
    memcpy( RxBuffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    // Indicates on a Red LED that the Acknowledgment is Received
    TimerStop(&timerLed );
    LED_Off( LED_BLUE);
    LED_Off( LED_GREEN );
    LED_Off( LED_RED1 );
    LED_Toggle( LED_RED2 );

    PRINTF("OnRxDone \t ||");
    PRINTF("Acknowledgment Received is: %s \t || \t Round Trip time = %d ms \t || \t RssiValue=%d dBm \t || \t SnrValue=%d \t ||", RxBuffer, Roundtrip, rssi, snr);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
  
    PRINTF("OnTxTimeout; ReTxing \n\r");
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    PRINTF("OnRxTimeout\n\r");
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    PRINTF("OnRxError\n\r");
}

static void OnledEvent( void )
{
  LED_Toggle( LED_BLUE ) ; 
  LED_Toggle( LED_RED1 ) ; 
  LED_Toggle( LED_RED2 ) ; 
  LED_Toggle( LED_GREEN ) ;   

  TimerStart(&timerLed );
}
static void OnRadioTransmission( void )
{
	PRINTF("...Txing the %u th payload \t ||", Txcount++);
	DelayMs( 1 );
	Radio.Send( TxBuffer, BufferSize );
	Currenttime = TimerGetCurrentTime();
	Radio.Rx( RX_TIMEOUT_VALUE );
	// Indicate on Blue LEd that Payload has be
	TimerStop(&timerLed );
	    LED_Off( LED_RED1);
	    LED_Off( LED_RED2 ) ;
	    LED_Off( LED_GREEN ) ;
	    LED_Toggle( LED_BLUE );

	PRINTF("TX Power = %d dBm \n\r",TX_OUTPUT_POWER );
	State = RX;
	TxState = 1;
	TimerStart(&timerRadioTx );
}

int My_Strcat(uint8_t * Destination_ptr, uint8_t * Source_ptr)
{
  (*Destination_ptr)? My_Strcat(++Destination_ptr, Source_ptr): (*Destination_ptr++ = *Source_ptr++)&&My_Strcat(Destination_ptr, Source_ptr);
  return 0;
}

void Push_Button_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the BUTTON Clock */
	BUTTONx_GPIO_CLK_ENABLE(Button);
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	if(ButtonMode == BUTTON_MODE_GPIO)
	{
	 /* Configure Button pin as input */
	 GPIO_InitStruct.Pin = KEY_BUTTON_PIN;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 HAL_GPIO_Init(KEY_BUTTON_GPIO_PORT, &GPIO_InitStruct);
	}

	if(ButtonMode == BUTTON_MODE_EXTI)
	{
	 /* Configure Button pin as input with External interrupt */
	 GPIO_InitStruct.Pin = KEY_BUTTON_EXTI_LINE;
	 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	 GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	 HAL_GPIO_Init(KEY_BUTTON_GPIO_PORT, &GPIO_InitStruct);

	 /* Select PB2 as the EXTI Interrupt's source */
	 RCC-> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	 SYSCFG -> EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2;
	 SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;

	 /* Enable and set Button EXTI Interrupt to the lowest priority */
	 NVIC_SetPriority((IRQn_Type)(KEY_BUTTON_EXTI_IRQn), 0x03);
	 HAL_NVIC_EnableIRQ((IRQn_Type)(KEY_BUTTON_EXTI_IRQn));
	 }
}

void EXTI2_3_IRQHandler(void)
{
	/* EXTI line interrupt detected */
	if((EXTI -> PR & EXTI_PR_PIF2) != RESET)
	{
	  EXTI -> PR |= EXTI_PR_PIF2;

	  if(TxState == 0)
	  {
		  /* Starting Tx timers */
		  TimerStart(&timerRadioTx );
		  PRINTF("Tx Timer Started !!!!!\n\r");
		  TimerStop(&timerLed );
		  Radio.Rx( RX_TIMEOUT_VALUE );
	  }
	  else
	  {
		  /* Stopping Tx timers */
		  TxState = 0;
		  TimerStop(&timerRadioTx );
		  PRINTF("Tx Timer Stopped !!!!!\n\r");
		  TimerStart(&timerLed );
		  Radio.Sleep();
	  }
	}
}

