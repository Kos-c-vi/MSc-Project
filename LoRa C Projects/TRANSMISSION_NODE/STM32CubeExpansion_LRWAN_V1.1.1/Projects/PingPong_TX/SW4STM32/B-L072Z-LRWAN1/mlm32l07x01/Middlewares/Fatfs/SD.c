/*
 * SD.c
 *
 *  Created on: 05 Oct 2017
 *      Author: Kossivi Fangbem
 *      Brief : Define I/O function for the SPI-SD Card module.
 *      		Adopted from http://www.8051projects.net/mmc-sd-interface-fat16/spi-functions.php#D
 */

#include "hw.h"
#include "SD.h"
#include "integer.h"
#include "timeServer.h"
#include "delay.h"
#include "vcom.h"

/* SPI2 Configuration variable*/
SPI_HandleTypeDef hspi2;
//static SPI_HandleTypeDef hspi_2;

void SystemClock_Config_0(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(uint32_t Freq_Hz);
/*!
 * @brief Function to set SPI2'S BAUDE RATE
 */
static uint32_t Spi2Frequency( uint32_t hz );

typedef union
{
   BYTE Index[6];
   struct
      {
      BYTE Command;
      DWORD Argument;
      BYTE Cksum;
   } CA;
} CommandStructure;

typedef union
{
   BYTE Byte_Arg[4];
   DWORD Arg;
} ArgumentStructure;

void SPI2_Init(uint32_t SPI2_Speed_Hz)
{
	/* Clock enable for GPIOB */
	RCC->IOPENR = RCC_IOPENR_GPIOBEN;

	/* Configuring PB13, PB14 and PB15 for Clock, MISO and MOSI */
	// Alternative function mode
	GPIOB->MODER |= GPIO_MODER_MODE13_1;
	GPIOB->MODER |= GPIO_MODER_MODE14_1;
	GPIOB->MODER |= GPIO_MODER_MODE15_1;
	// Very high speed
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED13;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED14;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED15;
	// Pull up resistor
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD13_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD14_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD15_0;
	// Alternative function selection for the pins
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFRH5;
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFRH6;
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFRH7;

	/* Configuring PB12 as output mode for the Chip Select function */
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED12;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD12_0;
	// Setting the CS bit High
	GPIOB->BSRR |= GPIO_BSRR_BS_12;

	/* Clock enable for SPI2 */
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	/* Configuring SPI2's parameters */
	if(SPI2_Speed_Hz <= 400000)
		SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1; // Baud Rate
	else
		SPI2->CR1 &= ~SPI_CR1_BR;

	SPI2->CR1 &= ~SPI_CR1_CPHA; // First edge
	SPI2->CR1 &= ~SPI_CR1_CPOL; // 0 when idle
	SPI2->CR1 |= SPI_CR1_MSTR; // Master Mode
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // MSB first
	SPI2->CR1 |= SPI_CR1_SSM; // Software management for the CS
	SPI2->CR1 &= ~SPI_CR1_DFF; // 8 Bits data format
	SPI2->CR1 &= ~SPI_CR1_CRCEN; // CRC Calculation disable
	SPI2->CR1 &= ~SPI_CR1_BIDIMODE; // Full-duplex communication

	/* ENABLE SPI2 */
	SPI2->CR1 |= SPI_CR1_SPE;
}
/*!
 * @brief Sends outData and receives inData using SPI2
 */
uint8_t HW_SPI2_InOut( uint8_t txData )
{
	uint8_t rxData;
//	/* Ensure that SPI is not busy */
//	while(((SPI2->SR) & SPI_SR_BSY) == (0x01U << (7U)));
//	/* Wait for the previous tx transfer */
//	while(((SPI2->SR) & SPI_SR_TXE) == 0x00);
//
//	/* Load data into the tx Buffer */
//	SPI2->DR = txData;
//
//	/* Wait for to copy the received data */
//	while(((SPI2->SR) & SPI_SR_RXNE) == 0x00);
//
////	DelayMs(10);
//
//	/* Load the received data */
//	rxData = SPI2->DR;
//
//	return rxData;


//	uint8_t rxData ;
//
	HAL_SPI_TransmitReceive( &hspi2, ( uint8_t * ) &txData, ( uint8_t* ) &rxData, 1, HAL_MAX_DELAY);

	return rxData;
}


static uint32_t Spi2Frequency( uint32_t hz )
{
  uint32_t divisor = 0;
  SystemCoreClockUpdate();
  uint32_t SysClkTmp = SystemCoreClock;
  uint32_t baudRate;

  while( SysClkTmp > hz)
  {
    divisor++;
    SysClkTmp= ( SysClkTmp >> 1);

    if (divisor >= 7)
      break;
  }

  baudRate =((( divisor & 0x4 ) == 0 )? 0x0 : SPI_CR1_BR_2  )|
            ((( divisor & 0x2 ) == 0 )? 0x0 : SPI_CR1_BR_1  )|
            ((( divisor & 0x1 ) == 0 )? 0x0 : SPI_CR1_BR_0  );

  return baudRate;
}

void SPI_NSS_ENABLE(void)
{
//	HW_GPIO_Write ( GPIOB, GPIO_PIN_12, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	GPIOB->BSRR |= GPIO_BSRR_BR_12;
}

void SPI_NSS_DISABLE(void)
{
//	HW_GPIO_Write ( GPIOB, GPIO_PIN_12, GPIO_PIN_SET );
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	GPIOB->BSRR |= GPIO_BSRR_BS_12;
}

/*
   * Brief: Function to send Command to the SD Card
*/
BYTE Send_SD_Command( BYTE The_Command, DWORD The_Argument)
{
	ArgumentStructure argument;
	uint8_t data_0xFF = 0xFF;

	argument.Arg = The_Argument;

	/* Enable the SD Card from SPI communication by resetting CS pin  */
	SPI_NSS_ENABLE();

	/* Send Clock to ensure that no operation is pending */
	HW_SPI2_InOut( data_0xFF);

	/* Send Command to SD Card */
	uint8_t Command = 0x40 | The_Command;
	HW_SPI2_InOut( Command );

	/* Send Argument of the Command to SD Card */
	for(int i = 0; i < 4; i++)
	{
		HW_SPI2_InOut( argument.Byte_Arg[i] );
	}

	/* Send CRC if Command is CMD_GO_IDLE_STATE OR CMD_SEND_IF_COND */
	HW_SPI2_InOut((The_Command == CMD_GO_IDLE_STATE)? 0x95:0xFF);
	HW_SPI2_InOut((The_Command == 0x08)? 0x0F:0xFF);

	/* Send Clock to ensure that no operation is pending */
	HW_SPI2_InOut( data_0xFF);

	return(0);
}

/*
   * Brief: Function1 to get Command responses R1 from the SD Card
*/
BYTE Get_SD_Response1( void )
{
	int Count = 1000;
		uint8_t resp;
		uint8_t data_0xFF = 0xFF;

		while (Count--) {
			resp = HW_SPI2_InOut( data_0xFF );
			if ((resp & 0x80) == 0)
				return resp;
		}
		return 0xff;
//	BYTE j;
//
//	for( int i=0; i<8; i++ )
//	   {                        /* response will be after 1-8 0xffs.. */
//	      j = HW_SPI2_InOut( 0xFF );
//	      if(j != 0xff)         /* if it isn't 0xff, it is a response */
//	         return(j);
//	   }
//	   return(j);
}

/*
   * Brief: Function2 to get Command responses R2 from the SD Card
*/
BYTE Get_SD_Response2( void )
{
	UINT ResponseR2;
	uint8_t data_0xFF = 0xFF;

	ResponseR2 = ((Get_SD_Response1()) << 8) & 0xff00;
	ResponseR2 |= HW_SPI2_InOut( data_0xFF );

	return( ResponseR2 );
}

/*
   * Brief: Function2 to get Command responses R3/R7 from the SD Card
*/
BYTE Get_SD_Response3_7( uint32_t *resp3_7 )
{

	UINT ResponseR3_7;
	uint8_t data_0xFF = 0xFF;

	ResponseR3_7 = Get_SD_Response1();
	if (ResponseR3_7 != 0x01)
		return ResponseR3_7;

	ResponseR3_7 = HW_SPI2_InOut( data_0xFF ) << 24;
	ResponseR3_7 = HW_SPI2_InOut( data_0xFF ) << 16;
	ResponseR3_7 = HW_SPI2_InOut( data_0xFF ) << 8;
	ResponseR3_7 = HW_SPI2_InOut( data_0xFF ) << 0;

	*resp3_7 = ResponseR3_7;

	return 0x01;
}

/*
   * Brief: Function to get time for stamping the files
*/
DWORD get_fattime( void )
{

}

/*
   * Brief: Function to ensure that SD Card is ready to execute next command
*/
BYTE  SD_Ready_Next_Command(void)
{
	BYTE response;
	UINT Count = 500;
	uint8_t data_0xFF = 0xFF;

	response = HW_SPI2_InOut( data_0xFF );
	while((response != 0xFF) && --Count)
		{
			response = HW_SPI2_InOut( 0xFF );
			DelayMs(1);
		}
	return response;
}

/*
   * Brief: Function to read a sector from the SD CARD
*/
BYTE SD_ReadSector( DWORD SectorNumber, BYTE *Buffer)
{
	BYTE RESP1, RESP2;
	UINT count;
	uint8_t data_0xFF = 0xFF;

	/* send block-read command... */
	Send_SD_Command( CMD_READ_SINGLE_BLOCK, SectorNumber << 9 );
	RESP1 = Get_SD_Response1();
	RESP2 = Get_SD_Response1();
	count = 0xFFFF;

	/* wait for data token... */
	while( (RESP2 == 0xff) && --count)
		RESP2 = Get_SD_Response1();

	/* handle time out... */
	if(RESP1 || RESP2 != 0xFE)
	    return( 1 );

	/* read the sector... */
	BYTE SD_DATA_SIZE = sizeof(Buffer)/sizeof(BYTE);
	for( count=0; count<SD_DATA_SIZE; count++)
	    *Buffer++ = HW_SPI2_InOut(data_0xFF);

	/* Ignore the checksum... */
	HW_SPI2_InOut(data_0xFF);
	HW_SPI2_InOut(data_0xFF);

	/* release the CS line... */
	SPI_NSS_DISABLE();
	HW_SPI2_InOut( data_0xFF );

	   return( 0 );
}
/*
   * Brief: Function to write to a sector from the SD CARD
*/
BYTE SD_WriteSector( DWORD SectorNumber, BYTE *Buffer)
{
	BYTE RESP;
	WORD count;
	uint8_t data_0xFF = 0xFF;

	/* send block-write command... */
	Send_SD_Command( CMD_WRITE_SINGLE_BLOCK, SectorNumber << 9 );
	RESP = Get_SD_Response1();

	/* send start block token... */
	uint8_t Token_data = 0xFE;
	HW_SPI2_InOut( Token_data );

	/* write the sector... */
	for( count= 0; count < 512; count++ )
	{
		HW_SPI2_InOut(*Buffer++);
	}

	/* ignore the checksum (dummy write)... */
	HW_SPI2_InOut(data_0xFF);
	HW_SPI2_InOut(data_0xFF);

	/* wait for response token... */
	while( HW_SPI2_InOut( data_0xFF ) != 0xFF)

	/* these 8 clock cycles are critical for the card */
	/* to finish up whatever it's working on at the */
	/* time... (before CS is released!) */
	HW_SPI2_InOut( data_0xFF );

	/* release the CS line... */
	SPI_NSS_DISABLE();
	HW_SPI2_InOut( data_0xFF );

	return( 0 );

}

/*
   * Bief: Function to INITIALIZE the SD CARD to interface with SPI
*/
BYTE SC_CARD_Init( void )
{
	BYTE CardStatus; // R2 value from status inquiry...
	UINT Count = 0;      // local counter
	uint32_t Resp3__7; // R7 response
	BYTE Resp1;  // Resp
	uint8_t data_0xFF = 0x55;
	uint8_t RxData;

	// Global CardType - b0:MMC, b1:SDv1, b2:SDv2

	SystemClock_Config_0();

	/* initial speed is slow... */
	MX_GPIO_Init();
	MX_SPI2_Init(400000);
	//SPI2_Init( 400000 );
	//PRINTF("SPI INIT");

	/* disable SPI chip select... */
	SPI_NSS_ENABLE();
//	SPI_NSS_DISABLE();

	/* fill send data with all ones - 96 bits long to   */
	/* establish link with SD card this fulfills the    */
	/* 74 clock cycle requirement...  */
	for(Count=0; Count < 5000; Count++)
	{
		uint16_t RespSPI;
		PRINTF("Count %d\n\r", Count++);
		RespSPI = HAL_SPI_TransmitReceive( &hspi2, &data_0xFF, &RxData, 1, HAL_MAX_DELAY);
		PRINTF("SPI working: %08d\n\r", RespSPI);
	}

	/* enable the card with the CS pin... */
	SPI_NSS_ENABLE();

//	/* ************************************************ */
//	/* SET SD CARD TO SPI MODE - IDLE STATE...          */
//	/* ************************************************ */
//	Count = 5000;     // one second...
//	BYTE CardType = 0;
//
//	/* wait for card to enter IDLE state... */
//	BYTE IDLE_STATE = 0x01;
//	do
//	{
//		 DelayMs(1);
//		 Send_SD_Command( CMD_GO_IDLE_STATE, 0 );
//	} while((Get_SD_Response1() != IDLE_STATE) && (--Count));
//
//	/* timeout if we never made it to IDLE state... */
//	if( !Count )
//	{
//		 PRINTF("CMD_GO_IDLE_STATE TimeOut!!!\n\r");
//		 return( 1 );
//	}
//	else
//	{
//		PRINTF("CMD_GO_IDLE_STATE OK !!!\n\r");
//	}
//
//	/* ************************************************ */
//	/* CHECK IF THE SD CARD'S VOLTAGE IS OK */
//	/* ************************************************ */
//	Send_SD_Command( 0x08, 0x000001AA );
//	PRINTF("CMD8\n\r");
//	Resp1 = Get_SD_Response3_7(&Resp3__7);
//	if( ( Resp1 &  0x01) == 0x01 )
//	{
//		PRINTF("CMD8 response OK \n\r");
//	}
//	else if( ( Resp1 &  0x05) == 0x05 )
//	{
//		PRINTF("CMD8 response ILLEGAL \n\r");
//	}
//
//	/* ************************************************ */
//	/* COMPLETE SD CARD INITIALIZATION                  */
//	/* FIGURE OUT WHAT TYPE OF CARD IS INSTALLED...     */
//	/* ************************************************ */
//	Count = 2000;     // two seconds...
//
//	/* Is card SDSC or MMC? */
//	Send_SD_Command( CMD_APP_CMD, 0 );
//	Send_SD_Command( ACMD_SEND_OP_COND, 0 );
//	if( Get_SD_Response1() <= 1 )
//	{
//		 CardType = 2;
//		 PRINTF("CARD type is %d\n\r",CardType);
//	}
//	else
//	{
//		 CardType = 1;
//	}
//
//	/* wait for initialization to finish... */
//	do
//	{
//		 Delay(1);
//	     if( CardType == 2 )
//	     {
//	    	 Send_SD_Command( CMD_APP_CMD, 0 );
//	    	 Send_SD_Command( ACMD_SEND_OP_COND, 0 );
//	     }
//	     else
//	     {
//	    	 Send_SD_Command( CMD_SEND_OP_COND, 0 );
//	     }
//	} while(Get_SD_Response1() && --Count);
//
//	if( !Count )
//		 return( 1 );
//
//	/* ************************************************ */
//	/* QUERY CARD STATUS...                             */
//	/* ************************************************ */
//	Send_SD_Command( CMD_SEND_STATUS, 0 );
//	CardStatus = Get_SD_Response2();
//	PRINTF("CARD STATUS IS %04x\n\r",CardStatus);
//
//	if( CardStatus )
//		 return( 2 );
//
//	/* ************************************************ */
//	/* SET BLOCK SIZE...                                */
//	/* ************************************************ */
//	Send_SD_Command( CMD_SET_BLOCKLEN, 512 );
//	if( Get_SD_Response1() )
//	{
//	      CardType = 0;
//	      return( 2 );
//	}
//
//	/* ************************************************ */
//	/* SWITCH TO HIGHEST SPI SPEED...                   */
//	/* ************************************************ */
//	SPI2_Init( 25000000 );
//
//	/* disable the card with the CS pin... */
//	SPI_NSS_DISABLE();

	/* return OK... */
	return 0;
}

/* SPI2 init function */
static void MX_SPI2_Init(uint32_t Freq_Hz)
{
	__HAL_RCC_SPI2_CLK_ENABLE();

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  if(Freq_Hz <= 400000)
	  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  else
	  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/** System Clock Configuration
*/
void SystemClock_Config_0(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
