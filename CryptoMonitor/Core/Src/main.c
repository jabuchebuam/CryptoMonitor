/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_touchgfx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Components/ili9341/ili9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REFRESH_COUNT           ((uint32_t)1386)   /* SDRAM refresh counter */
#define SDRAM_TIMEOUT           ((uint32_t)0xFFFF)

/**
 * @brief  FMC SDRAM Mode definition register defines
 */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define I2C3_TIMEOUT_MAX                    0x3000 /*<! The value of the maximal timeout for I2C waiting loops */
#define SPI5_TIMEOUT_MAX                    0x1000
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for GUI_Task */
osThreadId_t GUI_TaskHandle;
const osThreadAttr_t GUI_Task_attributes = {
		.name = "GUI_Task",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 8192 * 4
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI5_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
void TouchGFX_Task(void *argument);

/* USER CODE BEGIN PFP */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);

void program_run(void);
void send_alarm(void);

static uint8_t            I2C3_ReadData(uint8_t Addr, uint8_t Reg);
static void               I2C3_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t            I2C3_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);

/* SPIx bus function */
static void               SPI5_Write(uint16_t Value);
static uint32_t           SPI5_Read(uint8_t ReadSize);
static void               SPI5_Error(void);

/* Link function for LCD peripheral */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteData(uint16_t RegValue);
void                      LCD_IO_WriteReg(uint8_t Reg);
uint32_t                  LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
void                      LCD_Delay(uint32_t delay);

/* IOExpander IO functions */
void                      IOE_Init(void);
void                      IOE_ITConfig(void);
void                      IOE_Delay(uint32_t Delay);
void                      IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);

/* USER CODE END PFP */
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "a";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];


uint8_t Data_BTC_old [10] [11];
uint8_t Data_ETH_old [10] [9];

uint32_t Data_BTC[3];
uint32_t Data_ETH[3];
uint32_t Data_BTC_old_value [13];
uint32_t Data_ETH_old_value [13];
uint32_t Data_LTC_old_value [13];
uint32_t Data_BCH_old_value [13];
uint32_t Data_SOL_old_value [13];
uint32_t Data_ADA_old_value [13];
uint32_t Data_LINK_old_value [13];
uint32_t Data_GNO_old_value [13];

extern uint32_t Data_BTC_old_value [13];
extern uint32_t Data_ETH_old_value [13];
extern uint32_t Data_LTC_old_value [13];
extern uint32_t Data_BCH_old_value [13];
extern uint32_t Data_SOL_old_value [13];
extern uint32_t Data_ADA_old_value [13];
extern uint32_t Data_LINK_old_value [13];
extern uint32_t Data_GNO_old_value [13];

extern uint32_t data[10];


/* Private function prototypes -----------------------------------------------*/
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static LCD_DrvTypeDef* LcdDrv;

uint32_t I2c3Timeout = I2C3_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */  
uint32_t Spi5Timeout = SPI5_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */  
/* USER CODE END 0 */


/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (100)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_CRC_Init();
	MX_I2C3_Init();
	MX_SPI5_Init();
	MX_FMC_Init();
	MX_LTDC_Init();
	MX_DMA2D_Init();
	MX_TouchGFX_Init();

	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART1 clock */
	USARTx_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	UartHandle.Instance          = USARTx;

	UartHandle.Init.BaudRate     = 9600;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}


	/* Init scheduler */
	osKernelInitialize();

	/* creation of GUI_Task */
	GUI_TaskHandle = osThreadNew(TouchGFX_Task, NULL, &GUI_Task_attributes);

	/* Start scheduler */
	osKernelStart();

	while (1)
	{
		HAL_Delay(200);
	}
	/* USER CODE END 3 */
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	/* Turn LED4 on: Transfer error in reception/transmission process */
	//BSP_LED_On(LED4);
}

void send_alarm(void)
{
	if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 50)!= HAL_OK)
	{
		Error_Handler();
	}
}

void program_run(void)
{
	uint8_t aux,i;
	uint32_t value_temp, max_temp;


	if(HAL_UART_Receive(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 100) == HAL_OK)
	{
	}

	if((aRxBuffer[0]=='B') && (aRxBuffer[1]=='T') && (aRxBuffer[2]=='C'))
	{
		value_temp = 0;
		value_temp = (aRxBuffer[4] - '0')*10000;
		value_temp = ((aRxBuffer[5] - '0')*1000) + value_temp;
		value_temp = ((aRxBuffer[6] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[7] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[8] - '0')*1) + value_temp;
		Data_BTC_old_value[10] = value_temp;

		value_temp = (aRxBuffer[10] - '0')*10000;
		value_temp = ((aRxBuffer[11] - '0')*1000) + value_temp;
		value_temp = ((aRxBuffer[12] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[13] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[14] - '0')*1) + value_temp;
		Data_BTC_old_value[11] = value_temp;

		value_temp = (aRxBuffer[16] - '0')*10000;
		value_temp = ((aRxBuffer[17] - '0')*1000) + value_temp;
		value_temp = ((aRxBuffer[18] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[19] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[20] - '0')*1) + value_temp;
		Data_BTC_old_value[12] = value_temp;
	}

	if((aRxBuffer[0]=='E') && (aRxBuffer[1]=='T') && (aRxBuffer[2]=='H'))
	{
		value_temp = 0;
		value_temp = (aRxBuffer[4] - '0')*1000;
		value_temp = ((aRxBuffer[5] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[6] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[7] - '0')) + value_temp;
		Data_ETH_old_value[10] = value_temp;

		value_temp = (aRxBuffer[9] - '0')*1000;
		value_temp = ((aRxBuffer[10] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[11] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[12] - '0')) + value_temp;
		Data_ETH_old_value[11] = value_temp;

		value_temp = (aRxBuffer[14] - '0')*1000;
		value_temp = ((aRxBuffer[15] - '0')*100) + value_temp;
		value_temp = ((aRxBuffer[16] - '0')*10) + value_temp;
		value_temp = ((aRxBuffer[17] - '0')) + value_temp;
		Data_ETH_old_value[12] = value_temp;
	}

	if((aRxBuffer[0]=='L') && (aRxBuffer[1]=='T') && (aRxBuffer[2]=='C'))
	{
		value_temp = 0;
		value_temp = (aRxBuffer[4] - '0')*100;
		value_temp = ((aRxBuffer[5] - '0')*10) + value_temp;
		value_temp = (aRxBuffer[6] - '0') + value_temp;
		Data_LTC_old_value[10] = value_temp;

		value_temp = ((aRxBuffer[8] - '0')*100);
		value_temp = ((aRxBuffer[9] - '0')*10) + value_temp;
		value_temp = (aRxBuffer[10] - '0') + value_temp;
		Data_LTC_old_value[11] = value_temp;

		value_temp = ((aRxBuffer[12] - '0')*100);
		value_temp = ((aRxBuffer[13] - '0')*10) + value_temp;
		value_temp = (aRxBuffer[14] - '0') + value_temp;
		Data_LTC_old_value[12] = value_temp;
	}

	for(i=0; i<10;i++)
	{
		aux = i + '0';
		if((aRxBuffer[0]==aux) && (aRxBuffer[1]=='B') && (aRxBuffer[2]=='T') && (aRxBuffer[3]=='C'))
		{
			memcpy(Data_BTC_old[i],aRxBuffer,11);
			value_temp = 0;
			value_temp = (aRxBuffer[5] - '0')*10000;
			value_temp = ((aRxBuffer[6] - '0')*1000) + value_temp;
			value_temp = ((aRxBuffer[7] - '0')*100) + value_temp;
			value_temp = ((aRxBuffer[8] - '0')*10) + value_temp;
			value_temp = ((aRxBuffer[9] - '0')*1) + value_temp;
			Data_BTC_old_value[i] = value_temp;
		}

		if((aRxBuffer[0]==aux) && (aRxBuffer[1]=='E') && (aRxBuffer[2]=='T') && (aRxBuffer[3]=='H'))
		{
			value_temp = 0;
			value_temp = ((aRxBuffer[5] - '0')*1000) + value_temp;
			value_temp = ((aRxBuffer[6] - '0')*100) + value_temp;
			value_temp = ((aRxBuffer[7] - '0')*10) + value_temp;
			value_temp = ((aRxBuffer[8] - '0')) + value_temp;
			Data_ETH_old_value[i] = value_temp;
		}

		if((aRxBuffer[0]==aux) && (aRxBuffer[1]=='L') && (aRxBuffer[2]=='T') && (aRxBuffer[3]=='C'))
		{
			value_temp = 0;
			value_temp = ((aRxBuffer[5] - '0')*100) + value_temp;
			value_temp = ((aRxBuffer[6] - '0')*10) + value_temp;
			value_temp = ((aRxBuffer[7] - '0')) + value_temp;
			Data_LTC_old_value[i] = value_temp;
		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void)
{

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void)
{

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = {0};

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 9;
	hltdc.Init.VerticalSync = 1;
	hltdc.Init.AccumulatedHBP = 29;
	hltdc.Init.AccumulatedVBP = 3;
	hltdc.Init.AccumulatedActiveW = 269;
	hltdc.Init.AccumulatedActiveH = 323;
	hltdc.Init.TotalWidth = 279;
	hltdc.Init.TotalHeigh = 327;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK)
	{
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 240;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 320;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg.FBStartAdress = 0;
	pLayerCfg.ImageWidth = 240;
	pLayerCfg.ImageHeight = 320;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */
	/*Select the device */
	LcdDrv = &ili9341_drv;
	/* LCD Init */
	LcdDrv->Init();

	LcdDrv->DisplayOff();
	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void)
{

	/* USER CODE BEGIN SPI5_Init 0 */

	/* USER CODE END SPI5_Init 0 */

	/* USER CODE BEGIN SPI5_Init 1 */

	/* USER CODE END SPI5_Init 1 */
	/* SPI5 parameter configuration*/
	hspi5.Instance = SPI5;
	hspi5.Init.Mode = SPI_MODE_MASTER;
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi5.Init.NSS = SPI_NSS_SOFT;
	hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi5.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi5) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI5_Init 2 */



	/* USER CODE END SPI5_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = {0};

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 7;
	SdramTiming.SelfRefreshTime = 4;
	SdramTiming.RowCycleDelay = 7;
	SdramTiming.WriteRecoveryTime = 3;
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
	{
		Error_Handler( );
	}

	/* USER CODE BEGIN FMC_Init 2 */

	FMC_SDRAM_CommandTypeDef command;

	/* Program the SDRAM external device */
	BSP_SDRAM_Initialization_Sequence(&hsdram1, &command);
	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PG13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */
/**
 * @brief  Perform the SDRAM external memory initialization sequence
 * @param  hsdram: SDRAM handle
 * @param  Command: Pointer to SDRAM command structure
 * @retval None
 */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
	__IO uint32_t tmpmrd =0;

	/* Step 1:  Configure a clock configuration enable command */
	Command->CommandMode             = FMC_SDRAM_CMD_CLK_ENABLE;
	Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber       = 1;
	Command->ModeRegisterDefinition  = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Step 2: Insert 100 us minimum delay */
	/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
	HAL_Delay(1);

	/* Step 3: Configure a PALL (precharge all) command */
	Command->CommandMode             = FMC_SDRAM_CMD_PALL;
	Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber       = 1;
	Command->ModeRegisterDefinition  = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Step 4: Configure an Auto Refresh command */
	Command->CommandMode             = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber       = 4;
	Command->ModeRegisterDefinition  = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Step 5: Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
			SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
			SDRAM_MODEREG_CAS_LATENCY_3           |
			SDRAM_MODEREG_OPERATING_MODE_STANDARD |
			SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command->CommandMode             = FMC_SDRAM_CMD_LOAD_MODE;
	Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber       = 1;
	Command->ModeRegisterDefinition  = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Step 6: Set the refresh rate counter */
	/* Set the device refresh rate */
	HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}

/**
 * @brief  IOE Low Level Initialization.
 */
void IOE_Init(void) 
{
	//Dummy function called when initializing to stmpe811 to setup the i2c.
	//This is done with cubmx and is therfore not done here.
}

/**
 * @brief  IOE Low Level Interrupt configuration.
 */
void IOE_ITConfig(void)
{
	//Dummy function called when initializing to stmpe811 to setup interupt for the i2c.
	//The interupt is not used in our case, therefore nothing is done here.
}

/**
 * @brief  IOE Writes single data operation.
 * @param  Addr: I2C Address
 * @param  Reg: Reg Address
 * @param  Value: Data to be written
 */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	I2C3_WriteData(Addr, Reg, Value);
}

/**
 * @brief  IOE Reads single data.
 * @param  Addr: I2C Address
 * @param  Reg: Reg Address
 * @retval The read data
 */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
	return I2C3_ReadData(Addr, Reg);
}

/**
 * @brief  IOE Reads multiple data.
 * @param  Addr: I2C Address
 * @param  Reg: Reg Address
 * @param  pBuffer: pointer to data buffer
 * @param  Length: length of the data
 * @retval 0 if no problems to read multiple data
 */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	return I2C3_ReadBuffer(Addr, Reg, pBuffer, Length);
}

/**
 * @brief  IOE Delay.
 * @param  Delay in ms
 */
void IOE_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/**
 * @brief  Writes a value in a register of the device through BUS.
 * @param  Addr: Device address on BUS Bus.
 * @param  Reg: The target register address to write
 * @param  Value: The target register value to be written
 */
static void I2C3_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2c3Timeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		//I2Cx_Error();
	}
}

/**
 * @brief  Reads a register of the device through BUS.
 * @param  Addr: Device address on BUS Bus.
 * @param  Reg: The target register address to write
 * @retval Data read at register address
 */
static uint8_t I2C3_ReadData(uint8_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;

	status = HAL_I2C_Mem_Read(&hi2c3, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2c3Timeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		//I2Cx_Error();

	}
	return value;
}

/**
 * @brief  Reads multiple data on the BUS.
 * @param  Addr: I2C Address
 * @param  Reg: Reg Address
 * @param  pBuffer: pointer to read data buffer
 * @param  Length: length of the data
 * @retval 0 if no problems to read multiple data
 */
static uint8_t I2C3_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2c3Timeout);

	/* Check the communication status */
	if(status == HAL_OK)
	{
		return 0;
	}
	else
	{
		/* Re-Initialize the BUS */
		//I2Cx_Error();

		return 1;
	}
}

/**
 * @brief  Reads 4 bytes from device.
 * @param  ReadSize: Number of bytes to read (max 4 bytes)
 * @retval Value read on the SPI
 */
static uint32_t SPI5_Read(uint8_t ReadSize)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t readvalue;

	status = HAL_SPI_Receive(&hspi5, (uint8_t*) &readvalue, ReadSize, Spi5Timeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		SPI5_Error();
	}

	return readvalue;
}

/**
 * @brief  Writes a byte to device.
 * @param  Value: value to be written
 */
static void SPI5_Write(uint16_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_Transmit(&hspi5, (uint8_t*) &Value, 1, Spi5Timeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		SPI5_Error();
	}
}

/**
 * @brief  SPI5 error treatment function.
 */
static void SPI5_Error(void)
{
	/* De-initialize the SPI communication BUS */
	//HAL_SPI_DeInit(&SpiHandle);

	/* Re- Initialize the SPI communication BUS */
	//SPIx_Init();
}

void LCD_IO_Init(void)
{
	/* Set or Reset the control line */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
 * @brief  Writes register value.
 */
void LCD_IO_WriteData(uint16_t RegValue) 
{
	/* Set WRX to send data */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

	/* Reset LCD control line(/CS) and Send data */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	SPI5_Write(RegValue);

	/* Deselect: Chip Select high */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
 * @brief  Writes register address.
 */
void LCD_IO_WriteReg(uint8_t Reg) 
{
	/* Reset WRX to send command */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	/* Reset LCD control line(/CS) and Send command */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	SPI5_Write(Reg);

	/* Deselect: Chip Select high */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
 * @brief  Reads register value.
 * @param  RegValue Address of the register to read
 * @param  ReadSize Number of bytes to read
 * @retval Content of the register value
 */
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) 
{
	uint32_t readvalue = 0;

	/* Select: Chip Select low */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

	/* Reset WRX to send command */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	SPI5_Write(RegValue);

	readvalue = SPI5_Read(ReadSize);

	/* Set WRX to send data */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

	/* Deselect: Chip Select high */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

	return readvalue;
}

/**
 * @brief  Wait for loop in ms.
 * @param  Delay in ms.
 */
void LCD_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
 * @brief  Function implementing the GUI_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
