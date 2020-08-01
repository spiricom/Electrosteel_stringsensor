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
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INV_TWO_TO_32 0.000000000232831f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SPI_TX[16] __ATTR_RAM_D2;
uint8_t SPI_RX[16] __ATTR_RAM_D2;
uint8_t counter;
int SDReady = 0;
uint64_t SDWriteIndex = 0;
FRESULT res;
  FATFS MMCFatFs;
  FIL myFile;
  FATFS fs;
  DSTATUS statusH;
  BYTE work[1024]; /* Work area (larger is better for processing time) */
  uint8_t workBuffer[1024];
int finishSD = 0;

int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t ADC_values[NUM_ADC_CHANNELS * ADC_BUFFER_SIZE] __ATTR_RAM_D2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU_Conf(void);
void SDRAM_Initialization_sequence(void);
static void FS_FileOperations(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  MPU_Conf();
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_RNG_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t tempFPURegisterVal = __get_FPSCR();
  tempFPURegisterVal |= (1<<24); // set the FTZ (flush-to-zero) bit in the FPU control register
  __set_FPSCR(tempFPURegisterVal);

  for (int i = 0; i < 16; i++)
  {
	  SPI_TX[i] = counter++;
  }

  HAL_SPI_TransmitReceive_DMA(&hspi2, SPI_TX, SPI_RX, 16);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(10);

  SDRAM_Initialization_sequence();
  HAL_Delay(10);


  int transmit_status = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
  int receive_status = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

 if(BSP_SD_IsDetected())
 {




   FS_FileOperations();

 }
 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_values,NUM_ADC_CHANNELS * ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  int tempIntGP = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if (tempIntGP)
	  {
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  }
	  else

	  {
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FMC
                              |RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 344;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#define SDRAM_TIMEOUT ((uint32_t)0xFFFF)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0003)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

//#define SDRAM_REFRESH_COUNT                   	 ((uint32_t)956)// 7.9us in cycles of 8.333333ns + 20 cycles as recommended by datasheet page 866/3289 for STM32H743
#define SDRAM_REFRESH_COUNT                   	 ((uint32_t)0x0569)// 7.9us in cycles of 8.333333ns + 20 cycles as recommended by datasheet page 866/3289 for STM32H743
void SDRAM_Initialization_sequence(void)
{
    __IO uint32_t tmpmrd = 0;
    FMC_SDRAM_CommandTypeDef Command;
    /* Step 1: Configure a clock configuration enable command */
    Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    Command.AutoRefreshNumber = 1;
    Command.ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

    /* Step 2: Insert 100 us minimum delay */
    /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
    HAL_Delay(1);

    /* Step 3: Configure a PALL (precharge all) command */
    Command.CommandMode = FMC_SDRAM_CMD_PALL;
    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    Command.AutoRefreshNumber = 1;
    Command.ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

    /* Step 5: Program the external memory mode register */
    tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_4 | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL
        | SDRAM_MODEREG_CAS_LATENCY_2 | SDRAM_MODEREG_OPERATING_MODE_STANDARD
        | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

    Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    Command.AutoRefreshNumber = 1;
    Command.ModeRegisterDefinition = tmpmrd;

    /* Send the command */
    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

    /* Step 4: Configure the 1st Auto Refresh command */
    Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    Command.AutoRefreshNumber = 8;
    Command.ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

    /* Step 2: Insert 100 us minimum delay */
    /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
    HAL_Delay(1);

    /* Step 5: Configure the 2nd Auto Refresh command */
    Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    Command.AutoRefreshNumber = 8;
    Command.ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

    /* Step 6: Set the refresh rate counter */
    /* Set the device refresh rate */
    HAL_SDRAM_ProgramRefreshRate(&hsdram1, SDRAM_REFRESH_COUNT);
}


float randomNumber(void) {

	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (float)rand * INV_TWO_TO_32;
	return num;
}
volatile FRESULT res2;
uint8_t rtext[100];                                   /* File read buffer */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
uint8_t wtext[48000]; /* File write buffer */
uint8_t tempText[30];
int testNumber = 55559;
int8_t filename[30];
uint8_t fileExt[] = ".txt";
static void FS_FileOperations(void)
{
  int theNumber = randomNumber() * 65535;
  itoa(theNumber,tempText, 10);
  strncat(filename, tempText, sizeof(tempText));
  strncat(filename, fileExt, sizeof(fileExt));
  statusH = disk_initialize(0);
  /* Register the file system object to the FatFs module */
  if(f_mount(&MMCFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
  {
	  {
		  if(f_open(&myFile, filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		  {
			SDReady = 1;
		  }
	  }
  }
}

uint8_t comma[] = ", ";
uint8_t newline[] = "\r\n";
uint64_t memoryPointer = 0;
char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;
void writeToSD(int theIndex, int theNumber, int whichString)
{
	if(finishSD == 1)
	{

		SDReady = 0;
		largeMemory[memoryPointer] = 0;
		memoryPointer++;

		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		__disable_irq();
		 f_write(&myFile, largeMemory, memoryPointer, (void *)&byteswritten);
		 res2 = f_close(&myFile);
		 __enable_irq();
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		 if (res2 == FR_OK)
		 {
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		 }

	}
	else if (whichString == 0)
	{
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(theIndex,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}

		largeMemory[memoryPointer] = 44;
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;



		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(theNumber,tempText, 10);
		for (int i = 0; i < 5; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

/*

		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(myPos,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}

		itoa(lh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 30; i++)
		{
			tempText[i] = 0;
		}
		itoa(rh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;
	*/
	}
	else if ((whichString > 0) && (whichString < 9))
	{
		for (int i = 0; i < 30; i++)
		{
			tempText[i] = 0;
		}
		itoa(theNumber,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

/*
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(myPos,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(lh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 30; i++)
		{
			tempText[i] = 0;
		}
		itoa(rh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;
		*/
	}
	else
	{
		for (int i = 0; i < 30; i++)
		{
			tempText[i] = 0;
		}
		itoa(theNumber,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}

		largeMemory[memoryPointer] = 32;
		memoryPointer++;
/*
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(myPos,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(lh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		for (int i = 0; i < 30; i++)
		{
			tempText[i] = 0;
		}
		itoa(rh,tempText, 10);
		for (int i = 0; i < 10; i++)
		{
			if (tempText[i] != 0)
			{
				largeMemory[memoryPointer] = tempText[i];
				memoryPointer++;
			}
		}
		largeMemory[memoryPointer] = 32;
		memoryPointer++;
		*/
		largeMemory[memoryPointer] = 59;
		memoryPointer++;
		largeMemory[memoryPointer] = 13;
		memoryPointer++;
		largeMemory[memoryPointer] = 10;
		memoryPointer++;

		SDWriteIndex++;
	}
}
void MPU_Conf(void)
{
	//code from Keshikan https://github.com/keshikan/STM32H7_DMA_sample
  //Thanks, Keshikan! This solves the issues with accessing the SRAM in the D2 area properly. -JS
	//should test the different possible settings to see what works best while avoiding needing to manually clear the cache -JS

	MPU_Region_InitTypeDef MPU_InitStruct;

	  HAL_MPU_Disable();

	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

	  //D2 Domain�SRAM1
	  MPU_InitStruct.BaseAddress = 0x30000000;
	  // Increased region size to 256k. In Keshikan's code, this was 512 bytes (that's all that application needed).
	  // Each audio buffer takes up the frame size * 8 (16 bits makes it *2 and stereo makes it *2 and double buffering makes it *2)
	  // So a buffer size for read/write of 4096 would take up 64k = 4096*8 * 2 (read and write).
	  // I increased that to 256k so that there would be room for the ADC knob inputs and other peripherals that might require DMA access.
	  // we have a total of 256k in SRAM1 (128k, 0x30000000-0x30020000) and SRAM2 (128k, 0x30020000-0x3004000) of D2 domain.
	  // There is an SRAM3 in D2 domain as well (32k, 0x30040000-0x3004800) that is currently not mapped by the MPU (memory protection unit) controller.

	  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

	  //AN4838
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

	  //Shared Device
//	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
//	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;


	  MPU_InitStruct.Number = MPU_REGION_NUMBER0;

	  MPU_InitStruct.SubRegionDisable = 0x00;


	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;


	  HAL_MPU_ConfigRegion(&MPU_InitStruct);


	  //now set up D3 domain RAM

	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

	 	  //D2 Domain�SRAM1
	 	  MPU_InitStruct.BaseAddress = 0x38000000;


	 	  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

	 	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

	 	  //AN4838
	 	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	 	  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	 	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	 	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

	 	  //Shared Device
	 //	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	 //	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	 //	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	 //	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;


	 	  MPU_InitStruct.Number = MPU_REGION_NUMBER1;

	 	  MPU_InitStruct.SubRegionDisable = 0x00;


	 	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;


	 	  HAL_MPU_ConfigRegion(&MPU_InitStruct);


	  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

uint16_t stringPositions[10];

void ADC_Frame(int offset)
{
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11);

	//sampRecords[currentSamp] = frameCount;
	//currentSamp++;
	for (int i = offset; i < ADC_FRAME_SIZE + offset; i++)
	{
		for (int j = 0; j < NUM_ADC_CHANNELS; j++)
		{
			int tempInt = ADC_values[(i*NUM_ADC_CHANNELS) + j];
			//float tempSamp = (((float)tempInt - INV_TWO_TO_15) * INV_TWO_TO_15);

			//stringPositions[j] =  ((uint16_t)SPI_RX[j * 2] << 8) + ((uint16_t)SPI_RX[(j * 2) + 1] & 0xff);
			//if (stringPositions[j] == 65535)
			//{
			//	stringPositions[j] = 0;
			//}
			//stringTouchLH[j] = (SPI_RX[8] >> j) & 1;
			//stringTouchRH[j] = (SPI_RX[8] >> (j + 4)) & 1;

			//tempSamp = tHighpass_tick(&opticalHighpass[j+NUM_STRINGS], tHighpass_tick(&opticalHighpass[j], tempSamp));
			//itoa(SDWriteIndex, wtext, 4);

			if (SDReady)
			{
				writeToSD(SDWriteIndex, tempInt, j);

				if (memoryPointer >= (LARGE_MEM_SIZE - 3000))
				{
					finishSD = 1;
					HAL_ADC_Stop(&hadc1);
					//HAL_SAI_DMAStop(&hsai_BlockA1);
					//HAL_SAI_DMAStop(&hsai_BlockB1);
				}
			}

/*			}
			for (int k = 0; k < FILTER_ORDER; k++)
			{
				//tempSamp = tHighpass_tick(&opticalHighpass[j+ (NUM_STRINGS * k)], tVZFilter_tick(&opticalLowpass[j + (NUM_STRINGS * k)], tempSamp));
				tempSamp = tHighpass_tick(&opticalHighpass[j+ (NUM_STRINGS * k)], tempSamp);
				//
			}
			tempSamp = tVZFilter_tick(&opticalLowpass[j], tempSamp) * 1.5f;

			attackDetect2(j, tempSamp);
			processString(j, tempSamp);
			*/
		}
		//whichVal++;

	}
	//if (whichVal > 3000)
	//{
	//	waitTimeOver = 1;
	//}
	//ADC_Ready = 1;

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{


	ADC_Frame(ADC_FRAME_SIZE);


}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADC_Frame(0);
}
void HAL_ADC_Error(ADC_HandleTypeDef *hadc)
{

}



/* USER CODE END 4 */

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