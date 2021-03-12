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
#include "leaf.h"
#define SDRECORD 0
#define SAMPLE_RATE 48000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SPI_TX[8] __ATTR_RAM_D2;
uint8_t SPI_RX[8] __ATTR_RAM_D2;

uint8_t SPI_PLUCK_TX[22] __ATTR_RAM_D2;


uint8_t counter;
int SDReady = 0;
uint64_t SDWriteIndex = 0;
FRESULT res;
  FATFS MMCFatFs;
  FIL myFile;
  FATFS fs;
  DSTATUS statusH;

int finishSD = 0;

int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t ADC_values[NUM_ADC_CHANNELS * ADC_BUFFER_SIZE] __ATTR_RAM_D2;

#define FILTER_ORDER 2
#define ATODB_TABLE_SIZE 25000
#define ATODB_TABLE_SIZE_MINUS_ONE 24999


#define MEDIUM_MEM_SIZE 300000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;

tMempool mediumPool;
#define LARGE_MEM_SIZE_TWOBYTE 16777216
uint8_t largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;
uint memoryPointer = 0;

int smoothedInt[NUM_STRINGS];
int smoothedInt2[NUM_STRINGS];
int dbSmoothedInt[NUM_STRINGS];
int tempAbsInt[NUM_STRINGS];
float atodbTable[ATODB_TABLE_SIZE];
int storedMaximums[NUM_STRINGS] = {16197, 10646, 10144, 5136, 8447, 27212, 6251, 6126, 5982, 4324};
float storedMaxFloats[NUM_STRINGS];
uint16_t stringPositions[NUM_ADC_CHANNELS];
uint16_t stringTouchRH[NUM_ADC_CHANNELS];
int didPlucked[NUM_ADC_CHANNELS];
int stringSounding[NUM_ADC_CHANNELS];

int howManyFrames = 48000;
int brokedIt = 0;
int didPlucked2[10];
int pluckDelay[10];
int pluckValues[10];


float Dsmoothed;
float Dsmoothed2;
float dbSmoothed2;

int armed[NUM_STRINGS] = {0,0,0,0,0,0,0,0,0,0};
float prevdbSmoothed2[NUM_STRINGS];
int threshOut = 0;
int stringMaxes[NUM_STRINGS] = {0,0,0,0,0,0,0,0,0,0};
int downCounter[NUM_STRINGS];
int armedCounter[NUM_STRINGS];
float slopeStorage[NUM_STRINGS];


tHighpass opticalHighpass[NUM_ADC_CHANNELS][FILTER_ORDER];
tVZFilter opticalLowpass[NUM_ADC_CHANNELS][FILTER_ORDER];
tSlide fastSlide[NUM_ADC_CHANNELS];
tSlide slowSlide[NUM_ADC_CHANNELS];
tThreshold threshold[NUM_ADC_CHANNELS];
LEAF leaf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU_Conf(void);
void SDRAM_Initialization_sequence(void);
static void FS_FileOperations(void);
float randomNumber(void);
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

  for (int i = 0; i < 8; i++)
  {
	  SPI_TX[i] = counter++;
  }

  for (int i = 0; i < 22; i++)
  {
	  SPI_PLUCK_TX[i] = 0;
  }
  SPI_PLUCK_TX[0] = 254;//start message
  SPI_PLUCK_TX[21] = 253;//end message


  HAL_SPI_Receive_DMA(&hspi2, SPI_RX, 8);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(10);

  SDRAM_Initialization_sequence();
  HAL_Delay(10);


  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

#if SDRECORD
 if(BSP_SD_IsDetected())
 {
   FS_FileOperations();
 }
#endif
LEAF_init(&leaf, SAMPLE_RATE, AUDIO_FRAME_SIZE, mediumMemory, MEDIUM_MEM_SIZE, &randomNumber);


for (int i = 0; i < NUM_ADC_CHANNELS; i++)
{
	tThreshold_init(&threshold[i],0.1f * (float)storedMaximums[i], 0.1f * (float)storedMaximums[i], &leaf);
	tSlide_init(&fastSlide[i],1.0f,500.0f, &leaf); //500
	tSlide_init(&slowSlide[i],1.0f,500.0f, &leaf); //500

	storedMaxFloats[i] = (65535.0f / storedMaximums[i]);
	for (int j = 0; j < FILTER_ORDER; j++)
	{
		tVZFilter_init(&opticalLowpass[i][j], Lowpass, 1000.0f, 0.6f, &leaf);
		tHighpass_init(&opticalHighpass[i][j], 100.0f, &leaf);
	}
}

LEAF_generate_atodb(atodbTable, ATODB_TABLE_SIZE);



 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_values,NUM_ADC_CHANNELS * ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //int tempIntGP = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  //if (tempIntGP)
	  //{
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  //}
	  //else

	  //{
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	 // }
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


float randomNumber(void) {

	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (float)rand * INV_TWO_TO_32;
	return num;
}


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

#ifdef SDRECORD

volatile FRESULT res2;
uint8_t rtext[100];                                   /* File read buffer */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
uint8_t wtext[220000] __ATTR_RAM_D1; /* File write buffer */
uint32_t wtextPointer = 0;
char tempText[30];
int testNumber = 55559;
char filename[40];
char fileExt[] = ".txt";
static void FS_FileOperations(void)
{
  uint theNumber = randomNumber() * 65535;
  itoa(theNumber,tempText, 10);
  strncat(filename, tempText, sizeof(tempText));
  strncat(filename, fileExt, sizeof(fileExt));
  statusH = disk_initialize(0);
  HAL_Delay(10);
  /* Register the file system object to the FatFs module */
  if(f_mount(&MMCFatFs, (TCHAR const*)SDPath, 1) == FR_OK)
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

void writeSD(void)
{
	if (SDReady == 1)
	{
		int maxPointer = memoryPointer;
		__disable_irq();
		for (int j = 0; j < maxPointer; j++)
		{
			//get number
			int theNumber = (int)largeMemory[j];
			//clear temp text
			for (int i = 0; i < 10; i++)
			{
				tempText[i] = 0;
			}
			itoa(theNumber,tempText, 10);

			//if you don't have room in the temp buffer, write it to the SD card
			if (wtextPointer > (220000 - 20))
			{
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

				 f_write(&myFile, wtext, wtextPointer, (void *)&byteswritten);
				 wtextPointer = 0;

			}
			for (int i = 0; i < 5; i++)
			{
				if (tempText[i] != 0)
				{
					wtext[wtextPointer] = tempText[i];
					wtextPointer++;
				}
			}
			if (((j + 1) % 50) == 0)
			{

				wtext[wtextPointer] = 59;
				wtextPointer++;
				wtext[wtextPointer] = 13;
				wtextPointer++;
				wtext[wtextPointer] = 10;

			}
			else
			{
				wtext[wtextPointer] = 32;
			}

			wtextPointer++;

		}
		res2 = f_close(&myFile);

		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		 if (res2 == FR_OK)
		 {
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		 }
		__enable_irq();
	}
}



void writeToSD(int theIndex, int theNumber, int whichString, int value)
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
	else if (whichString < 9)
	{

		/*(for (int i = 0; i < 10; i++)
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
*/

		//input value
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

		//filtered abs
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(tempAbsInt[whichString],tempText, 10);
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

		//smoothed1
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt[whichString],tempText, 10);
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

		//smoothed2
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt2[whichString],tempText, 10);
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

		//smoothed db
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(dbSmoothedInt[whichString],tempText, 10);
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

		//armed counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(armedCounter[whichString],tempText, 10);
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

		//down counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(downCounter[whichString],tempText, 10);
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

		//string touched
		if (stringTouchRH[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;



		//string sounding
		if (stringSounding[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

	}
		/*
		//filtered abs
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(tempAbsInt[whichString],tempText, 10);
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

		//smoothed1
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt[whichString],tempText, 10);
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

		//smoothed2
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt2[whichString],tempText, 10);
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

		//smoothed db
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(dbSmoothedInt[whichString],tempText, 10);
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

		//didPlucked
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(didPlucked[whichString],tempText, 10);
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

		//armed counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(armedCounter[whichString],tempText, 10);
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

		//down counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(downCounter[whichString],tempText, 10);
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

		//string Maxes
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(stringMaxes[whichString],tempText, 10);
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



		//string touched
		if (stringTouchRH[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;



		//string sounding
		if (stringSounding[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;

		//end
		largeMemory[memoryPointer] = 59;
		memoryPointer++;
		largeMemory[memoryPointer] = 13;
		memoryPointer++;
		largeMemory[memoryPointer] = 10;
		memoryPointer++;

		SDWriteIndex++;



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
	}

	else if ((whichString > 0) && (whichString < 3))
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

		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(didPlucked,tempText, 10);
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


	}
*/
	else
	{
		for (int i = 0; i < 10; i++)
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

		//filtered abs
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(tempAbsInt[whichString],tempText, 10);
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

		//smoothed1
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt[whichString],tempText, 10);
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

		//smoothed2
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(smoothedInt2[whichString],tempText, 10);
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

		//smoothed db
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(dbSmoothedInt[whichString],tempText, 10);
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

		//armed counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(armedCounter[whichString],tempText, 10);
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

		//down counter
		for (int i = 0; i < 10; i++)
		{
			tempText[i] = 0;
		}
		itoa(downCounter[whichString],tempText, 10);
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

		//string touched
		if (stringTouchRH[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 32;
		memoryPointer++;



		//string sounding
		if (stringSounding[whichString] > 0)
		{
			largeMemory[memoryPointer] = 49;

		}
		else
		{
			largeMemory[memoryPointer] = 48;
		}
		memoryPointer++;
		largeMemory[memoryPointer] = 59;
		memoryPointer++;
		largeMemory[memoryPointer] = 13;
		memoryPointer++;
		largeMemory[memoryPointer] = 10;
		memoryPointer++;
	}
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

		largeMemory[memoryPointer] = 59;
		memoryPointer++;
		largeMemory[memoryPointer] = 13;
		memoryPointer++;
		largeMemory[memoryPointer] = 10;
		memoryPointer++;

		SDWriteIndex++;
	}
	*/
}
#endif

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

	  //D3 Domain�SRAM
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


int totalMaximums[NUM_STRINGS];

int attackDetectPeak2 (int whichString, int tempInt)
{
	float output = -1;
	float tempSamp = (((float)tempInt - TWO_TO_15) * INV_TWO_TO_15);
	for (int k = 0; k < FILTER_ORDER; k++)
	{
		// a highpass filter, remove any slow moving signal (effectively centers the signal around zero and gets rid of the signal that isn't high frequency vibration) cutoff of 100Hz, // applied 8 times to get rid of a lot of low frequency bumbling around
		tempSamp = tHighpass_tick(&opticalHighpass[whichString][k], tempSamp);
		tempSamp = tVZFilter_tickEfficient(&opticalLowpass[whichString][k], tempSamp);
	}

	float tempAbs = fabsf(tempSamp);
	tempAbsInt[whichString] = (tempAbs * (TWO_TO_16 - 1));
	Dsmoothed = tSlide_tick(&fastSlide[whichString], tempAbs);
	smoothedInt[whichString] = (Dsmoothed * (TWO_TO_16 - 1));
	Dsmoothed2 = tSlide_tick(&slowSlide[whichString], Dsmoothed);

	Dsmoothed2 = LEAF_clip(0.0f, Dsmoothed2, 1.0f);
	smoothedInt2[whichString] = (Dsmoothed2 * (TWO_TO_16 - 1));
	//dbSmoothed2 = atodb(Dsmoothed2);
	dbSmoothed2 = LEAF_clip(-60.0f, atodbTable[(uint32_t)(Dsmoothed2 * ATODB_TABLE_SIZE_MINUS_ONE)], 12.0f);
	dbSmoothedInt[whichString] = dbSmoothed2 * 100.0f;
	//dbSmoothed2 = LEAF_clip(-50.f, dbSmoothed2, 12.0f);
	//get the slope
	float slope = (dbSmoothed2 - prevdbSmoothed2[whichString]);
	slopeStorage[whichString] = slope;
	float integerVersion = Dsmoothed2 * (TWO_TO_16 - 1);
	threshOut = tThreshold_tick(&threshold[whichString], integerVersion);
	if ((slope > 0.1f) && (threshOut > 0))
	{
		armed[whichString] = 1;
	}

	if (armed[whichString] == 1)
	{
		if (integerVersion > stringMaxes[whichString])
		{
			stringMaxes[whichString] = integerVersion;
		}
		armedCounter[whichString]++;

		if (slope <= 0.0f)
		{
			downCounter[whichString]++;
		}
		if (slope > 0.01f)
		{
			downCounter[whichString] = 0;
		}
		if (downCounter[whichString] > 128)
		{
			//found a peak?
			output = stringMaxes[whichString];
			output = LEAF_clip(0.0f, output, 65535.0f);
			armed[whichString] = 0;
			armedCounter[whichString] = 0;
			downCounter[whichString] = 0;
			stringMaxes[whichString] = 0;
		}
	}

	prevdbSmoothed2[whichString] = dbSmoothed2;
	return (int)output;
}

int stringStates[10];

void ADC_Frame(int offset)
{
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	int changeHappened = 0;
	//sampRecords[currentSamp] = frameCount;
	//currentSamp++;

	for (int i = offset; i < ADC_FRAME_SIZE + offset; i++)
	{
		//for (int j = 0; j < NUM_ADC_CHANNELS; j++)
		if (howManyFrames > 0)
		{
			howManyFrames--;
		}
		for (int j = 0; j < 10; j++)
		{
			int tempInt = ADC_values[(i*NUM_ADC_CHANNELS) + j];

			if (j < 8)
			{
				stringTouchRH[j] = (SPI_RX[4] >> j) & 1;
			}
			else
			{
				stringTouchRH[j] = (SPI_RX[5] >> (j-8)) & 1;
			}
			didPlucked[j] = attackDetectPeak2(j, tempInt);
			/*
			if (didPlucked2[j] < 0)
			{
				didPlucked2[j] = 0;
			}
			//didPlucked2[j] = tPluckDetectorInt_tick(&myPluck[j], tempInt);
			if (didPlucked[j] < 0)
			{
				didPlucked[j] = 0;
			}
			*/
			/*
			else if (didPlucked[j] > 0)
			{
				pluckDelay[j] = 1;
				pluckValues[j] = didPlucked[j];
			}

			int doIt = 0;
*/
			if (howManyFrames == 0)
			{
				/*
				if (pluckDelay[j] > 128)
				{
					doIt = 1;
					pluckDelay[j] = 0;
				}
				if (pluckDelay[j] > 0)
				{
					pluckDelay[j]++;
				}
*/
				if ((didPlucked[j] > 0) && (!stringSounding[j]) && (!stringTouchRH[j]))
				{
					if (didPlucked[j] > totalMaximums[j])
					{
						totalMaximums[j] = didPlucked[j];
					}
					didPlucked[j] = (float)didPlucked[j] * storedMaxFloats[j];
					if (didPlucked[j] > 65535)
					{
						didPlucked[j] = 65535;
					}
					stringStates[j] = didPlucked[j];
					SPI_PLUCK_TX[(j * 2) + 1] = (didPlucked[j] >> 8);
					SPI_PLUCK_TX[(j * 2) + 2] = (didPlucked[j] & 0xff);

					//a pluck happened! send a message over SPI to the other ICs
					//TODO: a pluck message
					//if (j == 1)
					//{

						//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
					//}
					//HAL_SPI_Transmit_DMA(&hspi1, SPI_PLUCK_TX, 20);
					/*
					SPI_PLUCK_TX[0] = j;
					SPI_PLUCK_TX[1] = (uint8_t) (didPlucked[j] >> 8); //low byte
					SPI_PLUCK_TX[2] = (uint8_t) (didPlucked[j] & 0xff); //low byte
					SPI_PLUCK_TX[3] = 0;
					*/
					changeHappened = 1;
					stringSounding[j] = 1;
					//HAL_SPI_Transmit_DMA(&hspi1, SPI_PLUCK_TX, 4);
				}

				if ((stringTouchRH[j]) && (stringSounding[j]))
				{
					//a note-off happened! send a message over SPI to the other ICs
					/*
					SPI_PLUCK_TX[0] = j;
					SPI_PLUCK_TX[1] = 0;
					SPI_PLUCK_TX[2] = 0;
					SPI_PLUCK_TX[3] = 0;
					*/
					SPI_PLUCK_TX[(j * 2) + 1] = 0;
					SPI_PLUCK_TX[(j * 2) + 2] = 0;
					//if (j == 1)
					//{
						//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

					//}
					//HAL_SPI_Transmit_DMA(&hspi1, SPI_PLUCK_TX, 20);
					stringStates[j] = 0;
					changeHappened = 1;
					stringSounding[j] = 0;
				}
				#if SDRECORD
				if (SDReady)
				{
					writeToSD(SDWriteIndex, tempInt, j, stringSounding[j]);

					if (memoryPointer >= ((LARGE_MEM_SIZE/2) - 3000))
					{
						finishSD = 1;
						HAL_ADC_Stop(&hadc1);
						//HAL_SAI_DMAStop(&hsai_BlockA1);
						//HAL_SAI_DMAStop(&hsai_BlockB1);
					}
				}
				#endif
/*
				if (memoryPointer < ((LARGE_MEM_SIZE / 2) - 10))
				{
					largeMemory[memoryPointer++] = (uint16_t)tempInt;
					largeMemory[memoryPointer++] = (uint16_t)didPlucked[j];
					largeMemory[memoryPointer++] = (uint16_t)(myPluck[j]->ready_for_pluck);
					largeMemory[memoryPointer++] = (uint16_t)(myPluck[j]->midpoint_estimate);
					largeMemory[memoryPointer++] = (uint16_t)didPlucked2[j];
					largeMemory[memoryPointer++] = (uint16_t)armed[j];
					largeMemory[memoryPointer++] = (uint16_t)downCounter[j];
					largeMemory[memoryPointer++] = (uint16_t)((slopeStorage[j] * 4096.0f) + 2048.0f);
					largeMemory[memoryPointer++] = (uint16_t)stringTouchRH[j];
					largeMemory[memoryPointer++] = (uint16_t)stringSounding[j];
				}
				else
				{
					writeSD();

					while(1)
					{
						//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
					}
				}

*/
			}
			//itoa(SDWriteIndex, wtext, 4);



		}

	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	if (changeHappened)
	{
		HAL_SPI_Transmit_DMA(&hspi1, SPI_PLUCK_TX, 22);
	}
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

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
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
