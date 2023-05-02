/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
const float R_Const_1 = 180.0;    // R1 = 180.0 om
const float R_Const_2 = 4700.0;   // R2 = 4.7 кom
const float R_Const_3 = 100000.0; // R3 = 100.0 кom
float R0 = R_Const_2;
uint32_t uiADC;

uint8_t mstrelka_vlevo_passiv[32]  = {0x0, 0x0, 0x0, 0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x8, 0x8, 0x4, 0x4, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x4, 0x4, 0x8, 0x8, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x0};
uint8_t mstrelka_vlevo_activ[32]   = {0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0xBF, 0xBF, 0xDF, 0xDF, 0xEF, 0xEF, 0xF7, 0xF7, 0xFB, 0xFB, 0xFF, 0xFF, 0xFE, 0xFE, 0xFD, 0xFD, 0xFB, 0xFB, 0xF7, 0xF7, 0xEF, 0xEF, 0xDF, 0xDF, 0xBF, 0xBF, 0xFF};
uint8_t mOK_passiv[48]             = {0x0, 0x0, 0x0, 0xF8, 0xFC, 0x4, 0x4, 0x4, 0x4, 0xFC, 0xF8, 0x0, 0x0, 0x0, 0xFC, 0xFC, 0xC0, 0x60, 0x30, 0x18, 0xC, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1F, 0x3F, 0x20, 0x20, 0x20, 0x20, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x3F, 0x3F, 0x1, 0x3, 0xE, 0x18, 0x30, 0x20, 0x0, 0x0};
uint8_t mOK_activ[48]              = {0xFF, 0xFF, 0x7, 0x3, 0x3, 0xF3, 0xF3, 0x3, 0x3, 0x7, 0xFF, 0xFF, 0xFF, 0x3, 0x3, 0x3, 0x3F, 0x1F, 0x8F, 0xC3, 0xE3, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xC0, 0xC0, 0xCF, 0xCF, 0xC0, 0xC0, 0xE0, 0xFF, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xFE, 0xFC, 0xF0, 0xC1, 0xC7, 0xCF, 0xFF, 0xFF};
uint8_t mstrelka_vpravo_passiv[32] = {0x0, 0x4, 0x4, 0x8, 0x8, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x8, 0x8, 0x4, 0x4, 0x2, 0x2, 0x1, 0x1, 0x0};
uint8_t mstrelka_vpravo_activ[32]  = {0xFF, 0xFB, 0xFB, 0xF7, 0xF7, 0xEF, 0xEF, 0xDF, 0xDF, 0xBF, 0xBF, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xBF, 0xDF, 0xDF, 0xEF, 0xEF, 0xF7, 0xF7, 0xFB, 0xFB, 0xFD, 0xFD, 0xFE, 0xFE, 0xFF};

uint8_t m220[32]                   = {0x0, 0x00, 0xC0, 0x20, 0x20, 0xC0, 0x00, 0xC0, 0x20, 0x20, 0xC0, 0x00, 0xC0, 0x20, 0x20, 0xC0, 0x0, 0x00, 0x38, 0x24, 0x22, 0x31, 0x00, 0x38, 0x24, 0x22, 0x31, 0x00, 0x1F, 0x20, 0x20, 0x1F};
uint8_t mL_Proverka_LED[32]        = {0x00, 0x0, 0xFC, 0xFC, 0xFC, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x0, 0x7F, 0x7F, 0x7F, 0x60, 0x60, 0x60, 0x60, 0x60, 0x78, 0x78, 0x78, 0x0, 0x00, 0x00};
uint8_t m75[32]                    = {0x0, 0x00, 0x20, 0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x38, 0x04, 0x02, 0x01, 0x00, 0x11, 0x21, 0x21, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t m22[32]                    = {0x0, 0x00, 0xC0, 0x20, 0x20, 0xC0, 0x00, 0xC0, 0x20, 0x20, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x38, 0x24, 0x22, 0x31, 0x00, 0x38, 0x24, 0x22, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mV_Napriagenie[32]         = {0x00, 0x1C, 0x7C, 0xF0, 0xC0, 0x0, 0x0, 0x00, 0x0, 0x0, 0x0, 0xC0, 0xF0, 0x7C, 0x1C, 0x00, 0x00, 0x0, 0x00, 0x1, 0x7, 0xF, 0x1C, 0x78, 0x78, 0x1C, 0xF, 0x7, 0x1, 0x0, 0x00, 0x00};
uint8_t mR_Soprotivlenie[32]       = {0x00, 0x0, 0xFC, 0xFC, 0xFC, 0xC, 0xC, 0xC, 0xC, 0xC, 0xF8, 0xF8, 0xF0, 0x0, 0x0, 0x00, 0x00, 0x0, 0x7F, 0x7F, 0x7F, 0x7, 0xF, 0x1F, 0x3B, 0x73, 0x61, 0x61, 0x40, 0x0, 0x00, 0x00};

uint8_t m1[48]                     = {0x00, 0x00, 0x0, 0x80, 0xC0, 0xE0, 0xF8, 0xF8, 0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1, 0x1, 0x1, 0x1, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0x0};
uint8_t m2[48]                     = {0x00, 0x00, 0xC0, 0xE0, 0x70, 0x38, 0x38, 0x38, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1, 0x1, 0x0, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1F, 0xF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xE7, 0xE3, 0xE1, 0xE0, 0xE0, 0xE0, 0xFC, 0xFC, 0x0, 0x0, 0x0, 0x0};
uint8_t m3[48]                     = {0x00, 0x00, 0xC0, 0xE0, 0x70, 0x38, 0x38, 0x38, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1, 0x1, 0x0, 0x0, 0x20, 0x70, 0x70, 0xF8, 0xDF, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x1C, 0x3C, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x0};
uint8_t m4[48]                     = {0x00, 0x00, 0x0, 0x0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0x70, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0x8E, 0x87, 0x83, 0x81, 0x80, 0x80, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0};
uint8_t m5[48]                     = {0x00, 0x00, 0xF8, 0xF8, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF, 0xF, 0xE, 0xE, 0xE, 0xE, 0xE, 0x1C, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x1C, 0x3C, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x0};
uint8_t m6[48]                     = {0x00, 0x00, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x18, 0x18, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x73, 0x71, 0x70, 0x70, 0x70, 0xE0, 0xC0, 0x80, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, 0x1F, 0x3F, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x0};
uint8_t m7[48]                     = {0x00, 0x00, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1F, 0xF, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t m8[48]                     = {0x00, 0x00, 0xC0, 0xE0, 0x70, 0x38, 0x38, 0x38, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0xDF, 0xF8, 0x70, 0x70, 0x70, 0x70, 0xF8, 0xDF, 0x8F, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, 0x1F, 0x3F, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x0};
uint8_t m9[48]                     = {0x00, 0x00, 0xC0, 0xE0, 0x70, 0x38, 0x38, 0x38, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF, 0x1F, 0x38, 0x70, 0x70, 0x70, 0x70, 0x70, 0xFF, 0xFF, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, 0x0, 0xC0, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0xE, 0x7, 0x3, 0x0, 0x0, 0x0, 0x0};
uint8_t m0[48]                     = {0x00, 0x00, 0xC0, 0xE0, 0x70, 0x38, 0x38, 0x38, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, 0x1F, 0x3F, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x3F, 0x1F, 0x0, 0x0, 0x0, 0x0};

uint8_t mMegaOmy[48]               = {0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xE0, 0x80, 0x0, 0x0, 0x0, 0x80, 0xE0, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x3, 0xF, 0x3E, 0xF8, 0x3E, 0xF, 0x3, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x0, 0xFF, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0xFF, 0x0};
uint8_t mKiloOmy[24]               = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x80, 0xC0, 0xF0, 0x3C, 0xC, 0x00, 0xFF, 0xFF, 0x3, 0xF, 0x3C, 0xF0, 0xC0};
uint8_t mOmy[24]                   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mBolshe[48]                = {0x00, 0x00, 0x18, 0x38, 0x78, 0xF0, 0xE0, 0xC0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3, 0x7, 0x8F, 0xDE, 0xFC, 0xF8, 0x70, 0x20, 0x00, 0x00, 0x0, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0xF, 0x7, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0};

uint8_t* mCifra[10]                = {m0, m1, m2, m3, m4, m5, m6, m7, m8, m9};

uint8_t cifra_s_tochkoj[48];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END 1 */

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Stop_IT (&htim3);
  NVIC_EnableIRQ (EXTI0_1_IRQn);
  fDiapazon_R (eDiapazon_R_2);
  SSD1306_Init ();
  while (SSD1306_IsReady () == 0);
  f_cikl_na_ekran (m220);
  HAL_ADC_Start_IT (&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Const_1_Pin R_Const_2_Pin PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = R_Const_1_Pin|R_Const_2_Pin|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Const_3_Pin Proverka_LED_Pin PF6 PF7 */
  GPIO_InitStruct.Pin = R_Const_3_Pin|Proverka_LED_Pin|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*----------------------------------------------------------------------------*/
void f_Dobavit_Tochku_k_Cifre (uint8_t* pcifra)
{
 for (uint8_t j = 0; j < 48; j++)
 {
  cifra_s_tochkoj[j] = pcifra[j];
 }

 cifra_s_tochkoj[46] = cifra_s_tochkoj[46] + 224;
 cifra_s_tochkoj[47] = cifra_s_tochkoj[47] + 224;
}
/*----------------------------------------------------------------------------*/
void f_Chislo_v_Cifry (float fchislo)
{
 if (fchislo >= 1000000.0)
 {
  SSD1306_ClearBlock (0, 15, 2, 4);
  SSD1306_SendBlock (mBolshe, 48, 16, 31, 2, 4);
  f_Dobavit_Tochku_k_Cifre (mCifra[1]);
  SSD1306_SendBlock (cifra_s_tochkoj, 48, 32, 47, 2, 4);
  SSD1306_SendBlock (mMegaOmy, 48, 48, 63, 2, 4);
  SSD1306_SendBlock (mOmy, 24, 64, 71, 2, 4);
  return;
 }
 if (fchislo >= 1000.0)
 {
  f_4_Cifry_Chisla (fchislo);
  SSD1306_SendBlock (mKiloOmy, 24, 64, 71, 2, 4);
  return;
 }
 f_4_Cifry_Chisla (fchislo * 1000.0);
 SSD1306_SendBlock (mOmy, 24, 64, 71, 2, 4);
}
/*----------------------------------------------------------------------------*/
void f_4_Cifry_Chisla (float fchislo)
{
 uint8_t cifra;
 cifra = fchislo / 100000; // цифра сотен
 if (cifra == 0)
  SSD1306_ClearBlock (0, 15, 2, 4);
 else
  SSD1306_SendBlock (mCifra[cifra], 48, 0, 15, 2, 4);
 fchislo -= 100000 * cifra;

 cifra = fchislo / 10000;  // цифра десяток
 if (cifra == 0)
  SSD1306_ClearBlock (16, 31, 2, 4);
 else
  SSD1306_SendBlock (mCifra[cifra], 48, 16, 31, 2, 4);
 fchislo -= 10000 * cifra;

 cifra = fchislo / 1000;  // цифра единиц
 f_Dobavit_Tochku_k_Cifre (mCifra[cifra]);
 SSD1306_SendBlock (cifra_s_tochkoj, 48, 32, 47, 2, 4);
 fchislo -= 1000 * cifra;

 cifra = fchislo / 100;  // цифра дробной части
 SSD1306_SendBlock (mCifra[cifra], 48, 48, 63, 2, 4);
}
/*----------------------------------------------------------------------------*/
void f_strelka_vlevo_OK_strelka_vpravo_na_ekran (uint8_t* pstrelka_vlevo, uint8_t* pOK, uint8_t* pstrelka_vpravo)
{
 SSD1306_SendBlock (pstrelka_vlevo, 32, 0, 15, 0, 1);
 SSD1306_SendBlock (pOK, 48, 16, 39, 0, 1);
 SSD1306_SendBlock (pstrelka_vpravo, 32, 40, 55, 0, 1);
}
/*----------------------------------------------------------------------------*/
inline void f_cikl_na_ekran (uint8_t* pbuf)
{
 SSD1306_SendBlock (pbuf, 32, 56, 71, 0, 1);
}

inline void f_ClearBlock (uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd)
{
 SSD1306_ClearBlock (xStart, xEnd, yStart, yEnd);
}
/*----------------------------------------------------------------------------*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
