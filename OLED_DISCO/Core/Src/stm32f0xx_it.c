/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

enum enumCikl
{
 e220, eProverka_LED, e75, e22, eR, eV
} eCikl = e220;

enum enumDiapazon_R eDiapazon_R = eDiapazon_R_1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern uint8_t mstrelka_vlevo_passiv[32];
extern uint8_t mstrelka_vlevo_activ[32];
extern uint8_t mOK_passiv[48];
extern uint8_t mOK_activ[48];
extern uint8_t mstrelka_vpravo_passiv[32];
extern uint8_t mstrelka_vpravo_activ[32];

extern uint8_t m220[32];
extern uint8_t mL_Proverka_LED[32];
extern uint8_t m75[32];
extern uint8_t m22[32];
extern uint8_t mV_Napriagenie[32];
extern uint8_t mR_Soprotivlenie[32];

extern uint32_t uiADC;
extern const float R_Const_1;
extern const float R_Const_2;
extern const float R_Const_3;
extern float R0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
  NVIC_DisableIRQ (EXTI0_1_IRQn);
  HAL_TIM_Base_Start_IT (&htim3);
  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  HAL_TIM_Base_Stop_IT (&htim3);
  NVIC_EnableIRQ (EXTI0_1_IRQn);
  if (HAL_GPIO_ReadPin (B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
  {
   switch ((int) eCikl)
   {
    case e220:
     f_LED ();
     break;
    case eProverka_LED:
     f_75 ();
     break;
    case e75:
     f_22 ();
     break;
    case e22:
     f_R ();
     break;
    case eR:
     f_220 ();
     break;
   }
  }
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles I2C1 global interrupt.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//****************************************************
void fProverka_LED_Init (void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin (Proverka_LED_GPIO_Port, Proverka_LED_Pin, GPIO_PIN_SET);

 /*Configure Proverka_LED_Pin */
 GPIO_InitStruct.Pin = Proverka_LED_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init (Proverka_LED_GPIO_Port, &GPIO_InitStruct);

}
//****************************************************************
void fProverka_LED_Deinit (void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 /*Configure Proverka_LED_Pin */
 GPIO_InitStruct.Pin = Proverka_LED_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init (Proverka_LED_GPIO_Port, &GPIO_InitStruct);
}
//****************************************************************
void f_220 (void)
{
 f_cikl_na_ekran (m220);
 eCikl = e220;
}
//****************************************************************
void f_LED (void)
{
 HAL_ADC_Stop_IT (&hadc);
 fProverka_LED_Init ();

 f_cikl_na_ekran (mL_Proverka_LED);
 f_ClearBlock (0, 55, 0, 1);
 f_ClearBlock (0, 71, 2, 4);
 eCikl = eProverka_LED;
}
//****************************************************************
void f_75 (void)
{
 fProverka_LED_Deinit ();
 HAL_ADC_Start_IT (&hadc);

 f_cikl_na_ekran (m75);
  eCikl = e75;
}
//****************************************************************
void f_22 (void)
{
 f_cikl_na_ekran (m22);
  eCikl = e22;
}
//****************************************************************
void f_R (void)
{
 f_ClearBlock (0, 55, 0, 1);
 f_cikl_na_ekran (mR_Soprotivlenie);
 eCikl = eR;
}
//****************************************************************
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc_tmp)
{
 const uint32_t c220_MIN = (4095.0 * 200.0 / (200.0 + R_Const_1));   // 200 om
 const uint32_t c220_MAX = (4095.0 * 240.0 / (240.0 + R_Const_1));   // 240 om
 const uint32_t c75_MIN =  (4095.0 * 70.0 / (70.0 + R_Const_1));     //  70 om
 const uint32_t c75_MAX =  (4095.0 * 80.0 / (80.0 + R_Const_1));     //  80 om
 const uint32_t c22_MIN =  (4095.0 * 20.0 / (20.0 + R_Const_1));     //  20 om
 const uint32_t c22_MAX =  (4095.0 * 25.0 / (25.0 + R_Const_1));     //  25 om

 enum enumMenshe_OK_Bolshe
 {
  ePusto, eMenshe, eOK, eBolshe
 } eMenshe_OK_Bolshe = eOK;

 uiADC = HAL_ADC_GetValue (hadc_tmp);

 switch ((int) eCikl)
 {
  case e220:
   if (uiADC < c220_MIN) eMenshe_OK_Bolshe = eMenshe;
   if (uiADC > c220_MAX) eMenshe_OK_Bolshe = eBolshe;
   break;
  case e75:
   if (uiADC < c75_MIN) eMenshe_OK_Bolshe = eMenshe;
   if (uiADC > c75_MAX) eMenshe_OK_Bolshe = eBolshe;
   break;
  case e22:
   if (uiADC < c22_MIN) eMenshe_OK_Bolshe = eMenshe;
   if (uiADC > c22_MAX) eMenshe_OK_Bolshe = eBolshe;
   break;
  case eR:
   eMenshe_OK_Bolshe = ePusto;
   /*if (uiADC > 3500 && eDiapazon_R == eDiapazon_R_1)
   {
    fDiapazon_R (eDiapazon_R = eDiapazon_R_2);
    return;
   }
   if (uiADC > 3500 && eDiapazon_R == eDiapazon_R_2)
   {
    fDiapazon_R (eDiapazon_R = eDiapazon_R_3);
    return;
   }
   if (uiADC < 500 && eDiapazon_R == eDiapazon_R_2)
   {
    fDiapazon_R (eDiapazon_R = eDiapazon_R_1);
    return;
   }
   if (uiADC < 500 && eDiapazon_R == eDiapazon_R_3)
   {
    fDiapazon_R (eDiapazon_R = eDiapazon_R_2);
    return;
   }*/
   break;
 }

 //float fRx = R0 * uiADC / (4095.0 - uiADC);
 float fRx = R0 * uiADC / (4030.0 - uiADC);
 f_Chislo_v_Cifry (fRx);

 switch ((int) eMenshe_OK_Bolshe)
 {
  case ePusto:
   f_ClearBlock (0, 55, 0, 1);
   break;
  case eMenshe:
   f_strelka_vlevo_OK_strelka_vpravo_na_ekran (mstrelka_vlevo_activ, mOK_passiv, mstrelka_vpravo_passiv);
   break;
  case eOK:
   f_strelka_vlevo_OK_strelka_vpravo_na_ekran (mstrelka_vlevo_passiv, mOK_activ, mstrelka_vpravo_passiv);
   break;
  case eBolshe:
   f_strelka_vlevo_OK_strelka_vpravo_na_ekran (mstrelka_vlevo_passiv, mOK_passiv, mstrelka_vpravo_activ);
   break;
 }
}
//****************************************************
void fDiapazon_R (int edipazon_r)
{
 static GPIO_InitTypeDef GPIO_InitStruct_1 = {0};
 static GPIO_InitTypeDef GPIO_InitStruct_2 = {0};
 static GPIO_InitTypeDef GPIO_InitStruct_3 = {0};

 GPIO_InitStruct_1.Pin = R_Const_1_Pin;
 GPIO_InitStruct_1.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct_1.Pull = GPIO_NOPULL;

 GPIO_InitStruct_2.Pin = R_Const_2_Pin;
 GPIO_InitStruct_2.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct_2.Pull = GPIO_NOPULL;

 GPIO_InitStruct_3.Pin = R_Const_3_Pin;
 GPIO_InitStruct_3.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct_3.Pull = GPIO_NOPULL;

 switch (edipazon_r)
 {
   case eDiapazon_R_1:
    R0 = R_Const_1;
    HAL_GPIO_WritePin (R_Const_1_GPIO_Port, R_Const_1_Pin, GPIO_PIN_SET);
    GPIO_InitStruct_1.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_1.Speed = GPIO_SPEED_FREQ_LOW;
    break;
   case eDiapazon_R_2:
    //R0 = R_Const_1 + R_Const_2;
    R0 = R_Const_2;
    HAL_GPIO_WritePin (R_Const_2_GPIO_Port, R_Const_2_Pin, GPIO_PIN_SET);
    GPIO_InitStruct_2.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_2.Speed = GPIO_SPEED_FREQ_LOW;
    break;
   case eDiapazon_R_3:
    R0 = R_Const_1 + R_Const_2 + R_Const_3;
    HAL_GPIO_WritePin (R_Const_3_GPIO_Port, R_Const_3_Pin, GPIO_PIN_SET);
    GPIO_InitStruct_3.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_3.Speed = GPIO_SPEED_FREQ_LOW;
    break;
  }
 HAL_GPIO_Init (R_Const_1_GPIO_Port, &GPIO_InitStruct_1);
 HAL_GPIO_Init (R_Const_2_GPIO_Port, &GPIO_InitStruct_2);
 HAL_GPIO_Init (R_Const_3_GPIO_Port, &GPIO_InitStruct_3);
}
//****************************************************************
/* USER CODE END 1 */
