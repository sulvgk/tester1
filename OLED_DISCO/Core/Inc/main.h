/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
 enum enumDiapazon_R
 {
  eDiapazon_R_0, eDiapazon_R_1, eDiapazon_R_2, eDiapazon_R_3
 };
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void f_Dobavit_Tochku_k_Cifre (uint8_t* pcifra);
void f_Chislo_v_Cifry (float fchislo);
void f_4_Cifry_Chisla (float fchislo);
void f_strelka_vlevo_OK_strelka_vpravo_na_ekran (uint8_t* pstrelka_vlevo, uint8_t* pOK, uint8_t* pstrelka_vpravo);
void f_cikl_na_ekran (uint8_t* pbuf);
void f_ClearBlock (uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI0_1_IRQn
#define R_Const_1_Pin GPIO_PIN_2
#define R_Const_1_GPIO_Port GPIOA
#define R_Const_2_Pin GPIO_PIN_3
#define R_Const_2_GPIO_Port GPIOA
#define R_Const_3_Pin GPIO_PIN_4
#define R_Const_3_GPIO_Port GPIOF
#define Proverka_LED_Pin GPIO_PIN_5
#define Proverka_LED_GPIO_Port GPIOF
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
