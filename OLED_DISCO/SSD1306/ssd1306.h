/**
  ******************************************************************************
  * @file           : ssd1306.h
  * @brief          : SSD1306 driver header
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */

#ifndef SSD1306_H
#define SSD1306_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/

#define SSD1306_X_SIZE                                                  72
#define SSD1306_Y_SIZE                                                  40

/* Functions -----------------------------------------------------------------*/

extern void SSD1306_Init ();
extern void SSD1306_ClearBlock (uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd);
extern void SSD1306_SendBlock (uint8_t* data, uint8_t size, uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd);
extern uint8_t SSD1306_IsReady ();

#endif // #ifndef SSD1306_H
