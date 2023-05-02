/**
  ******************************************************************************
  * @file           : ssd1306.c
  * @brief          : SSD1306 driver
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */



/* Includes ------------------------------------------------------------------*/

#include "ssd1306.h"
#include "ssd1306_interface.h"

/* Declarations and definitions ----------------------------------------------*/

SSD1306_State SSD1306_state = SSD1306_READY;

/* Functions -----------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
void SSD1306_Init ()
{   
  uint8_t data[3];

  HAL_Delay(100);
  data[0] = 0xAE;
  SendCommand(data, 1);
  HAL_Delay(100);

  data[0] = 0xD5;
  data[1] = 0x80;
  SendCommand (data, 2);
  data[0] = 0xA8;
  data[1] = 0x27;
  SendCommand (data, 2);
  data[0] = 0xD3;
  data[1] = 0;
  SendCommand (data, 2);
  data[0] = 0x40;
  SendCommand (data, 1);
  data[0] = 0x8D;
  data[1] = 0x14;
  SendCommand (data, 2);
  data[0] = 0xA1;
  SendCommand (data, 1);
  data[0] = 0xC8;
  SendCommand (data, 1);
  data[0] = 0xDA;
  data[1] = 0b00010010;
  SendCommand (data, 2);
  data[0] = 0x81;
  data[1] = 0x00;
  SendCommand (data, 2);
  data[0] = 0xD9;
  data[1] = 0x22;
  SendCommand (data, 2);
  data[0] = 0xDB;
  data[1] = 0x20;
  SendCommand (data, 2);
  data[0] = 0x20;
  data[1] = 0x00;
  SendCommand (data, 2);
  data[0] = 0x21;
  data[1] = 0;
  data[2] = 71;
  SendCommand (data, 3);
  data[0] = 0x22;
  data[1] = 0;
  data[2] = 4;
  SendCommand (data, 3);
  data[0] = 0xA4;
  SendCommand (data, 1);
  HAL_Delay(100);
  data[0] = 0xAF;
  SendCommand (data, 1);
  HAL_Delay(100);

  /*

  // Set MUX ratio
  data[0] = 0xA8;
  data[1] = 31;
  SendCommand (data, 2);

  // Set display offset
  data[0] = 0xD3;
  data[1] = 0;
  SendCommand (data, 2);

  // Set display start line
  data[0] = 0x40;
  SendCommand (data, 1);
  
  // Set segment remap
  data[0] = 0xA0;
  SendCommand (data, 1);
  
  // Set COM output scan direction
  data[0] = 0xC0;
  SendCommand (data, 1);
  
  // Set COM pins hardware configuration
  data[0] = 0xDA;
  data[1] = 0x02;
  SendCommand (data, 2);

  // Set contrast
  data[0] = 0x81;
  data[1] = 0x2F;
  SendCommand (data, 2);

  // Entire display on
  data[0] = 0xA4;
  SendCommand(data, 1);

  //Set normal display
  data[0] = 0xA6;
  SendCommand (data, 1);

  // Set oscillator frequency
  data[0] = 0xD5;
  data[1] = 0x80;
  SendCommand (data, 2);

  // Enable charge pump regulator
  data[0] = 0x8D;
  data[1] = 0x14;
  SendCommand (data, 2);
  
  // Set horizontal addressing mode
  data[0] = 0x20;
  data[1] = 0x00;
  SendCommand (data, 2);
  
 //  Set column address
  data[0] = 0x21;
  data[1] = 0;
  data[2] = 71;
  SendCommand (data, 3);
  
  // Set page address
  data[0] = 0x22;
  data[1] = 0;
  data[2] = 3;
  SendCommand (data, 3);
  HAL_Delay(100);
  // Set display on
  data[0] = 0xAF;
  SendCommand (data, 1);
  HAL_Delay(100);


  //data[0] = 0xA5;
  //SendCommand (data, 1);

*/

}


/*----------------------------------------------------------------------------*/
void  SSD1306_ClearBlock (uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd)
{
 static uint8_t mPusto[8]  = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

  for (uint16_t x = xStart; x < xEnd; x += 8)
  {
   for (uint16_t y = yStart; y < yEnd + 1; y++)
   {
    SSD1306_SendBlock (mPusto, 8, x, x + 7, y, y);
   }
  }
}

/*----------------------------------------------------------------------------*/
void SSD1306_SendBlock (uint8_t* data, uint8_t size, uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd)
{
 uint8_t datatmp[3];
 datatmp[0] = 0x21;
 datatmp[1] = 28 + xStart;
 datatmp[2] = 28 + xEnd;
 SendCommand (datatmp, 3);
 datatmp[0] = 0x22;
 datatmp[1] = yStart;
 datatmp[2] = yEnd;
 SendCommand (datatmp, 3);
 SendData (data, size);
 while (SSD1306_IsReady () == 0);
}

/*----------------------------------------------------------------------------*/
inline uint8_t SSD1306_IsReady()
{
 return ((SSD1306_state == SSD1306_BUSY) ? 0 : 1);
}



/*----------------------------------------------------------------------------*/
