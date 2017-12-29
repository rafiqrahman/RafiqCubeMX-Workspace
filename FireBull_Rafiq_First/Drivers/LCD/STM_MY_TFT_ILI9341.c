/*
		Library: 			TFT 2.4 inch LCD - ILI9341
		Written by:  	Mohamed Yaqoob
		Date:				 	16/08/2016
		Description:	This is a library for the 2.4 inches TFT LCD arduino shield with the
									ILI9341 driver. It perform basic displaying operations such as printing
									colored text and numbers. In addition to some more advanced oprations
									including drawing geometrical objects like line, rectange, circle,
									triangle and more...
		References**:
									This was written by referring mainly to the datasheet of the ILI9341
									LCD driver and by re-using some open source Arduino libraries as the
									following:
									(1): The ILI9341 LCD driver datasheet: 
											 https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf
									
									(2): SPFD5408 Adafruit Arduino library:
											 https://github.com/JoaoLopesF/SPFD5408
*/

//https://www.youtube.com/watch?v=30ouQjr0nh8&t=230s


//(1): Header files
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "STM_MY_TFT_ILI9341.h"

//(2): varaible declarations
static uint8_t iData[5];
static uint8_t rotationNum=1;
static bool _cp437    = false;

//(3): Private functions definition
//***************************************************************************
void TFT_LCD_WR(bool state)
{
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, (GPIO_PinState)state);
}
//***************************************************************************
static void TFT_LCD_RD(bool state)
{
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, (GPIO_PinState)state);
}
//***************************************************************************
static void TFT_LCD_RS(bool state)
{
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, (GPIO_PinState)state);
}
//***************************************************************************
static void TFT_LCD_CS(bool state)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, (GPIO_PinState)state);
}
//***************************************************************************
static void TFT_LCD_RST(bool state)
{
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, (GPIO_PinState)state);
}
//***************************************************************************
static void GPIO_read(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin 
                          |LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//***************************************************************************
static void GPIO_write(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin 
                          |LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//***************************************************************************
static void writeRegister8(uint8_t Reg, uint8_t data)
{
	TFT_LCD_RS(false);
	write8(Reg);
	TFT_LCD_RS(true);
	write8(data);
}
//***************************************************************************
static void write8(uint8_t data)
{
	TFT_LCD_CS(false);
	TFT_LCD_RD(true);
	PORT_D0_3->ODR &= ~(LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin);
	PORT_D4_7->ODR &= ~(LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin);
	
	TFT_LCD_WR(false);
	HAL_GPIO_WritePin(PORT_D0_3, LCD_D0_Pin, (GPIO_PinState)((data>>0) & 1UL));
	HAL_GPIO_WritePin(PORT_D0_3, LCD_D1_Pin, (GPIO_PinState)((data>>1) & 1UL));
	HAL_GPIO_WritePin(PORT_D0_3, LCD_D2_Pin, (GPIO_PinState)((data>>2) & 1UL));
	HAL_GPIO_WritePin(PORT_D0_3, LCD_D3_Pin, (GPIO_PinState)((data>>3) & 1UL));
	
	HAL_GPIO_WritePin(PORT_D4_7, LCD_D4_Pin, (GPIO_PinState)((data>>4) & 1UL));
	HAL_GPIO_WritePin(PORT_D4_7, LCD_D5_Pin, (GPIO_PinState)((data>>5) & 1UL));
	HAL_GPIO_WritePin(PORT_D4_7, LCD_D6_Pin, (GPIO_PinState)((data>>6) & 1UL));
	HAL_GPIO_WritePin(PORT_D4_7, LCD_D7_Pin, (GPIO_PinState)((data>>7) & 1UL));
	
	TFT_LCD_WR(true);
	TFT_LCD_CS(true);
}	
//***************************************************************************
static void write8_fast(uint8_t data[2], uint32_t size)
{
	TFT_LCD_CS(false);
	TFT_LCD_RD(true);
	PORT_D0_3->ODR &= ~(LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin);
	PORT_D4_7->ODR &= ~(LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin);
	for(uint32_t k=0; k<size;k++)
	{
		TFT_LCD_WR(false);
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D0_Pin, (GPIO_PinState)((data[0]>>0) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D1_Pin, (GPIO_PinState)((data[0]>>1) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D2_Pin, (GPIO_PinState)((data[0]>>2) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D3_Pin, (GPIO_PinState)((data[0]>>3) & 1UL));
		
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D4_Pin, (GPIO_PinState)((data[0]>>4) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D5_Pin, (GPIO_PinState)((data[0]>>5) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D6_Pin, (GPIO_PinState)((data[0]>>6) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D7_Pin, (GPIO_PinState)((data[0]>>7) & 1UL));
		TFT_LCD_WR(true);

		TFT_LCD_WR(false);
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D0_Pin, (GPIO_PinState)((data[1]>>0) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D1_Pin, (GPIO_PinState)((data[1]>>1) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D2_Pin, (GPIO_PinState)((data[1]>>2) & 1UL));
		HAL_GPIO_WritePin(PORT_D0_3, LCD_D3_Pin, (GPIO_PinState)((data[1]>>3) & 1UL));
		
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D4_Pin, (GPIO_PinState)((data[1]>>4) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D5_Pin, (GPIO_PinState)((data[1]>>5) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D6_Pin, (GPIO_PinState)((data[1]>>6) & 1UL));
		HAL_GPIO_WritePin(PORT_D4_7, LCD_D7_Pin, (GPIO_PinState)((data[1]>>7) & 1UL));
		TFT_LCD_WR(true);
	}
	TFT_LCD_CS(true);
}
//***************************************************************************
static void writeRegister16(uint8_t Reg, uint8_t data[2])
{
	TFT_LCD_RS(false);
	write8(Reg);
	TFT_LCD_RS(true);
	write8(data[0]);
	write8(data[1]);
}
//***************************************************************************
static void writeRegister32(uint8_t Reg, uint32_t data)
{
	//uint8_t data_8;
	TFT_LCD_RS(false);
	write8(Reg);
	TFT_LCD_RS(true);
	//data_8 = 0xFF&(data>>24);
	write8(0xFF&(data>>24));
	write8(0xFF&(data>>16));
	write8(0xFF&(data>>8));
	write8(0xFF&(data));
}
//***************************************************************************
static void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	TFT_LCD_CS(false);
	uint32_t data_32;
	data_32 = (x1<<16)|(x2);
	writeRegister32(ILI9341_COLADDRSET, data_32);
	data_32 = (y1<<16)|(y2);
	writeRegister32(ILI9341_PAGEADDRSET, data_32);
	TFT_LCD_CS(true);
}
//***************************************************************************
static void readRegister8(uint8_t Reg, uint8_t size, uint8_t *dataRet)
{
	uint8_t data_sub;
	TFT_LCD_RS(false);
	write8(Reg);
	TFT_LCD_CS(false);
	GPIO_read();
	TFT_LCD_RS(true);
	TFT_LCD_WR(true);
	for(uint8_t j=0; j<=size;j++)
	{
		TFT_LCD_RD(false);
		for(uint32_t i; i<100;i++);
		data_sub  = ((1UL&HAL_GPIO_ReadPin(PORT_D0_3, LCD_D0_Pin))<<0);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D0_3, LCD_D1_Pin))<<1);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D0_3, LCD_D2_Pin))<<2);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D0_3, LCD_D3_Pin))<<3);
		
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D4_7, LCD_D4_Pin))<<4);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D4_7, LCD_D5_Pin))<<5);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D4_7, LCD_D6_Pin))<<6);
		data_sub |= ((1UL&HAL_GPIO_ReadPin(PORT_D4_7, LCD_D7_Pin))<<7);
		TFT_LCD_RD(true);
		dataRet[j] = data_sub;
	}
	TFT_LCD_CS(true);
	GPIO_write();
}
//***************************************************************************
//***************************************************************************
//(4): Public functions definition
void ILI9341_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
  /*Configure GPIO pin Output Level */
	//(1): Reset all pins
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(PORT_D0_3, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_D4_7, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_WR_Pin LCD_RS_Pin LCD_CS_Pin LCD_RST_Pin 
                           LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin */
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PORT_D0_3, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_WR_Pin;
	HAL_GPIO_Init(LCD_WR_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_RS_Pin;
	HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_CS_Pin;
	HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_RST_Pin;
	HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_RD_Pin;
	HAL_GPIO_Init(LCD_RD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin 
                           LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  HAL_GPIO_Init(PORT_D4_7, &GPIO_InitStruct);
	
	//(1): Begin function
	HAL_Delay(20);
	TFT_LCD_RST(true);
	readRegister8(0x09, 4, iData); 
	writeRegister8(ILI9341_SLEEPOUT, 0);
	HAL_Delay(500);
	readRegister8(0x09, 4, iData);
	writeRegister8(ILI9341_SOFTRESET, 0);
	HAL_Delay(50);
	writeRegister8(ILI9341_DISPLAYOFF, 0);
	writeRegister8(ILI9341_POWERCONTROL1, 0x23);
	writeRegister8(ILI9341_POWERCONTROL2, 0x10);
	iData[0] = 0x2b; iData[1] = 0x2b; 
	writeRegister16(ILI9341_VCOMCONTROL1, iData);
	writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
  writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);  //ILI9341_MADCTL_MY |   ILI9341_MADCTL_BGR
  writeRegister8(ILI9341_PIXELFORMAT, 0x55);
	iData[0] = 0x1b; iData[1] = 0x00; 
	writeRegister16(ILI9341_FRAMECONTROL, iData);
	writeRegister8(ILI9341_ENTRYMODE, 0x07);
	writeRegister8(ILI9341_SLEEPOUT, 0);
	HAL_Delay(150);
	writeRegister8(ILI9341_DISPLAYON, 0);
	HAL_Delay(500);//ILI9341_INVERTON
	//ILI9341_fillScreen(ILI9341_BLACK);
	HAL_Delay(20);
}
//***************************************************************************
void ILI9341_setRotation(uint8_t rotate)
{
	switch(rotate)
	{
		case 1:
			rotationNum = 1;
			writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
			break;
		case 2:
			rotationNum = 2;
			writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
			break;
		case 3:
			rotationNum = 3;
			writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
			break;
		case 4:
			rotationNum = 4;
			writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
			break;
		default:
			rotationNum = 1;
			writeRegister8(ILI9341_MEMCONTROL,ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
			break;
	}
}
//***************************************************************************
void ILI9341_fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint8_t colrData[2];
	setAddrWindow(x, y, w+x, y+h);
	TFT_LCD_RS(false);
	write8(0x2c);
	TFT_LCD_RS(true);
	colrData[0] = 0xFF&(color>>8); colrData[1] = 0xFF&color;
	write8_fast(colrData, w*h);
}
//***************************************************************************
void ILI9341_drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	uint8_t colrData[2];
	setAddrWindow(x, y, TFTWIDTH-1, TFTHEIGHT-1);
	TFT_LCD_RS(false);
	write8(0x2c);
	TFT_LCD_RS(true);
	colrData[0] = 0xFF&(color>>8); colrData[1] = 0xFF&color;
	write8_fast(colrData, 1);
}
//***************************************************************************
void ILI9341_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  ILI9341_drawPixel(x0  , y0+r, color);
  ILI9341_drawPixel(x0  , y0-r, color);
  ILI9341_drawPixel(x0+r, y0  , color);
  ILI9341_drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    ILI9341_drawPixel(x0 + x, y0 + y, color);
    ILI9341_drawPixel(x0 - x, y0 + y, color);
    ILI9341_drawPixel(x0 + x, y0 - y, color);
    ILI9341_drawPixel(x0 - x, y0 - y, color);
    ILI9341_drawPixel(x0 + y, y0 + x, color);
    ILI9341_drawPixel(x0 - y, y0 + x, color);
    ILI9341_drawPixel(x0 + y, y0 - x, color);
    ILI9341_drawPixel(x0 - y, y0 - x, color);
  }
}
//***************************************************************************
void ILI9341_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      ILI9341_drawPixel(y0, x0, color);
    } else {
      ILI9341_drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}	
//***************************************************************************
void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	ILI9341_drawLine(x, y, x, y+h-1, color);
}
//***************************************************************************
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      ILI9341_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      ILI9341_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      ILI9341_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      ILI9341_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}
//***************************************************************************
void ILI9341_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	ILI9341_drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}
//***************************************************************************
void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	ILI9341_drawLine(x, y, x+w-1, y, color);
}
//***************************************************************************
void ILI9341_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	ILI9341_drawLine(x0, y0, x1, y1, color);
  ILI9341_drawLine(x1, y1, x2, y2, color);
  ILI9341_drawLine(x2, y2, x0, y0, color);
}
//***************************************************************************
void ILI9341_fillScreen(uint16_t color)
{
	//ILI9341_fillRect(0, 0, 320, 241, color); //240, 322
	if(rotationNum==1 || rotationNum==3)
	{
		ILI9341_fillRect(0, 0, 240, 322, color);
	}
	else if(rotationNum==2 || rotationNum==4)
	{
		ILI9341_fillRect(0, 0, 320, 241, color);
	}
}
//***************************************************************************
void ILI9341_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
	// smarter version
  ILI9341_drawFastHLine(x+r  , y    , w-2*r, color); // Top
  ILI9341_drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  ILI9341_drawFastVLine(x    , y+r  , h-2*r, color); // Left
  ILI9341_drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}
//***************************************************************************
static void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
	int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      ILI9341_drawPixel(x0 + x, y0 + y, color);
      ILI9341_drawPixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      ILI9341_drawPixel(x0 + x, y0 - y, color);
      ILI9341_drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      ILI9341_drawPixel(x0 - y, y0 + x, color);
      ILI9341_drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      ILI9341_drawPixel(x0 - y, y0 - x, color);
      ILI9341_drawPixel(x0 - x, y0 - y, color);
    }
  }
}
//***************************************************************************
void ILI9341_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	if((x >= TFTWIDTH)            || // Clip right
     (y >= TFTHEIGHT)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

  for (int8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5) 
      line = 0x0;
    else 
      line = pgm_read_byte(font+(c*5)+i);
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          ILI9341_drawPixel(x+i, y+j, color);
        else {  // big size
          ILI9341_fillRect(x+(i*size), y+(j*size), size, size+1, color);
        } 
      } else if (bg != color) {
        if (size == 1) // default size
          ILI9341_drawPixel(x+i, y+j, bg);
        else {  // big size
          ILI9341_fillRect(x+i*size, y+j*size, size, size+1, bg);
        }
      }
      line >>= 1;
    }
  }
}
//***************************************************************************
void ILI9341_printText(char text[], int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size)
{
	int16_t offset;
	offset = size*6;
	for(uint16_t i=0; i<40 && text[i]!=NULL; i++)
	{
		ILI9341_drawChar(x+(offset*i), y, text[i],color,bg,size);
	}
}
//***************************************************************************
void ILI9341_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
	// smarter version
  ILI9341_fillRect(x+r, y, w-2*r, h, color);
  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}
//***************************************************************************
void ILI9341_drawButton(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color,bool inverted)
{
	ILI9341_fillRoundRect(x - (w/2), y - (h/2), w, h, min(w,h)/4, color);
  ILI9341_drawRoundRect(x - (w/2), y - (h/2), w, h, min(w,h)/4, color);
}
//***************************************************************************
void ILI9341_printImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data, uint32_t size)
{
	setAddrWindow(x, y, w+x, h+y);
	TFT_LCD_RS(false);
	write8(0x2c);
	TFT_LCD_RS(true);
	for(uint32_t i=0; i<size;i++)
	{
		write8(data[i]);
	}
}
//***************************************************************************
void ILI9341_sleepIn(void)
{
	writeRegister8(ILI9341_SLEEPIN, 0);
}
//***************************************************************************
void ILI9341_sleepOut(void)
{
	writeRegister8(ILI9341_SLEEPOUT, 0);
}
//***************************************************************************
void ILI9341_displayOff(void)
{
	writeRegister8(ILI9341_DISPLAYOFF, 0);
}
//***************************************************************************
void ILI9341_displayOn(void)
{
	writeRegister8(ILI9341_DISPLAYON, 0);
}
//***************************************************************************
void ILI9341_invertOn(void)
{
	writeRegister8(ILI9341_INVERTON, 0);
}
//***************************************************************************
void ILI9341_invertOff(void)
{
	writeRegister8(ILI9341_INVERTOFF, 0);
}
//***************************************************************************
