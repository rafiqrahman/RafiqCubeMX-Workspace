//(1): Header files

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "Rafiq_ILI9325_TFT.h"

//(2): varaible declarations
static uint8_t rotationNum=1;
static bool _cp437    = false;

// Brush color, background color
uint16_t POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;

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
	
	GPIO_InitStruct.Pin = (	GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 |
													GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 |
													GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 |
													GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
	
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_Data_Lines_Port, &GPIO_InitStruct);
}
//***************************************************************************
static void GPIO_write(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = (	GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 |
													GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 |
													GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 |
													GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}
//***************************************************************************
static void LCD_WR_DATA(uint16_t data)
{
	TFT_LCD_CS(false);
	TFT_LCD_RS(true);
	DATAOUT(data);
	TFT_LCD_WR(false);
	TFT_LCD_WR(true);
	TFT_LCD_CS(true);
}
//***************************************************************************
static void LCD_WR_REG(uint8_t data)
{ 
	TFT_LCD_CS(false);
	TFT_LCD_RS(false);
	DATAOUT(data);
	TFT_LCD_WR(false);
	TFT_LCD_WR(true);
	TFT_LCD_CS(true);
}
//***************************************************************************
// Write register 
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);	    		 
}


//***************************************************************************
// Read register 
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{										   
	uint16_t t;
	LCD_WR_REG(LCD_Reg);  // Write the register address to be read  
	GPIO_read();
	TFT_LCD_CS(false);
	TFT_LCD_RS(true);
		// Read data (read the register, do not need to read 2 times)
	TFT_LCD_RD(false);
	TFT_LCD_RD(true);
	t=DATAIN;  
	TFT_LCD_CS(true);
	GPIO_write();
	return t;
}
//***************************************************************************
//Prepare to write GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(R34);
}
//***************************************************************************
//LCD write to GRAM
void LCD_WriteRAM(uint16_t RGB_Code)
{							    
	LCD_WR_DATA(RGB_Code);// Write sixteen Bit GRAM
}
//***************************************************************************
// The data read from ILI93xx is in GBR format, and we write it in RGB format.
// Converted by this function
// c: GBR format color value
// Return Value: RGB format color value
uint16_t LCD_BGR2RGB(uint16_t c)
{
  uint16_t  r, g, b, rgb;   
  b=(c>>0) & 0x1F;
  g=(c>>5) & 0x3F;
  r=(c>>11) & 0x1F;	 
  rgb=(b<<11) + (g<<5) + (r<<0);		 
  return(rgb);
}
//***************************************************************************
// Read the color value of a certain point	 
//x:0~239
//y:0~319
// Return Value: The color of this point
uint16_t LCD_ReadPoint(uint8_t x,uint16_t y)
{
	uint16_t t;			   
	LCD_SetCursor(x,y);
	LCD_WR_REG(R34);       // Select the GRAM address 
	GPIO_read();
	TFT_LCD_CS(false);
	TFT_LCD_RS(true);
	// Read data (read GRAM, you need to read 2 times)
	TFT_LCD_RD(false);
	TFT_LCD_RD(true);
	//dummy READ above
	TFT_LCD_RD(false);
	TFT_LCD_RD(true);
	t = DATAIN;  
	TFT_LCD_CS(true);
	GPIO_write();
	return LCD_BGR2RGB(t);
}
//***************************************************************************
//LCD Turn on the display
void LCD_DisplayOn(void)
{					   
	LCD_WriteReg(R7, 0x0173); //26 Wan color display open
}
//***************************************************************************
//LCD Turn off the display
void LCD_DisplayOff(void)
{	   
	LCD_WriteReg(R7, 0x0);// Turn off the display 
}
//***************************************************************************
// Set the cursor position
//Xpos:abscissa
//Ypos:y-axis
__inline void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos)
{
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);
}
//***************************************************************************
//Draw Point
//x:0~239
//y:0~319
//POINT_COLOR:The Color of this point
void LCD_DrawPoint(uint8_t x,uint16_t y)
{
	LCD_SetCursor(x,y);// Set the cursor position 
	LCD_WR_REG(R34);//Start writing GRAM
	LCD_WR_DATA(POINT_COLOR); 
}
//***************************************************************************
void LCD_Init(void)
{
  uint16_t DeviceCode;
	
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
 	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	
  /*Configure GPIO pin Output Level */
	//(1): Reset all pins
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_RESET);
	
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
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

	GPIO_InitStruct.Pin = (	GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 |
													GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 |
													GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 |
													GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
	HAL_GPIO_Init(LCD_Data_Lines_Port, &GPIO_InitStruct);
	
	TFT_LCD_CS(true);
	TFT_LCD_RD(true);
	TFT_LCD_RS(true);
	TFT_LCD_WR(true);
	
	//(1): Begin function
	HAL_Delay(50);
	LCD_WriteReg(0x0000,0x0001);
	HAL_Delay(50); // delay 50 ms 
	DeviceCode = LCD_ReadReg(0x0000);   
	//printf("\r\n");   
	//printf("ID=0x%x \r\n",DeviceCode);   
	if(DeviceCode==0x9325||DeviceCode==0x9328)//ILI9325
			{
				LCD_WriteReg(0x00e7,0x0010);      
        LCD_WriteReg(0x0000,0x0001);// Turn on the internal clock 
        LCD_WriteReg(0x0001,0x0100);     
        LCD_WriteReg(0x0002,0x0700);//Power On                    
				//LCD_WriteReg(0x0003,(1<<3)|(1<<4) ); 	//65K  RGB
				//DRIVE TABLE(Register 03H)
				//BIT3=AM BIT4:5=ID0:1
				//AM ID0 ID1   FUNCTION
				// 0  0   0	   R->L D->U
				// 1  0   0	   D->U	R->L
				// 0  1   0	   L->R D->U
				// 1  1   0    D->U	L->R
				// 0  0   1	   R->L U->D
				// 1  0   1    U->D	R->L
				// 0  1   1    L->R U->D Normal use this
				// 1  1   1	   U->D	L->R
        LCD_WriteReg(0x0003,(1<<12)|(3<<4)|(0<<3) );//65K    
        LCD_WriteReg(0x0004,0x0000);                                   
        LCD_WriteReg(0x0008,0x0207);	           
        LCD_WriteReg(0x0009,0x0000);         
        LCD_WriteReg(0x000a,0x0000);//display setting         
        LCD_WriteReg(0x000c,0x0001);//display setting          
        LCD_WriteReg(0x000d,0x0000);//0f3c          
        LCD_WriteReg(0x000f,0x0000);
				
				// Power configuration
        LCD_WriteReg(0x0010,0x0000);   
        LCD_WriteReg(0x0011,0x0007);
        LCD_WriteReg(0x0012,0x0000);                                                                 
        LCD_WriteReg(0x0013,0x0000);                 
        HAL_Delay(50);; 
        LCD_WriteReg(0x0010,0x1590);   
        LCD_WriteReg(0x0011,0x0227);
        HAL_Delay(50);; 
        LCD_WriteReg(0x0012,0x009c);                  
        HAL_Delay(50);; 
        LCD_WriteReg(0x0013,0x1900);   
        LCD_WriteReg(0x0029,0x0023);
        LCD_WriteReg(0x002b,0x000e);
        HAL_Delay(50);; 
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x013f);           
				HAL_Delay(50);; 
				
				//Gamma Correction
        LCD_WriteReg(0x0030,0x0007); 
        LCD_WriteReg(0x0031,0x0707);   
        LCD_WriteReg(0x0032,0x0006);
        LCD_WriteReg(0x0035,0x0704);
        LCD_WriteReg(0x0036,0x1f04); 
        LCD_WriteReg(0x0037,0x0004);
        LCD_WriteReg(0x0038,0x0000);        
        LCD_WriteReg(0x0039,0x0706);     
        LCD_WriteReg(0x003c,0x0701);
        LCD_WriteReg(0x003d,0x000f);
        HAL_Delay(50);; 
        
				LCD_WriteReg(0x0050,0x0000); // Horizontal GRAM start position 
        LCD_WriteReg(0x0051,0x00ef); // Horizontal GRAM end position                    
        LCD_WriteReg(0x0052,0x0000); // Vertical GRAM start position                    
        LCD_WriteReg(0x0053,0x013f); // Vertical GRAM termination position          
        LCD_WriteReg(0x0060,0xa700);        
        LCD_WriteReg(0x0061,0x0001); 
        LCD_WriteReg(0x006a,0x0000);
        LCD_WriteReg(0x0080,0x0000);
        LCD_WriteReg(0x0081,0x0000);
        LCD_WriteReg(0x0082,0x0000);
        LCD_WriteReg(0x0083,0x0000);
        LCD_WriteReg(0x0084,0x0000);
        LCD_WriteReg(0x0085,0x0000);      
        LCD_WriteReg(0x0090,0x0010);     
        LCD_WriteReg(0x0092,0x0000);  
        LCD_WriteReg(0x0093,0x0003);
        LCD_WriteReg(0x0095,0x0110);
        LCD_WriteReg(0x0097,0x0000);        
        LCD_WriteReg(0x0098,0x0000);  
        
				// Turn on the display settings    
        LCD_WriteReg(0x0007,0x0133);   
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x013f);	
			}
	
			else if(DeviceCode==0x9320||DeviceCode==0x9300)
			{
				LCD_WriteReg(0x00,0x0000);
				LCD_WriteReg(0x01,0x0100);	//Driver Output Control.
				LCD_WriteReg(0x02,0x0700);	//LCD Driver Waveform Control.
				LCD_WriteReg(0x03,0x1030);//Entry Mode Set.
				//LCD_WriteReg(0x03,0x1018);	//Entry Mode Set.
	
				LCD_WriteReg(0x04,0x0000);	//Scaling Control.
				LCD_WriteReg(0x08,0x0202);	//Display Control 2.(0x0207)
				LCD_WriteReg(0x09,0x0000);	//Display Control 3.(0x0000)
				LCD_WriteReg(0x0a,0x0000);	//Frame Cycle Control.(0x0000)
				LCD_WriteReg(0x0c,(1<<0));	//Extern Display Interface Control 1.(0x0000)
				LCD_WriteReg(0x0d,0x0000);	//Frame Maker Position.
				LCD_WriteReg(0x0f,0x0000);	//Extern Display Interface Control 2.	    
				HAL_Delay(50);; 
				LCD_WriteReg(0x07,0x0101);	//Display Control.
				HAL_Delay(50);; 								  
				LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
				LCD_WriteReg(0x11,0x0007);								//Power Control 2.(0x0001)
				LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));				//Power Control 3.(0x0138)
				LCD_WriteReg(0x13,0x0b00);								//Power Control 4.
				LCD_WriteReg(0x29,0x0000);								//Power Control 7.
	
				LCD_WriteReg(0x2b,(1<<14)|(1<<4));	    
				LCD_WriteReg(0x50,0);	//Set X Start
				//Horizontal GRAM end position..
				LCD_WriteReg(0x51,239);	// Set X End 
				LCD_WriteReg(0x52,0);	// Set Y Start 
				LCD_WriteReg(0x53,319);	// Set Y End 
	
				LCD_WriteReg(0x60,0x2700);	//Driver Output Control.
				LCD_WriteReg(0x61,0x0001);	//Driver Output Control.
				LCD_WriteReg(0x6a,0x0000);	//Vertical Scroll Control.
	
				LCD_WriteReg(0x80,0x0000);	//Display Position? Partial Display 1.
				LCD_WriteReg(0x81,0x0000);	//RAM Address Start? Partial Display 1.
				LCD_WriteReg(0x82,0x0000);	//RAM Address End-Partial Display 1.
				LCD_WriteReg(0x83,0x0000);	//Display Position? Partial Display 2.
				LCD_WriteReg(0x84,0x0000);	//RAM Address Start? Partial Display 2.
				LCD_WriteReg(0x85,0x0000);	//RAM Address End? Partial Display 2.
	
				LCD_WriteReg(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
				LCD_WriteReg(0x92,0x0000);	//Panel Interface Control 2.(0x0000)
				LCD_WriteReg(0x93,0x0001);	//Panel Interface Control 3.
				LCD_WriteReg(0x95,0x0110);	//Frame Cycle Control.(0x0110)
				LCD_WriteReg(0x97,(0<<8));	//
				LCD_WriteReg(0x98,0x0000);	//Frame Cycle Control.	   
				LCD_WriteReg(0x07,0x0173);	//(0x0173)	
			}
			
			else if(DeviceCode==0x1505)					
			{		
				// second release on 3/5  ,luminance is acceptable, water wave appear during camera preview
				LCD_WriteReg(0x0007,0x0000);
        HAL_Delay(50);; 
        LCD_WriteReg(0x0012,0x011C);//0x011A   why need to set several times?
        LCD_WriteReg(0x00A4,0x0001);//NVM	 
        LCD_WriteReg(0x0008,0x000F);
        LCD_WriteReg(0x000A,0x0008);
        LCD_WriteReg(0x000D,0x0008);	    
				
				//Gamma Correction
        LCD_WriteReg(0x0030,0x0707);
        LCD_WriteReg(0x0031,0x0007); //0x0707
        LCD_WriteReg(0x0032,0x0603); 
        LCD_WriteReg(0x0033,0x0700); 
        LCD_WriteReg(0x0034,0x0202); 
        LCD_WriteReg(0x0035,0x0002); //?0x0606
        LCD_WriteReg(0x0036,0x1F0F);
        LCD_WriteReg(0x0037,0x0707); //0x0f0f  0x0105
        LCD_WriteReg(0x0038,0x0000); 
        LCD_WriteReg(0x0039,0x0000); 
        LCD_WriteReg(0x003A,0x0707); 
        LCD_WriteReg(0x003B,0x0000); //0x0303
        LCD_WriteReg(0x003C,0x0007); //?0x0707
        LCD_WriteReg(0x003D,0x0000); //0x1313//0x1f08
        HAL_Delay(50);; 
        LCD_WriteReg(0x0007,0x0001);
        LCD_WriteReg(0x0017,0x0001);// Turn on the power
        HAL_Delay(50);; 
				
				// Power configuration
        LCD_WriteReg(0x0010,0x17A0); 
        LCD_WriteReg(0x0011,0x0217);//reference voltage VC[2:0]   Vciout = 1.00*Vcivl
        LCD_WriteReg(0x0012,0x011E);//0x011c  //Vreg1out = Vcilvl*1.80   is it the same as Vgama1out ?
        LCD_WriteReg(0x0013,0x0F00);//VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl
        LCD_WriteReg(0x002A,0x0000);  
        LCD_WriteReg(0x0029,0x000A);//0x0001F  Vcomh = VCM1[4:0]*Vreg1out    gate source voltage??
        LCD_WriteReg(0x0012,0x013E);// 0x013C  power supply on
        
				//Coordinates Control//
        LCD_WriteReg(0x0050,0x0000);//0x0e00
        LCD_WriteReg(0x0051,0x00EF); 
        LCD_WriteReg(0x0052,0x0000); 
        LCD_WriteReg(0x0053,0x013F); 
				
				//Pannel Image Control//
        LCD_WriteReg(0x0060,0x2700); 
        LCD_WriteReg(0x0061,0x0001); 
        LCD_WriteReg(0x006A,0x0000); 
        LCD_WriteReg(0x0080,0x0000); 
				
				//Partial Image Control//
        LCD_WriteReg(0x0081,0x0000); 
        LCD_WriteReg(0x0082,0x0000); 
        LCD_WriteReg(0x0083,0x0000); 
        LCD_WriteReg(0x0084,0x0000); 
        LCD_WriteReg(0x0085,0x0000); 
				
				//Panel Interface Control//
        LCD_WriteReg(0x0090,0x0013);//0x0010 frequency
        LCD_WriteReg(0x0092,0x0300); 
        LCD_WriteReg(0x0093,0x0005); 
        LCD_WriteReg(0x0095,0x0000); 
        LCD_WriteReg(0x0097,0x0000); 
        LCD_WriteReg(0x0098,0x0000); 
  
        LCD_WriteReg(0x0001,0x0100); 
        LCD_WriteReg(0x0002,0x0700); 
        LCD_WriteReg(0x0003,0x1030); 
        LCD_WriteReg(0x0004,0x0000); 
        LCD_WriteReg(0x000C,0x0000); 
        LCD_WriteReg(0x000F,0x0000); 
        LCD_WriteReg(0x0020,0x0000); 
        LCD_WriteReg(0x0021,0x0000); 
        LCD_WriteReg(0x0007,0x0021); 
        HAL_Delay(200);;
        LCD_WriteReg(0x0007,0x0061); 
        HAL_Delay(200);;
        LCD_WriteReg(0x0007,0x0173); 
        HAL_Delay(200);;					
			}
			
			else if(DeviceCode==0x8989)
			{
				LCD_WriteReg(0x0000,0x0001);HAL_Delay(50);// Open the crystal
				LCD_WriteReg(0x0003,0xA8A4);HAL_Delay(50);//0xA8A4
				LCD_WriteReg(0x000C,0x0000);HAL_Delay(50);    
				LCD_WriteReg(0x000D,0x080C);HAL_Delay(50);    
				LCD_WriteReg(0x000E,0x2B00);HAL_Delay(50);    
				LCD_WriteReg(0x001E,0x00B0);HAL_Delay(50);    
				LCD_WriteReg(0x0001,0x2B3F);HAL_Delay(50);//Drive Output Control 320*240  0x6B3F
				LCD_WriteReg(0x0002,0x0600);HAL_Delay(50); 
				LCD_WriteReg(0x0010,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0011,0x6070);HAL_Delay(50);// Define data format 16-bit color horizontal screen 0x6058
				LCD_WriteReg(0x0005,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0006,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0016,0xEF1C);HAL_Delay(50); 
				LCD_WriteReg(0x0017,0x0003);HAL_Delay(50); 
				LCD_WriteReg(0x0007,0x0233);HAL_Delay(50);//0x0233       
				LCD_WriteReg(0x000B,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x000F,0x0000);HAL_Delay(50);// Scan start address
				LCD_WriteReg(0x0041,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0042,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0048,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0049,0x013F);HAL_Delay(50); 
				LCD_WriteReg(0x004A,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x004B,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0044,0xEF00);HAL_Delay(50); 
				LCD_WriteReg(0x0045,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0046,0x013F);HAL_Delay(50); 
				LCD_WriteReg(0x0030,0x0707);HAL_Delay(50); 
				LCD_WriteReg(0x0031,0x0204);HAL_Delay(50); 
				LCD_WriteReg(0x0032,0x0204);HAL_Delay(50); 
				LCD_WriteReg(0x0033,0x0502);HAL_Delay(50); 
				LCD_WriteReg(0x0034,0x0507);HAL_Delay(50); 
				LCD_WriteReg(0x0035,0x0204);HAL_Delay(50); 
				LCD_WriteReg(0x0036,0x0204);HAL_Delay(50); 
				LCD_WriteReg(0x0037,0x0502);HAL_Delay(50); 
				LCD_WriteReg(0x003A,0x0302);HAL_Delay(50); 
				LCD_WriteReg(0x003B,0x0302);HAL_Delay(50); 
				LCD_WriteReg(0x0023,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0024,0x0000);HAL_Delay(50); 
				LCD_WriteReg(0x0025,0x8000);HAL_Delay(50); 
				LCD_WriteReg(0x004f,0);        //The first line 0
				LCD_WriteReg(0x004e,0);        //The first line 0	
			}
				
			HAL_Delay(500);
			//LCD_LED=0;// Lit the backlight	 
			LCD_Clear(ILI93xx_CYAN);
}  		  
//***************************************************************************
// Clear screen function 
// Color: fill color to be cleared
void LCD_Clear(uint16_t Color)
{
	uint32_t index=0;      
	LCD_SetCursor(0x00,0x0000);//Set the cursor position 
	LCD_WriteRAM_Prepare();     // Start writing GRAM	 	  
	for(index=0;index<76800;index++)
	{
		LCD_WR_DATA(Color);    
	}
}
//***************************************************************************
