#ifndef __LCD_H
#define __LCD_H

#include "stdio.h"	 
#include "stdlib.h"
#include "stm32f1xx.h"

typedef signed   char  u8;
typedef unsigned char  u8;
typedef signed   short s16;
typedef unsigned short u16;
typedef signed   long  s32;
typedef unsigned long  u32;


//2.4/2.8 inch TFTLCD Driver (ILI9325/RM68021/ILI9320 Verision)
//TFTLCD Driver Code			 
//???@Dream
//www.openmcu.com
//2010/11/5

//TFTLCD ExternalVariables		   
extern u16  POINT_COLOR;//Default Red    
extern u16  BACK_COLOR; // Background color. The default is white
//-----------------LCD Port Definition---------------- 
//#define	LCD_LED PCout(10) //LCD Backlight    		 PC10
#define	LCD_CS	PDout(12)  //Chip Select  	     PD12
#define	LCD_RS	PDout(13)  //Data/Command       PD13	   
#define	LCD_WR	PDout(14)  //Write Pin			 PD14
#define	LCD_RD	PDout(15)  //Read Pin			 PD15

								    
//PE0~15,As Data Line
#define DATAOUT(x) GPIOE->ODR=x; // Data output	
#define DATAIN     GPIOE->IDR;   //Data input     
//Brush Color
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //brown
#define BRRED 			 0XFC07 //maroon
#define GRAY  			 0X8430 //grey
#define LGRAY 			 0XC618 //light grey
	    															  
extern u16 BACK_COLOR, POINT_COLOR ;  
void LCD_Init(void);
void LCD_Clear(u16 Color);	 
void LCD_SetCursor(u8 Xpos, u16 Ypos);
void LCD_DrawPoint(u8 x,u16 y);//Draw point
u16 LCD_ReadPoint(u8 x,u16 y); //Reading point
void Draw_Circle(u8 x0,u16 y0,u8 r);
void LCD_DrawLine(u8 x1, u16 y1, u8 x2, u16 y2);
void LCD_DrawRectangle(u8 x1, u16 y1, u8 x2, u16 y2);		   
void LCD_Fill(u8 xsta,u16 ysta,u8 xend,u16 yend,u16 color);
void LCD_ShowChar(u8 x,u16 y,u8 num,u8 size,u8 mode);//Display a character
void LCD_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);  //Display a number
void LCD_ShowString(u8 x,u16 y,const u8 *p);		 // Show a string, 16 fonts
									    
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16 LCD_ReadRAM(void);		   

																					 
//9320/9325 LCD register  
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5							  		 
#endif  
	 


