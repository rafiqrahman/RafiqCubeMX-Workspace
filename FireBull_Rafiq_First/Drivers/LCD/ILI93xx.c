#include "lcd.h"
#include "font.h" 
#include "stdio.h"	 
//2.8/3.Inch TFT LCD Driver(ILI9325/RM68021/ILI9320 Verision)
//TFTLCD Driver Code			 
//???@Dream
//www.openmcu.com
//2010/11/5

// Brush color, background color
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  

// Write 8-bit data function 
// Use macro definitions to increase speed
#define LCD_WR_DATA(data){\
LCD_RS=1;\
LCD_CS=0;\
DATAOUT(data);\
LCD_WR=0;\
LCD_WR=1;\
LCD_CS=1;\
} 
// Write register function 
void LCD_WR_REG(u8 data)
{ 
	LCD_RS=0;//Write Address  
 	LCD_CS=0; 
	DATAOUT(data); 
	LCD_WR=0; 
	LCD_WR=1; 
 	LCD_CS=1;  
}  
// Write register 
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   
// Read register 
u16 LCD_ReadReg(u8 LCD_Reg)
{										   
	u16 t;
	LCD_WR_REG(LCD_Reg);  // Write the register number to be read  
	GPIOE->CRL=0X88888888; //PB0-7  Pull-up input
	GPIOE->CRH=0X88888888; //PB8-15 Pull-up input 
	GPIOE->ODR=0XFFFF;    // All output high
	LCD_RS=1;
	LCD_CS=0;
	// Read data (read the register, do not need to read 2 times)
	LCD_RD=0;					   
	LCD_RD=1;
	t=DATAIN;  
	LCD_CS=1;   
	GPIOE->CRL=0X33333333; //PB0-7  Pull-up output
	GPIOE->CRH=0X33333333; //PB8-15 Pull-up output
	GPIOE->ODR=0XFFFF;    // All output high
	return t;  
}   
//Start to write GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(R34);
}	 
//LCD write GRAM
void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD_WR_DATA(RGB_Code);// Write sixteen Bit GRAM
}

// The data read from ILI93xx is in GBR format, and we write it in RGB format.
// Converted by this function
// c: GBR format color value
// Return Value: RGB format color value
u16 LCD_BGR2RGB(u16 c)
{
  u16  r,g,b,rgb;   
  b=(c>>0)&0x1f;
  g=(c>>5)&0x3f;
  r=(c>>11)&0x1f;	 
  rgb=(b<<11)+(g<<5)+(r<<0);		 
  return(rgb);
}		 
// Read the color value of a certain point	 
//x:0~239
//y:0~319
// Return Value: The color of this point
u16 LCD_ReadPoint(u8 x,u16 y)
{
	u16 t;			   
	LCD_SetCursor(x,y);
	LCD_WR_REG(R34);       // Select the GRAM address 
	GPIOE->CRL=0X88888888; //PB0-7  Pull-up input
	GPIOE->CRH=0X88888888; //PB8-15 Pull-up input
	GPIOE->ODR=0XFFFF;     // All output high
	LCD_RS=1;
	LCD_CS=0;
	// Read data (read GRAM, you need to read 2 times)
	LCD_RD=0;					   
	LCD_RD=1;
	//dummy READ
	LCD_RD=0;					   
	LCD_RD=1;
	t=DATAIN;  
	LCD_CS=1;   
	GPIOE->CRL=0X33333333; //PB0-7  Pull-up output 
	GPIOE->CRH=0X33333333; //PB8-15 Pull-up output
	GPIOE->ODR=0XFFFF;    // All output high  
	return LCD_BGR2RGB(t);
}
//LCDTurn on the display
void LCD_DisplayOn(void)
{					   
	LCD_WriteReg(R7, 0x0173); //26 Wan color display open
}	 
//LCD Close the display
void LCD_DisplayOff(void)
{	   
	LCD_WriteReg(R7, 0x0);// Close the display 
}    
//LCD Delay function 10MS
void Delay (u32 nCount)
{
	volatile int i;	 	
	for (i=0;i<nCount*100;i++);
}

// Set the cursor position
//Xpos:abscissa
//Ypos:y-axis
__inline void LCD_SetCursor(u8 Xpos, u16 Ypos)
{
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);
} 
//Draw Point
//x:0~239
//y:0~319
//POINT_COLOR:The Color of this point
void LCD_DrawPoint(u8 x,u16 y)
{
	LCD_SetCursor(x,y);// Set the cursor position 
	LCD_WR_REG(R34);//Start writing GRAM
	LCD_WR_DATA(POINT_COLOR); 
} 	 
//Initialization lcd
// The initialization function can initialize a variety of ILI93XX LCD, but other functions are based on the ILI9320!
// No test on other driver chips!
void LCD_Init(void)
{ 
	u16 DeviceCode;	 
 	RCC->APB2ENR|=1<<5;// Enable periprhral clock first PORTD
 	RCC->APB2ENR|=1<<6;// Enable periprhral clock first PORTE

	RCC->APB2ENR|=1<<0;    // Turn on the auxiliary clock 
	//AFIO->MAPR&=~(7<<24);  // Close JTAG, keep SWD 
	//AFIO->MAPR|=2<<24;     // Close JTAG, keep SWD 
											 
	//PORTD12~15 Multiplexed push-pull output 	
	GPIOD->CRH&=0X0000FFFF;
	GPIOD->CRH|=0X33330000; 
	GPIOD->ODR|=0XF000; 	 
	//PORTE Push-pull output 	
	GPIOE->CRH=0X33333333;
	GPIOE->CRL=0X33333333; 	 
	GPIOE->ODR=0XFFFF;
	  					 
	Delay(5); // delay 50 ms 
	LCD_WriteReg(0x0000,0x0001);
	Delay(5); // delay 50 ms 
	DeviceCode = LCD_ReadReg(0x0000);   
	printf("\r\n");   
	printf("ID=0x%x \r\n",DeviceCode);   
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
        Delay(5); 
        LCD_WriteReg(0x0010,0x1590);   
        LCD_WriteReg(0x0011,0x0227);
        Delay(5); 
        LCD_WriteReg(0x0012,0x009c);                  
        Delay(5); 
        LCD_WriteReg(0x0013,0x1900);   
        LCD_WriteReg(0x0029,0x0023);
        LCD_WriteReg(0x002b,0x000e);
        Delay(5); 
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x013f);           
		Delay(5); 
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
        Delay(5); 
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
		Delay(5); 
		LCD_WriteReg(0x07,0x0101);	//Display Control.
		Delay(5); 								  
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
        Delay(5); 
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
        Delay(5); 
        LCD_WriteReg(0x0007,0x0001);
        LCD_WriteReg(0x0017,0x0001);// Turn on the power
        Delay(5); 
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
        Delay(20);
        LCD_WriteReg(0x0007,0x0061); 
        Delay(20);
        LCD_WriteReg(0x0007,0x0173); 
        Delay(20);
	}							 
	else if(DeviceCode==0x8989)
	{
		LCD_WriteReg(0x0000,0x0001);Delay(5);// Open the crystal
    	LCD_WriteReg(0x0003,0xA8A4);Delay(5);//0xA8A4
    	LCD_WriteReg(0x000C,0x0000);Delay(5);    
    	LCD_WriteReg(0x000D,0x080C);Delay(5);    
    	LCD_WriteReg(0x000E,0x2B00);Delay(5);    
    	LCD_WriteReg(0x001E,0x00B0);Delay(5);    
    	LCD_WriteReg(0x0001,0x2B3F);Delay(5);//Drive Output Control 320*240  0x6B3F
    	LCD_WriteReg(0x0002,0x0600);Delay(5); 
    	LCD_WriteReg(0x0010,0x0000);Delay(5); 
    	LCD_WriteReg(0x0011,0x6070);Delay(5);// Define data format 16-bit color horizontal screen 0x6058
    	LCD_WriteReg(0x0005,0x0000);Delay(5); 
    	LCD_WriteReg(0x0006,0x0000);Delay(5); 
    	LCD_WriteReg(0x0016,0xEF1C);Delay(5); 
    	LCD_WriteReg(0x0017,0x0003);Delay(5); 
    	LCD_WriteReg(0x0007,0x0233);Delay(5);//0x0233       
    	LCD_WriteReg(0x000B,0x0000);Delay(5); 
    	LCD_WriteReg(0x000F,0x0000);Delay(5);// Scan start address
    	LCD_WriteReg(0x0041,0x0000);Delay(5); 
    	LCD_WriteReg(0x0042,0x0000);Delay(5); 
    	LCD_WriteReg(0x0048,0x0000);Delay(5); 
    	LCD_WriteReg(0x0049,0x013F);Delay(5); 
    	LCD_WriteReg(0x004A,0x0000);Delay(5); 
    	LCD_WriteReg(0x004B,0x0000);Delay(5); 
    	LCD_WriteReg(0x0044,0xEF00);Delay(5); 
    	LCD_WriteReg(0x0045,0x0000);Delay(5); 
    	LCD_WriteReg(0x0046,0x013F);Delay(5); 
    	LCD_WriteReg(0x0030,0x0707);Delay(5); 
    	LCD_WriteReg(0x0031,0x0204);Delay(5); 
    	LCD_WriteReg(0x0032,0x0204);Delay(5); 
    	LCD_WriteReg(0x0033,0x0502);Delay(5); 
    	LCD_WriteReg(0x0034,0x0507);Delay(5); 
    	LCD_WriteReg(0x0035,0x0204);Delay(5); 
    	LCD_WriteReg(0x0036,0x0204);Delay(5); 
    	LCD_WriteReg(0x0037,0x0502);Delay(5); 
    	LCD_WriteReg(0x003A,0x0302);Delay(5); 
    	LCD_WriteReg(0x003B,0x0302);Delay(5); 
    	LCD_WriteReg(0x0023,0x0000);Delay(5); 
    	LCD_WriteReg(0x0024,0x0000);Delay(5); 
    	LCD_WriteReg(0x0025,0x8000);Delay(5); 
    	LCD_WriteReg(0x004f,0);        //The first line 0
    	LCD_WriteReg(0x004e,0);        //The first line 0
	}					  
	Delay(5000);
	//LCD_LED=0;// Lit the backlight	 
	LCD_Clear(WHITE);
}  		  
  

// Clear screen function 
// Color: fill color to be cleared
void LCD_Clear(u16 Color)
{
	u32 index=0;      
	LCD_SetCursor(0x00,0x0000);//Set the cursor position 
	LCD_WriteRAM_Prepare();     // Start writing GRAM	 	  
	for(index=0;index<76800;index++)
	{
		LCD_WR_DATA(Color);    
	}
}  
// Fill in the specified area with the specified color
//Area Size:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(u8 xsta,u16 ysta,u8 xend,u16 yend,u16 color)
{                    
    u32 n;
	//Settings window										
	LCD_WriteReg(R80, xsta); // GRAM start address in horizontal direction
	LCD_WriteReg(R81, xend); // Horizontal direction GRAM end address
	LCD_WriteReg(R82, ysta); // Vertical start address of GRAM
	LCD_WriteReg(R83, yend); // Vertical GRAM end address	
	LCD_SetCursor(xsta,ysta);//Set the cursor position  
	LCD_WriteRAM_Prepare();  //Start writing GRAM	 	   	   
	n=(u32)(yend-ysta+1)*(xend-xsta+1);    
	while(n--){LCD_WR_DATA(color);}// Display the filled color 
	//Restore settings
	LCD_WriteReg(R80, 0x0000); // GRAM start address in horizontal direction
	LCD_WriteReg(R81, 0x00EF); // GRAM end address in horizontal direction
	LCD_WriteReg(R82, 0x0000); // GRAM start address in vertical direction
	LCD_WriteReg(R83, 0x013F); // GRAM end address in vertical direction	    
}  
//Draw a line
//x1,y1:Starting point coordinates
//x2,y2:End point coordinates  
void LCD_DrawLine(u8 x1, u16 y1, u8 x2, u16 y2)
{
    u16 x, y, t;
	if((x1==x2)&&(y1==y2))LCD_DrawPoint(x1, y1);
	else if(abs(y2-y1)>abs(x2-x1))//Slope is greater than 1 
	{
		if(y1>y2) 
		{
			t=y1;
			y1=y2;
			y2=t; 
			t=x1;
			x1=x2;
			x2=t; 
		}
		for(y=y1;y<y2;y++)// Based on the y-axis 
		{
			x=(u32)(y-y1)*(x2-x1)/(y2-y1)+x1;
			LCD_DrawPoint(x, y);  
		}
	}
	else     // Slope less than or equal to 1 
	{
		if(x1>x2)
		{
			t=y1;
			y1=y2;
			y2=t;
			t=x1;
			x1=x2;
			x2=t;
		}   
		for(x=x1;x<=x2;x++)// Based on the x-axis 
		{
			y =(u32)(x-x1)*(y2-y1)/(x2-x1)+y1;
			LCD_DrawPoint(x,y); 
		}
	} 
}    
//Image Rectangle
void LCD_DrawRectangle(u8 x1, u16 y1, u8 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//Draw a circle of a specific size at the specified position
//(x,y):Centre Point
//r    :radius
void Draw_Circle(u8 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             // Judge the next location of the flag
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		// Draw a circle using the Bresenham algorithm     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 
// Display a character at the specified location
//x:0~234
//y:0~308
//num: The characters to display:" "--->"~"
//size: Font Size 12/16
//mode: Overlay (1) or non-overlay (0)
void LCD_ShowChar(u8 x,u16 y,u8 num,u8 size,u8 mode)
{       
#define MAX_CHAR_POSX 232
#define MAX_CHAR_POSY 304 
    u8 temp;
    u8 pos,t;      
    if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;	    
	// Settings window										
	LCD_WriteReg(R80,x);           // GRAM start address in horizontal direction
	LCD_WriteReg(R81,x+(size/2-1));// GRAM end address in horizontal direction
	LCD_WriteReg(R82,y);           // GRAM start address in vertical direction 
	LCD_WriteReg(R83,y+size-1);    // GRAM end address in vertical direction	
	LCD_SetCursor(x,y);            //Set the cursor position  
	LCD_WriteRAM_Prepare();        //Start writing to GRAM	   
	num=num-' ';// Get the offset value
	if(!mode) // Non-superimposed way
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];// Call 1206 fonts
			else temp=asc2_1608[num][pos]; // Call 1608 fonts
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)
				{
					LCD_WR_DATA(POINT_COLOR);
				}else LCD_WR_DATA(BACK_COLOR);	        
		        temp>>=1; 
		    }
		}	
	}else//Overlay method
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];// Call 1206 fonts
			else temp=asc2_1608[num][pos];		 // Call 1608 fonts
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//Draw a point     
		        temp>>=1; 
		    }
		}
	}	    
	// Restore the form size	 
	LCD_WriteReg(R80, 0x0000); // GRAM start address in horizontal direction
	LCD_WriteReg(R81, 0x00EF); // GRAM end address in horizontal direction
	LCD_WriteReg(R82, 0x0000); // GRAM start address in vertical direction
	LCD_WriteReg(R83, 0x013F); // GRAM end address in vertical direction 
}  
//m^n function
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//Show 2 numbers
//x,y : Starting point coordinates	 
//len : Number of digits
//size:font size
//color:colour
//num:value(0~4294967295);	 
void LCD_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
//Display the string 
//x,y:Starting point coordinates  
//*p:String start addres
// Use 16 fonts
void LCD_ShowString(u8 x,u16 y,const u8 *p)
{         
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(WHITE);}
        LCD_ShowChar(x,y,*p,16,0);
        x+=8;
        p++;
    }  
}































