/******************** (C) COPYRIGHT 2010 KSGIANTLUS ********************
* File Name          : lcd.c
* Author             : JASON
* Version            : V1.0
* Date               : 2010.01.20
*************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "lcd.h"
#include "fonts.h"

u8   DotclkValue = 31; //MHz		   //53HZ

u16  VDP= 800;     
u16  VBP= 23 ;    //
u16  VFP= 23 ;    //

u16  HDP= 480;     
u16  HBP= 58;     //	
u16  HFP= 58;     //	

u16	 VPW=4;  //通常不需要调整
u16	 HPW=6;  //通常不需要调整

#define YDP 800	   //对应0XB0使用0x30
#define XDP 480   


static  vu16 TextColor = 0x0000, BackColor = 0xFFFF;
u16 DeviceCode;
extern void Delay(u32 nCount);

u16 mm=0;
u8 FontR,FontG,FontB;
extern u8 SDShowTimes;
/*******************************************************************************
* Function Name  : STM32_SSD1963_Init
* Description    : Initializes the SSD1963.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM32_SSD1963_Init(void) //Initializes the SSD1963.
{ 
	u16   VT,HT,VPS,HPS;
	float temp ;
	u32  LCDC_FPR;
	temp=((DotclkValue<<20)/100-1);
	LCDC_FPR=(u32)temp;
	VT=VBP+VFP+VDP;
	HT=HBP+HFP+HDP;
	HPS=HBP;
	VPS=VBP;
	
	Delay(5);
	/* Configure the LCD Control pins -LCD 相关控制端口初始化----------------------*/
	LCD_CtrlLinesConfig();
	
	Delay(5); /* delay 50 ms */
  
/* Start Initial Sequence ----------------------------------------------------*/
{//初始化SSD1963    
	
	LCD_RST(0);//硬件复位
	         SSD1963_CS(1); 
	         LCD_RS(1); 
	         LCD_WR(1); LCD_RD(1);
	Delay(5);
	LCD_RST(1) ;
	LCD_RD(1);
	Delay(20);

	{
	  //software reset
	   SSD1963Command_8(0x01); 
	   Delay(20);
	   SSD1963Command_8(0x01); 
	   Delay(20);   
	                       
	   //set_pixel_data_interface-设置MCU到1963的数据格式
	   SSD1963Command_8(0xf0);
	   SSD1963Data_8(0x00);   //0x00: 8-bit; 0x03:16-bit (565 format)
	   
	     
	    
	   //set_pll_mn;  m= 3 m= 2  设置1963内部PLL，基本不须修改
	   SSD1963Command_8(0xe2); //VCO = Reference input clock x (M + 1);PLL frequency = VCO / (N + 1)
	   //* Note : 250MHz < VCO < 800MHz     
	   SSD1963Data_8(49);   //Multiplier (M) of PLL. (POR = 00101101) ,M<256
	   SSD1963Data_8(3);   //Divider (N) of PLL. (POR = 0011),N<16
	   SSD1963Data_8(0x04); // Effectuate the multiplier and divider value
	   
	   //Start the PLL. Before the start, the system was operated with the crystal oscillator or clock input.
	   SSD1963Command_8(0xe0);
	   SSD1963Data_8(0x01);
	   Delay(10);
	   SSD1963Command_8(0xe0);  
	   SSD1963Data_8(0x03); 
		Delay(28);


	   //dotclk setting,Set the LSHIFT (pixel clock) frequency --For parallel LCD interface:
	   SSD1963Command_8(0xe6);  //:Configure the pixel clock 
	   // For parallel LCD interface DOTCLK Freq =PLL freq x ((LCDC_FPR + 1) / 1048576)
	   SSD1963Data_8(LCDC_FPR>>16);
	   SSD1963Data_8(LCDC_FPR>>8); 
	   SSD1963Data_8(LCDC_FPR);
	
	
	   //Set_lcd_mode,Panel setting
	   SSD1963Command_8(0xb0);  // Delay(10);
	   SSD1963Data_8(0x34);   // Set the LCD panel mode and resolution;TFT color depth enhancement enable;;
	   //LSHIFT polarity;Set the horizontal sync pulse polarity;LFRAME polarity (POR = 0)
	   //此处vs hs dotclk的极性设置于1963规格书说明相反，估计资料错误
	   SSD1963Data_8(0x00);   //B[7] : LCD panel mode (POR = 0)
	                          //     0 Hsync+Vsync +DE mode
	                          //     1 TTL mode
	                          // B[6:5] : TFT type (POR = 01)
	                          //  00, 01 TFT mode
	                         //  10 Serial RGB mode
	                         //  11 Serial RGB+dummy mode
	   
	   //Horizontal panel size = (HDP + 1) pixels
	   SSD1963Data_8((HDP-1)>>8); //High byte of the horizontal panel size       
	   SSD1963Data_8(HDP-1);  //Low byte of the horizontal panel size;
	   
	   SSD1963Data_8((VDP-1)>>8); //High byte of the vertical panel size
	   //Vertical panel size = (VDP + 1) lines
	   SSD1963Data_8(VDP-1); //Low byte of the vertical panel size (POR = 11011111)
	   
	   SSD1963Data_8(0x23); //RGB sequence for serial TFT interface
	
	   //hsync setting
	   SSD1963Command_8(0xb4); 
	   SSD1963Data_8((HT-1)>>8);  //HT[10:8];(display + non-display),
	   SSD1963Data_8(HT-1);   //HT[7:0];Horizontal total period = (HT + 1) pixels
	   SSD1963Data_8(HPS>>8);  //HPS[10:8] : High byte of the non-display period between the start of the horizontal sync (LLINE) signal and the first display data. (POR = 000)
	   SSD1963Data_8(HPS);   //HPS[7:0]
	   SSD1963Data_8(HPW);   //HPW[6:0] : Set the horizontal sync pulse width (LLINE) in pixel clock. (POR = 000111)
	                          //Horizontal Sync Pulse Width = (HPW + 1) pixels
	   SSD1963Data_8(0x00);  //LPS[10:8] :Set the horizontal sync pulse (LLINE) start location in pixel clock. (POR = 000)
	   SSD1963Data_8(0x00);   //LPS[7:0] :Set the horizontal sync pulse width (LLINE) in start. (POR = 00000000)
	   SSD1963Data_8(0x00);  //Set the horizontal sync pulse subpixel start position (POR = 00)
	
	   
	
	   //vsync setting
	   SSD1963Command_8(0xb6); //Set the vertical blanking interval between last scan line and next LFRAME pulse
	   SSD1963Data_8(VT>>8);    //VT:High byte of the vertical total (display + non-display) period in lines (POR = 001)
	   SSD1963Data_8(VT-1);   //Low byte of the vertical total (display + non-display) period in lines (POR = 11101111)
	                          //Vertical Total = (VT + 1) lines
	   SSD1963Data_8(VPS>>8);    //VPS[10:8] :High byte the non-display period in lines between the start of the frame and the first display data in line.
	   SSD1963Data_8(VPS);     //VPS[7:0] :The non-display period in lines between the start of the frame and the first display data in line.
	                          //Vertical Sync Pulse Start Position = VPS lines
	   SSD1963Data_8(VPW);    //VPW[6:0] :Set the vertical sync pulse width (LFRAME) in lines. (POR = 000001)
	                          //Vertical Sync Pulse Width = (VPW + 1) lines
	   SSD1963Data_8(0x00);   // FPS[10:8] : High byte of the vertical sync pulse (LFRAME) start location in lines. (POR = 000)
	   SSD1963Data_8(0x00);   // FPS[7:0] :Low byte of the vertical sync pulse (LFRAME) start location in lines. (POR = 00000000)
	                           //Vertical Display Period Start Position = FPS lines 
	
	    
	
	   SSD1963Command_8(0x36);//Set the read order from host processor to frame buffer by A[7:5] and A[3] and from frame buffer to the display panel by A[2:0] and A[4].
	   SSD1963Data_8(0x00);   //数据操作顺序为左至右，从上到下
	
	  
	   SSD1963Command_8(0x29); ////display on
	     
	 
	   SSD1963Command_8(0x2a);  //Set the column address of frame buffer
	   SSD1963Data_8(0x00);
	   SSD1963Data_8(0x00);
	   SSD1963Data_8(0x1);
	   SSD1963Data_8(0x67);// 
	
	   //row start_end
	   SSD1963Command_8(0x2b);   //Set the page address of the frame
	   SSD1963Data_8(0x00);
	   SSD1963Data_8(0x00);
	   SSD1963Data_8(0x02);
	   SSD1963Data_8(0x7f);// 
	}
	} 
	SSD1963Command_8(0xB8);  //Set the GPIOs configuration. If the GPIOs are not used for LCD, set the direction. Otherwise, they are toggled with  LCD signals by 0xC0 – 0xCF.
	    SSD1963Data_8(0x0F);     //
	    SSD1963Data_8(0x01);     //
	  
	   SSD1963Command_8(0xba);  //Set GPIO value for GPIO configured as output,4个GPIO全部输出1
	   SSD1963Data_8(0x0e);
	   Delay(5);   
	   
	   SSD1963Command_8(0xba);  //Set GPIO value for GPIO configured as output,4个GPIO全部输出1
	   SSD1963Data_8(0x0F); 
	   Delay(20);

////////////=============================================================================
////////////在SSD1963的I/O工作正常后可以对SSD2825进行配置。以便后续对IC进行初始化配置
	
	//对HX8369 IC进行初始化
	SSD1963INITSSD2825(); 

	SSD2825INITIAL_HX8369B();
///////////此处对SSD2825重新配置为VIDEO模式，一定要在SSD1963时序生成OK后进行配置2825，
	VIDEO_ON();

}

void SSD1963Command_8(u8 cmd)
{            LCD_RS(0);
             SSD1963_CS(0); //asm("NOP");asm("NOP");
             
             LCD_WR(0);//_WR=0; 
             DATAOUT(cmd);//asm("NOP"); //P1=cmd;
             LCD_WR(1);//_WR=1; 
             SSD1963_CS(1);//CSB=1;
             
}     

void SSD1963Data_8(u8 sdata)
{      LCD_RS(1);
             SSD1963_CS(0);//asm("NOP");asm("NOP");
             //DCX=0; 
             LCD_WR(0);//_WR=0; 
             DATAOUT(sdata);//asm("NOP"); //P1=cmd;
             LCD_WR(1);//_WR=1; 
             SSD1963_CS(1);//CSB=1;
}
//--------------------------------------------------------------------------------

void SSD1963GPIOOUTCOM (u8 instru_H)
{ 
	u8 i=0;
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x05);    //SDI=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x01);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x05);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_H & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_H=instru_H<<1;
	} 
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0F);
  	
}


void SSD1963GPIOOUTDATA_16 (u16 instru)  
{ 
	u8 i=0;
	u8 instru_H,instru_L;

	instru_L=instru;
	instru_H=instru>>8;

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_L & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_L=instru_L<<1;
	} 

	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_H & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_H=instru_H<<1;
	}
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0F);
  	
}

void SSD1963toSSD2825DATA_8BIT(u8 data)
{
	u8 i=0;

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(data & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		data=data<<1;
	} 
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0F);		//CS  = 1	
}

void SSD1963GPIOOUTDATA_8 (u8 instru_L,u8 instru_H)  
{ 
	u8 i=0;

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_L & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_L=instru_L<<1;
	} 

	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_H & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_H=instru_H<<1;
	}
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0F);
  	
}

void SSD1963GPIOOUTDATA_18 (u8 instru_L)  		 //通过SSD1963的GPIO 以三线串行方式送数据
{ 
	u8 i=0;

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x03);    //sck=0
	
	SSD1963Command_8(0xBa);  
	SSD1963Data_8(0x07);     //sck=1
	
	for (i=0;i<8;i++)     // 
	{ 
		if( !(instru_L & 0x80))
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x05);    //SDI=0
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x01);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x05);     //sck=1
		}
		else
		{ 
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x07);    //SDI=1
			SSD1963Command_8(0xBa); 
			SSD1963Data_8(0x03);    //sck=0
			
			SSD1963Command_8(0xBa);  
			SSD1963Data_8(0x07);     //sck=1
		}
		
		instru_L=instru_L<<1;
	} 
	
	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0F);
  	
}

void SPISDASetInput(void)
{
	GPIO_InitTypeDef QGPIO_InitStructure;
	QGPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	QGPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ; //GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &QGPIO_InitStructure);
}
void SPISDASetOutput(void)
{
	GPIO_InitTypeDef QGPIO_InitStructure;
	QGPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	QGPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	QGPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  		//OUTPUT
	GPIO_Init(GPIOA, &QGPIO_InitStructure);
}

u8 SSD2825_READ(u8 CMD)
{
	u8 i;
	u8 mm_L,mm_H;
	u8 Parameter=0xfa;
		
	mm = 0;

	SPISDASetInput();
	SPISDASetInput();

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x07);   //CS=0
	
	SSD1963Command_8(0xBa); 
    SSD1963Data_8(0x05);    //SDI=0
    SSD1963Command_8(0xBa); 
    SSD1963Data_8(0x01);    //sck=0

    SSD1963Command_8(0xBa);  
    SSD1963Data_8(0x05);     //sck=1

	for (i=0;i<8;i++)     // 
         { 
          if( !(CMD & 0x80))
              { 
			  	SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x05);    //SDI=0
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x01);    //sck=0
  
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x05);     //sck=1
              }
          else
          	  { 
			  	SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x07);    //SDI=1
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x03);    //sck=0
  
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x07);     //sck=1
               }
                
          CMD=CMD<<1;
         }	

	SSD1963Command_8(0xBa); 
    SSD1963Data_8(0x05);    //SDI=0
    SSD1963Command_8(0xBa); 
    SSD1963Data_8(0x01);    //sck=0

    SSD1963Command_8(0xBa);  
    SSD1963Data_8(0x05);     //sck=1
		 
	for (i=0;i<8;i++)     // 
         { 
          if( !(Parameter & 0x80))
              { 
			  	SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x05);    //SDI=0
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x01);    //sck=0
  
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x05);     //sck=1
              }
          else
          	  { 
			  	SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x07);    //SDI=1
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x03);    //sck=0
  
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x07);     //sck=1
               }
                
          Parameter=Parameter<<1;
         }
		 
	for (i=0;i<8;i++)     // 
         { 
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x01);    //sck=0
				Delay(1);
				mm_L<<=1;
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x05);     //sck=1
				
				mm_L=mm_L+GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
				Delay(1);
         }
	for (i=0;i<8;i++)     // 
         { 
                SSD1963Command_8(0xBa); 
                SSD1963Data_8(0x01);    //sck=0
				Delay(1);
				mm_H<<=1;
                SSD1963Command_8(0xBa);  
                SSD1963Data_8(0x05);     //sck=1
				
				mm_H=mm_H+GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
				Delay(1);
         }
	mm = mm_H;
	mm = mm_L + (mm<<8);

	SSD1963Command_8(0xBa); 
	SSD1963Data_8(0x0f);   //CS=1

	SPISDASetOutput();
	SPISDASetOutput();

	return mm;		 	 		 	
}
/*******************************************************************************
* Function Name  : READ_IC
* Description    : read data from IC register...
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 READ_IC(u16 cmd)
{

}

/*******************************************************************************
* Function Name  : ENTER_LP_mode()
* Description    : set MIPI lane enter LP mode...
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENTER_LP_mode(void)
{
	
	SSD1963Command_8(0xb9); //PLL disable 
	SSD1963GPIOOUTDATA_16(0x0000);
	SSD1963GPIOOUTDATA_18(0xba); //PLL setting 
	SSD1963GPIOOUTDATA_16(0x400a);
	SSD1963GPIOOUTDATA_18(0xbb); //LP clock divider 
	SSD1963GPIOOUTDATA_16(0x0002);
	SSD1963GPIOOUTDATA_18(0xb9); //PLL enable 
	SSD1963GPIOOUTDATA_16(0x4001); //4分频，SYS_CLK输出24/4=6MHZ
	Delay(1);
	SSD1963Command_8(0xb7); //DCS mode, LP mode
	SSD1963GPIOOUTDATA_16(0x0252); //short packet 
}

/*******************************************************************************
* Function Name  : ENTER_READ_mode()
* Description    : set MIPI lane D0   read mode...
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENTER_READ_mode(void)
{
	SSD1963Command_8(0xb7);
//	SSD1963GPIOOUTDATA_16(0x02c2); //////enter read mode      for generic read
	SSD1963GPIOOUTDATA_16(0x02d2);
}

void SSD1963INITSSD2825(void)		//通过SSD1963的GPIO以三线串行方式8bit对SSD2825进行初始化
{    
	u16 VHPW,VHBP,VHFP;
	VHPW=(VPW<<8)+HPW;
	VHBP=(VBP<<8)+HBP;
	VHFP=(VFP<<8)+HFP;


		SSD1963GPIOOUTCOM(0xc0);
		SSD1963GPIOOUTDATA_16(0x0100);
		Delay(20);

			SSD1963GPIOOUTCOM(0xB1);
		SSD1963GPIOOUTDATA_16(VHPW);		//Vertical sync and horizontal sync active period 

			SSD1963GPIOOUTCOM(0xB2);
		SSD1963GPIOOUTDATA_16(VHBP);		//Vertical and horizontal back porch period 

			SSD1963GPIOOUTCOM(0xB3);
		SSD1963GPIOOUTDATA_16(VHFP);		//Vertical and horizontal front porch period 

			SSD1963GPIOOUTCOM(0xB4);
		SSD1963GPIOOUTDATA_16(HDP);		//Horizontal active period 

			SSD1963GPIOOUTCOM(0xB5);
		SSD1963GPIOOUTDATA_16(VDP);		//Vertical active period 

			SSD1963GPIOOUTCOM(0xB6);		//Video mode and video pixel format
		SSD1963GPIOOUTDATA_16(0x0007);		//24bit 

			SSD1963GPIOOUTCOM(0xDE);		
		SSD1963GPIOOUTDATA_16(0x0001);		//MIPI lane select  

			SSD1963GPIOOUTCOM(0xd6);	
		SSD1963GPIOOUTDATA_16(0x0001);		//Color order and endianess 

			SSD1963GPIOOUTCOM(0xb9);		//PLL disable 
		SSD1963GPIOOUTDATA_16(0x0000);
		Delay(1);
			
			SSD1963GPIOOUTCOM(0xba);		//PLL setting 
		SSD1963GPIOOUTDATA_16(0x400a);			

			SSD1963GPIOOUTCOM(0xbb);		//LP clock divider 
		SSD1963GPIOOUTDATA_16(0x0003);			

			SSD1963GPIOOUTCOM(0xb9);		//PLL enable 
		SSD1963GPIOOUTDATA_16(0x4001);
		Delay(1);

			SSD1963GPIOOUTCOM(0xb8);		//VC register 
		SSD1963GPIOOUTDATA_16(0x0000);

			SSD1963GPIOOUTCOM(0xb7);		//Generic mode, HS video mode
		SSD1963GPIOOUTDATA_16(0x0702);		//    			generic		long packet

		Delay(5);

}

void VIDEO_ON(void)
{
		u16 VHPW,VHBP,VHFP;
	VHPW=(VPW<<8)+HPW;
	VHBP=(VBP<<8)+HBP;
	VHFP=(VFP<<8)+HFP;

			SSD1963GPIOOUTCOM(0xB1);
		SSD1963GPIOOUTDATA_16(VHPW);		//Vertical sync and horizontal sync active period 

			SSD1963GPIOOUTCOM(0xB2);
		SSD1963GPIOOUTDATA_16(VHBP);		//Vertical and horizontal back porch period 

			SSD1963GPIOOUTCOM(0xB3);
		SSD1963GPIOOUTDATA_16(VHFP);		//Vertical and horizontal front porch period 

			SSD1963GPIOOUTCOM(0xB4);
		SSD1963GPIOOUTDATA_16(HDP);		//Horizontal active period 

			SSD1963GPIOOUTCOM(0xB5);
		SSD1963GPIOOUTDATA_16(VDP);		//Vertical active period 

			SSD1963GPIOOUTCOM(0xB6);		//Video mode and video pixel format
		SSD1963GPIOOUTDATA_16(0x0007);		//24bit 

			SSD1963GPIOOUTCOM(0xDE);		
		SSD1963GPIOOUTDATA_16(0x0001);		//MIPI lane select  

			SSD1963GPIOOUTCOM(0xd6);	
		SSD1963GPIOOUTDATA_16(0x0001);		//Color order and endianess 
		
		
		SSD1963GPIOOUTCOM(0xb9);		//PLL disable 
		SSD1963GPIOOUTDATA_16(0x0000);
		Delay(1);
		
		SSD1963GPIOOUTCOM(0xba);		//PLL setting 
		SSD1963GPIOOUTDATA_16(0x8014);			//for SSD2828	 DEBUG	   0x8014

			SSD1963GPIOOUTCOM(0xbb);		//LP clock divider 
		SSD1963GPIOOUTDATA_16(0x0002);			

			SSD1963GPIOOUTCOM(0xb9);		//PLL enable 
		SSD1963GPIOOUTDATA_16(0x4001);
		Delay(1);

			SSD1963GPIOOUTCOM(0xb8);		//VC register 
		SSD1963GPIOOUTDATA_16(0x0000);

			SSD1963GPIOOUTCOM(0xb7);		//Generic mode, HS video mode 
		SSD1963GPIOOUTDATA_16(0x0709);
		
		Delay(5);
}


/////////////GPM1316B0  USE/////
void HX8369_cmd_4(u8 cmd,u8 num,u8 data,u8 data2)
{
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0004);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(cmd);
	SSD1963GPIOOUTDATA_18(num);
	SSD1963GPIOOUTDATA_18(data);	  //
	SSD1963GPIOOUTDATA_18(data2);
}

void SSD2825INITIAL_HX8369B(void)	//通过SSD1963的GPIO以三线串行方式8bit把数据送给SSD2825，通过SSD2825对模组进行初始化
{    
	///////FOR GPM1316B0 USE
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0004);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xB9);
	SSD1963GPIOOUTDATA_18(0xff);	  //
	SSD1963GPIOOUTDATA_18(0x83);	  //
	SSD1963GPIOOUTDATA_18(0x69);

	/////////MIPI command
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x000F);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xBA);	  //
	SSD1963GPIOOUTDATA_18(0x31);	  //
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x16);
	SSD1963GPIOOUTDATA_18(0xCA);
	SSD1963GPIOOUTDATA_18(0xB1);
	SSD1963GPIOOUTDATA_18(0x0A);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x28);
	SSD1963GPIOOUTDATA_18(0x02);
	SSD1963GPIOOUTDATA_18(0x21);
	SSD1963GPIOOUTDATA_18(0x21);
	SSD1963GPIOOUTDATA_18(0x9A);
	SSD1963GPIOOUTDATA_18(0x1A);
	SSD1963GPIOOUTDATA_18(0x8F);
	
	//////////GOA Timing Control
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x005D);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xD5);	  //
	SSD1963GPIOOUTDATA_18(0x00);	//1  //
	SSD1963GPIOOUTDATA_18(0x00);	//2
	SSD1963GPIOOUTDATA_18(0x0F);	//3
	SSD1963GPIOOUTDATA_18(0x03);	//4
	SSD1963GPIOOUTDATA_18(0x36);	//5
	SSD1963GPIOOUTDATA_18(0x00);	//6
	SSD1963GPIOOUTDATA_18(0x00);	//7
	SSD1963GPIOOUTDATA_18(0x10);	//8
	SSD1963GPIOOUTDATA_18(0x01);	//9
	SSD1963GPIOOUTDATA_18(0x00);	//10
	SSD1963GPIOOUTDATA_18(0x00);	//11
	SSD1963GPIOOUTDATA_18(0x00);	//12
	SSD1963GPIOOUTDATA_18(0x1A);	//13
	SSD1963GPIOOUTDATA_18(0x50);	//14
	SSD1963GPIOOUTDATA_18(0x45);	//15
	SSD1963GPIOOUTDATA_18(0x00);	//16
	SSD1963GPIOOUTDATA_18(0x00);	//17
	SSD1963GPIOOUTDATA_18(0x13);	//18
	SSD1963GPIOOUTDATA_18(0x44);	//19
	SSD1963GPIOOUTDATA_18(0x39);	//20
	SSD1963GPIOOUTDATA_18(0x47);	//21
	SSD1963GPIOOUTDATA_18(0x00);	//22
	SSD1963GPIOOUTDATA_18(0x00);	//23
	SSD1963GPIOOUTDATA_18(0x02);	//24
	SSD1963GPIOOUTDATA_18(0x04);	//25
	SSD1963GPIOOUTDATA_18(0x00);	//26
	SSD1963GPIOOUTDATA_18(0x00);	//27
	SSD1963GPIOOUTDATA_18(0x00);	//28
	SSD1963GPIOOUTDATA_18(0x00);	//29
	SSD1963GPIOOUTDATA_18(0x00);	//30
	SSD1963GPIOOUTDATA_18(0x00);	//31
	SSD1963GPIOOUTDATA_18(0x00);	//32
	SSD1963GPIOOUTDATA_18(0x03);	//33
	SSD1963GPIOOUTDATA_18(0x00);	//34
	SSD1963GPIOOUTDATA_18(0x00);	//35
	SSD1963GPIOOUTDATA_18(0x08);	//36
	SSD1963GPIOOUTDATA_18(0x88);	//37
	SSD1963GPIOOUTDATA_18(0x88);	//38
	SSD1963GPIOOUTDATA_18(0x37);	//39
	SSD1963GPIOOUTDATA_18(0x5F);	//40
	SSD1963GPIOOUTDATA_18(0x1E);	//41
	SSD1963GPIOOUTDATA_18(0x18);	//42
	SSD1963GPIOOUTDATA_18(0x88);	//43
	SSD1963GPIOOUTDATA_18(0x88);	//44
	SSD1963GPIOOUTDATA_18(0x85);	//45
	SSD1963GPIOOUTDATA_18(0x88);	//46
	SSD1963GPIOOUTDATA_18(0x88);	//47
	SSD1963GPIOOUTDATA_18(0x40);	//48
	SSD1963GPIOOUTDATA_18(0x2F);	//49
	SSD1963GPIOOUTDATA_18(0x6E);	//50
	SSD1963GPIOOUTDATA_18(0x48);	//51
	SSD1963GPIOOUTDATA_18(0x88);	//52
	SSD1963GPIOOUTDATA_18(0x88);	//53
	SSD1963GPIOOUTDATA_18(0x80);	//54
	SSD1963GPIOOUTDATA_18(0x88);	//55
	SSD1963GPIOOUTDATA_18(0x88);	//56
	SSD1963GPIOOUTDATA_18(0x26);	//57
	SSD1963GPIOOUTDATA_18(0x4F);	//58
	SSD1963GPIOOUTDATA_18(0x0E);	//59
	SSD1963GPIOOUTDATA_18(0x08);	//60
	SSD1963GPIOOUTDATA_18(0x88);	//61
	SSD1963GPIOOUTDATA_18(0x88);	//62
	SSD1963GPIOOUTDATA_18(0x84);	//63
	SSD1963GPIOOUTDATA_18(0x88);	//64
	SSD1963GPIOOUTDATA_18(0x88);	//65
	SSD1963GPIOOUTDATA_18(0x51);	//66
	SSD1963GPIOOUTDATA_18(0x3F);	//67
	SSD1963GPIOOUTDATA_18(0x7E);	//68
	SSD1963GPIOOUTDATA_18(0x58);	//69
	SSD1963GPIOOUTDATA_18(0x88);	//70
	SSD1963GPIOOUTDATA_18(0x88);	//71
	SSD1963GPIOOUTDATA_18(0x81);	//72
	SSD1963GPIOOUTDATA_18(0x00);	//73
	SSD1963GPIOOUTDATA_18(0x00);	//74
	SSD1963GPIOOUTDATA_18(0x00);	//75
	SSD1963GPIOOUTDATA_18(0x01);	//76
	SSD1963GPIOOUTDATA_18(0x00);	//77
	SSD1963GPIOOUTDATA_18(0x00);	//78
	SSD1963GPIOOUTDATA_18(0x00);	//79
	SSD1963GPIOOUTDATA_18(0x07);	//80
	SSD1963GPIOOUTDATA_18(0xF8);	//81
	SSD1963GPIOOUTDATA_18(0x0F);	//82
	SSD1963GPIOOUTDATA_18(0xFF);	//83
	SSD1963GPIOOUTDATA_18(0xFF);	//84
	SSD1963GPIOOUTDATA_18(0x07);	//85
	SSD1963GPIOOUTDATA_18(0xF8);	//86
	SSD1963GPIOOUTDATA_18(0x0F);	//87
	SSD1963GPIOOUTDATA_18(0xFF);	//88
	SSD1963GPIOOUTDATA_18(0xFF);	//89
	SSD1963GPIOOUTDATA_18(0x00);	//90
	SSD1963GPIOOUTDATA_18(0x00);	//91
	SSD1963GPIOOUTDATA_18(0x5A);	//92
	
	////////SET Pixel 
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0002);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0x3A);	  //
	SSD1963GPIOOUTDATA_18(0x70);	  //

	////////Display Direction
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0002);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0x36);	  //
	SSD1963GPIOOUTDATA_18(0xc0);	  //
	
	////////BGP voltage
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0003);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xB5);	  //
	SSD1963GPIOOUTDATA_18(0x12);	  //
	SSD1963GPIOOUTDATA_18(0x12);
	
	/////////////Power Control
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x000b);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xB1);	  //
	SSD1963GPIOOUTDATA_18(0x12);	  //
	SSD1963GPIOOUTDATA_18(0x83);
	SSD1963GPIOOUTDATA_18(0x77);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x12);
	SSD1963GPIOOUTDATA_18(0x12);
	SSD1963GPIOOUTDATA_18(0x1a);	//1E
	SSD1963GPIOOUTDATA_18(0x1a);	//1E
	SSD1963GPIOOUTDATA_18(0x0c);
	SSD1963GPIOOUTDATA_18(0x1a);
	
	////////RGB Setting
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0005);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xB3);	  //
	SSD1963GPIOOUTDATA_18(0x83);	  //
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x3a);
	SSD1963GPIOOUTDATA_18(0x17);
	
	////////Display Inversion Setting
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0002);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xB4);	  //
	SSD1963GPIOOUTDATA_18(0x00);	  // 01 DOT INVERSION     00 COLUMN INVERSION

	////////SET VCOM
//			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
//	SSD1963GPIOOUTDATA_16(0x0004);						  //接下来写入的数据
//			SSD1963GPIOOUTCOM(0xbf);
//	SSD1963GPIOOUTDATA_18(0xB6);	  //
//	SSD1963GPIOOUTDATA_18(0xbA);	//BD					//////because have auto otp function....so do not down this code..
//	SSD1963GPIOOUTDATA_18(0xbA);	//BC
//	SSD1963GPIOOUTDATA_18(0x00);
	
	////////SET EQ
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0005);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xe3);	  //
	SSD1963GPIOOUTDATA_18(0x0f);
	SSD1963GPIOOUTDATA_18(0x0f);
	SSD1963GPIOOUTDATA_18(0x0f);
	SSD1963GPIOOUTDATA_18(0x0f);
	
	////////SET SOURCE option
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0007);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xc0);	  //
	SSD1963GPIOOUTDATA_18(0x73);
	SSD1963GPIOOUTDATA_18(0x50);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x34);
	SSD1963GPIOOUTDATA_18(0xc4);
	SSD1963GPIOOUTDATA_18(0x00);
	
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x007F);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xc1);	  //
	SSD1963GPIOOUTDATA_18(0x01);	//1
	SSD1963GPIOOUTDATA_18(0x00);	//2
	SSD1963GPIOOUTDATA_18(0x0a);	//3
	SSD1963GPIOOUTDATA_18(0x15);	//4
	SSD1963GPIOOUTDATA_18(0x1f);	//5
	SSD1963GPIOOUTDATA_18(0x2a);	//6
	SSD1963GPIOOUTDATA_18(0x35);	//7
	SSD1963GPIOOUTDATA_18(0x42);	//8
	SSD1963GPIOOUTDATA_18(0x4c);	//9
	SSD1963GPIOOUTDATA_18(0x57);	//10
	SSD1963GPIOOUTDATA_18(0x60);	//11
	SSD1963GPIOOUTDATA_18(0x6a);	//12
	SSD1963GPIOOUTDATA_18(0x73);	//13
	SSD1963GPIOOUTDATA_18(0x7b);	//14
	SSD1963GPIOOUTDATA_18(0x83);	//15
	SSD1963GPIOOUTDATA_18(0x8a);	//16
	SSD1963GPIOOUTDATA_18(0x91);	//17
	SSD1963GPIOOUTDATA_18(0x99);	//18
	SSD1963GPIOOUTDATA_18(0xa0);	//19
	SSD1963GPIOOUTDATA_18(0xa7);	//20
	SSD1963GPIOOUTDATA_18(0xad);	//21
	SSD1963GPIOOUTDATA_18(0xB4);	//22
	SSD1963GPIOOUTDATA_18(0xBA);	//23
	SSD1963GPIOOUTDATA_18(0xC0);	//24
	SSD1963GPIOOUTDATA_18(0xC6);	//25
	SSD1963GPIOOUTDATA_18(0xCC);	//26
	SSD1963GPIOOUTDATA_18(0xD2);	//27
	SSD1963GPIOOUTDATA_18(0xD8);	//28
	SSD1963GPIOOUTDATA_18(0xDF);	//29
	SSD1963GPIOOUTDATA_18(0xE6);	//30
	SSD1963GPIOOUTDATA_18(0xEC);	//31
	SSD1963GPIOOUTDATA_18(0xF3);	//32
	SSD1963GPIOOUTDATA_18(0xF8);	//33
	SSD1963GPIOOUTDATA_18(0xFF);	//34
	SSD1963GPIOOUTDATA_18(0x22);	//35
	SSD1963GPIOOUTDATA_18(0xE0);	//36
	SSD1963GPIOOUTDATA_18(0x30);	//37
	SSD1963GPIOOUTDATA_18(0x4B);	//38
	SSD1963GPIOOUTDATA_18(0x43);	//39
	SSD1963GPIOOUTDATA_18(0x01);	//40
	SSD1963GPIOOUTDATA_18(0xAF);	//41
	SSD1963GPIOOUTDATA_18(0x0A);	//42
	SSD1963GPIOOUTDATA_18(0xC0);	//43
	SSD1963GPIOOUTDATA_18(0x00);	//44
	SSD1963GPIOOUTDATA_18(0x09);	//45
	SSD1963GPIOOUTDATA_18(0x13);	//46
	SSD1963GPIOOUTDATA_18(0x1C);	//47
	SSD1963GPIOOUTDATA_18(0x26);	//48
	SSD1963GPIOOUTDATA_18(0x2F);	//49
	SSD1963GPIOOUTDATA_18(0x37);	//50
	SSD1963GPIOOUTDATA_18(0x41);	//51
	SSD1963GPIOOUTDATA_18(0x49);	//52
	SSD1963GPIOOUTDATA_18(0x52);	//53
	SSD1963GPIOOUTDATA_18(0x5B);	//54
	SSD1963GPIOOUTDATA_18(0x64);	//55
	SSD1963GPIOOUTDATA_18(0x6C);	//56
	SSD1963GPIOOUTDATA_18(0x74);	//57
	SSD1963GPIOOUTDATA_18(0x7C);	//58
	SSD1963GPIOOUTDATA_18(0x84);	//59
	SSD1963GPIOOUTDATA_18(0x8B);	//60
	SSD1963GPIOOUTDATA_18(0x92);	//61
	SSD1963GPIOOUTDATA_18(0x9B);	//62
	SSD1963GPIOOUTDATA_18(0xA2);	//63
	SSD1963GPIOOUTDATA_18(0xA9);	//64
	SSD1963GPIOOUTDATA_18(0xB0);	//65
	SSD1963GPIOOUTDATA_18(0xB7);	//66
	SSD1963GPIOOUTDATA_18(0xBD);	//67
	SSD1963GPIOOUTDATA_18(0xC3);	//68
	SSD1963GPIOOUTDATA_18(0xCA);	//69
	SSD1963GPIOOUTDATA_18(0xD1);	//70
	SSD1963GPIOOUTDATA_18(0xD8);	//71
	SSD1963GPIOOUTDATA_18(0xE0);	//72
	SSD1963GPIOOUTDATA_18(0xE8);	//73
	SSD1963GPIOOUTDATA_18(0xEF);	//74
	SSD1963GPIOOUTDATA_18(0xF8);	//75
	SSD1963GPIOOUTDATA_18(0xFF);	//76
	SSD1963GPIOOUTDATA_18(0x22);	//77
	SSD1963GPIOOUTDATA_18(0x58);	//78
	SSD1963GPIOOUTDATA_18(0xBD);	//79
	SSD1963GPIOOUTDATA_18(0xF4);	//80
	SSD1963GPIOOUTDATA_18(0x5A);	//81
	SSD1963GPIOOUTDATA_18(0xA5);	//82
	SSD1963GPIOOUTDATA_18(0xFA);	//83
	SSD1963GPIOOUTDATA_18(0x4C);	//84
	SSD1963GPIOOUTDATA_18(0xC0);	//85
	SSD1963GPIOOUTDATA_18(0x00);	//86
	SSD1963GPIOOUTDATA_18(0x07);	//87
	SSD1963GPIOOUTDATA_18(0x0F);	//88
	SSD1963GPIOOUTDATA_18(0x16);	//89
	SSD1963GPIOOUTDATA_18(0x1E);	//90
	SSD1963GPIOOUTDATA_18(0x25);	//91
	SSD1963GPIOOUTDATA_18(0x2D);	//92
	SSD1963GPIOOUTDATA_18(0x34);	//93
	SSD1963GPIOOUTDATA_18(0x3C);	//94
	SSD1963GPIOOUTDATA_18(0x45);	//95
	SSD1963GPIOOUTDATA_18(0x4E);	//96
	SSD1963GPIOOUTDATA_18(0x57);	//97
	SSD1963GPIOOUTDATA_18(0x5F);	//98
	SSD1963GPIOOUTDATA_18(0x68);	//99
	SSD1963GPIOOUTDATA_18(0x70);	//100
	SSD1963GPIOOUTDATA_18(0x78);	//101
	SSD1963GPIOOUTDATA_18(0x80);	//102
	SSD1963GPIOOUTDATA_18(0x87);	//103
	SSD1963GPIOOUTDATA_18(0x8E);	//104
	SSD1963GPIOOUTDATA_18(0x96);	//105
	SSD1963GPIOOUTDATA_18(0x9D);	//106
	SSD1963GPIOOUTDATA_18(0xA4);	//107
	SSD1963GPIOOUTDATA_18(0xAB);	//108
	SSD1963GPIOOUTDATA_18(0xB3);	//109
	SSD1963GPIOOUTDATA_18(0xB9);	//110
	SSD1963GPIOOUTDATA_18(0xC0);	//111
	SSD1963GPIOOUTDATA_18(0xC8);	//112
	SSD1963GPIOOUTDATA_18(0xCF);	//113
	SSD1963GPIOOUTDATA_18(0xD7);	//114
	SSD1963GPIOOUTDATA_18(0xE1);	//115
	SSD1963GPIOOUTDATA_18(0xE9);	//116
	SSD1963GPIOOUTDATA_18(0xF4);	//117
	SSD1963GPIOOUTDATA_18(0xFF);	//118
	SSD1963GPIOOUTDATA_18(0x22);	//119
	SSD1963GPIOOUTDATA_18(0x22);	//120
	SSD1963GPIOOUTDATA_18(0x17);	//121
	SSD1963GPIOOUTDATA_18(0xC9);	//122
	SSD1963GPIOOUTDATA_18(0x1C);	//123
	SSD1963GPIOOUTDATA_18(0xAD);	//124
	SSD1963GPIOOUTDATA_18(0xF7);	//125
	SSD1963GPIOOUTDATA_18(0xC9);	//126
	
	////////SET SOURCE 
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0004);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xc6);	  //
	SSD1963GPIOOUTDATA_18(0x41);
	SSD1963GPIOOUTDATA_18(0xFF);
	SSD1963GPIOOUTDATA_18(0x7D);

	////////SET PANEL
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0002);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xcC);	  //
	SSD1963GPIOOUTDATA_18(0x00);	 //00

	////////SET MESSI
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0002);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xEA);	  //
	SSD1963GPIOOUTDATA_18(0x7A);
	
	////////Gamma Setting
	SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0024);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xE0);	  //
	SSD1963GPIOOUTDATA_18(0x00);	//1
	SSD1963GPIOOUTDATA_18(0x10);	//2
	SSD1963GPIOOUTDATA_18(0x18);	//3
	SSD1963GPIOOUTDATA_18(0x0E);	//4
	SSD1963GPIOOUTDATA_18(0x10);	//5
	SSD1963GPIOOUTDATA_18(0x3C);	//6
	SSD1963GPIOOUTDATA_18(0x26);	//7
	SSD1963GPIOOUTDATA_18(0x32);	//8
	SSD1963GPIOOUTDATA_18(0x07);	//9
	SSD1963GPIOOUTDATA_18(0x0C);	//10
	SSD1963GPIOOUTDATA_18(0x10);	//11
	SSD1963GPIOOUTDATA_18(0x13);	//12
	SSD1963GPIOOUTDATA_18(0x16);	//13
	SSD1963GPIOOUTDATA_18(0x14);	//14
	SSD1963GPIOOUTDATA_18(0x15);	//15
	SSD1963GPIOOUTDATA_18(0x13);	//16
	SSD1963GPIOOUTDATA_18(0x15);	//17
	SSD1963GPIOOUTDATA_18(0x00);	//18
	SSD1963GPIOOUTDATA_18(0x10);	//19
	SSD1963GPIOOUTDATA_18(0x18);	//20
	SSD1963GPIOOUTDATA_18(0x0E);	//21
	SSD1963GPIOOUTDATA_18(0x10);	//22
	SSD1963GPIOOUTDATA_18(0x3C);	//23
	SSD1963GPIOOUTDATA_18(0x26);	//24
	SSD1963GPIOOUTDATA_18(0x32);	//25
	SSD1963GPIOOUTDATA_18(0x07);	//26
	SSD1963GPIOOUTDATA_18(0x0C);	//27
	SSD1963GPIOOUTDATA_18(0x10);	//28
	SSD1963GPIOOUTDATA_18(0x13);	//29
	SSD1963GPIOOUTDATA_18(0x16);	//30
	SSD1963GPIOOUTDATA_18(0x14);	//31
	SSD1963GPIOOUTDATA_18(0x15);	//32
	SSD1963GPIOOUTDATA_18(0x13);	//33
	SSD1963GPIOOUTDATA_18(0x15);	//34
	SSD1963GPIOOUTDATA_18(0x01);	//35

	////////sleep out
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0001);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0x11);	  //
	Delay(20); 			  ////delay 200ms
	
	////////display on
			SSD1963GPIOOUTCOM(0xBC);		                  // Packet Size Control Register
	SSD1963GPIOOUTDATA_16(0x0001);						  //接下来写入的数据
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0x29);	  //
	Delay(5); 			   ///delay 50ms
}




void enter_sleep_mode(void)
{

////////ic  display off  and  in sleep mode  setting

	SSD1963GPIOOUTCOM(0xBC);
	SSD1963GPIOOUTDATA_8(0x01,0x00);
			SSD1963GPIOOUTCOM(0xbf); 
	SSD1963GPIOOUTDATA_8(0x28,0x00);
	Delay(1);


	SSD1963GPIOOUTCOM(0xBC);
	SSD1963GPIOOUTDATA_8(0x01,0x00);
			SSD1963GPIOOUTCOM(0xbf); 
	SSD1963GPIOOUTDATA_8(0x10,0x00);
	Delay(5);

	 SSD1963GPIOOUTCOM(0xBC);				 	//soft reset  need mini 5ms
	SSD1963GPIOOUTDATA_8(0x01,0x00);
			SSD1963GPIOOUTCOM(0xbf); 
	SSD1963GPIOOUTDATA_8(0x01,0x00);
	Delay(20); 


  //2825 CHANGE MODE  IN SLEEP
	SSD1963GPIOOUTCOM(0xc0);		  //SSD2825 SOFT RESET
		SSD1963GPIOOUTDATA_16(0x0100);
		Delay(20);

			SSD1963GPIOOUTCOM(0xB6);		//Video mode and video pixel format
		SSD1963GPIOOUTDATA_16(0x0006);		//18bit 

			SSD1963GPIOOUTCOM(0xDE);		
		SSD1963GPIOOUTDATA_16(0x0001);		//MIPI lane select  

			SSD1963GPIOOUTCOM(0xd6);	
		SSD1963GPIOOUTDATA_16(0x0000);		//Color order and endianess 

			SSD1963GPIOOUTCOM(0xb7);	
		SSD1963GPIOOUTDATA_16(0x0342);		//DCS mode 

			SSD1963GPIOOUTCOM(0xb8);		//VC register 
		SSD1963GPIOOUTDATA_16(0x0080);

		SSD1963GPIOOUTCOM(0xb9);		//PLL disable 
		SSD1963GPIOOUTDATA_16(0x0000);

			SSD1963GPIOOUTCOM(0xba);		//PLL setting 
		SSD1963GPIOOUTDATA_16(0x400A);				//Fout=(Fin/MS)*NF,MS=0,NF=8,Fout=(12/1)*8=96MHZ

			SSD1963GPIOOUTCOM(0xbb);		//LP clock divider 
		SSD1963GPIOOUTDATA_16(0x0001);				//LP clock=Fout/LPD/8,LPD=2,LP clock=96/2/8=6MHZ

			SSD1963GPIOOUTCOM(0xb9);		//PLL enable 
		SSD1963GPIOOUTDATA_16(0x0001);

			SSD1963GPIOOUTCOM(0xb8);		//VC register 
		SSD1963GPIOOUTDATA_16(0x0000);

			SSD1963GPIOOUTCOM(0xb7);		//Generic mode, HS video mode
		SSD1963GPIOOUTDATA_16(0x0314);		//

		Delay(20);

		//ssd1963 into sleep mode

		SSD1963Command_8(0x01);
	   Delay(20);

		SSD1963Command_8(0x10);

	   Delay(2);
	   SSD1963Command_8(0xe5); 

}

void exit_sleep_mode(void)
{	
		STM32_SSD1963_Init();
}

/*******************************************************************************
* Function Name  : LCD_SetDisplayWindow
* Description    : Sets a display window
* Input          : - Xpos: specifies the X buttom left position.
*                  - Ypos: specifies the Y buttom left position.
*                  - Height: display window height.
*                  - Width: display window width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetDisplayWindow(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey)
{            //此函数非常重要
  u16 sx= add_sx;
  u16 ex= add_ex;
  u16 sy= add_sy;
  u16 ey= add_ey;
  
       SSD1963Command_8(0x2a); 
       SSD1963Data_8(sx>>8);
       SSD1963Data_8(sx);
       SSD1963Data_8(ex>>8);
       SSD1963Data_8(ex);// 

       //row start_end
       SSD1963Command_8(0x2b);  
       SSD1963Data_8(sy>>8);
       SSD1963Data_8(sy);
       SSD1963Data_8(ey>>8);
       SSD1963Data_8(ey);// 
}
/*******************************************************************************
* Function Name  : LCD_WriteRAM_Prepare
* Description    : Prepare to write to the LCD RAM.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{//此函数非常重要
  SSD1963Command_8(0x2c);
}
void LCD_DrawPoint(u32 point)  //向Display RAM 写入1点的数据，此函数与“LCD_WriteRAM_Prepare”配合使用
{ //此函数非常重要
	u8 rr,gg,bb;
	//u8 tep;
  rr=(point>>16);  
  gg=(point>>8);  
  bb=point;
  
	if(SDShowTimes == 1)
	{// SSD1963设定为24bits色彩输出，
		SSD1963Data_8(rr);SSD1963Data_8(gg);SSD1963Data_8(bb);
	}
	else
	{	;	}
}



void LCD_WriteArea(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey, u8 corr,u8 corg,u8 corb)
{
  u16 m,n;
  
  LCD_SetDisplayWindow(add_sx, add_ex, add_sy, add_ey);
  LCD_WriteRAM_Prepare();
 
  for(m = 0; m <(add_ex-add_sx+1); m++)
   {
  	  for(n = 0; n <(add_ey-add_sy+1); n++)
  	     {
	         SSD1963Data_8(corr);SSD1963Data_8(corg);SSD1963Data_8(corb);
	       }
   }
  
}

void Flicker_sub_pixel(void)
{
	u16 m,n;
  
  LCD_SetDisplayWindow(0, XDP-1, 0, YDP-1);
  LCD_WriteRAM_Prepare();
 
	for(m = 0; m <(YDP); m++)
	{
		for(n = 0; n <(XDP/2); n++)
		{
	         SSD1963Data_8(0X80);SSD1963Data_8(0X00);SSD1963Data_8(0X80);
			 SSD1963Data_8(0X00);SSD1963Data_8(0X80);SSD1963Data_8(0X00);
		}
		
	}	
}

void Flicker_PIXEL(void)
{
	u16 m,n;
  
  LCD_SetDisplayWindow(0, XDP-1, 0, YDP-1);
  LCD_WriteRAM_Prepare();
 
	for(m = 0; m <(YDP); m++)
	{
		for(n = 0; n <(XDP/2); n++)
		{
	         SSD1963Data_8(0X00);SSD1963Data_8(0X00);SSD1963Data_8(0X00);
			 SSD1963Data_8(0X80);SSD1963Data_8(0X80);SSD1963Data_8(0X80);
		}
		
	}
}

void Flicker_COLUMN(void)
{
	u16 m,n;
  
  LCD_SetDisplayWindow(0, XDP-1, 0, YDP-1);
  LCD_WriteRAM_Prepare();
 
	for(m = 0; m <(YDP); m++)
	{
		for(n = 0; n <(XDP/2); n++)
		{
	         SSD1963Data_8(0X80);SSD1963Data_8(0X80);SSD1963Data_8(0X80);
			 SSD1963Data_8(0X00);SSD1963Data_8(0X00);SSD1963Data_8(0X00);
		}

	}
}

void RGB_color(void)
{
	u16 m,n;
	u8 corr,corg,corb;

	
  
	LCD_SetDisplayWindow(0,(XDP-1),0,(YDP-1));
	LCD_WriteRAM_Prepare();

 	corr = corg = corb = 0;
	for(m = 0; m <(YDP); m++)
	{
		
  	  for(n = 0; n <XDP/4; n++)
  	     {
	         SSD1963Data_8(corr);SSD1963Data_8(0);SSD1963Data_8(0);
			 
	     }
		 corr = (m*255)/YDP;

		 for(n = 0; n <XDP/4; n++)
  	     {
	         SSD1963Data_8(0);SSD1963Data_8(corg);SSD1963Data_8(0);
			 
	     }
		 corg = (m*255)/YDP;

		 for(n = 0; n <XDP/4; n++)
  	     {
	         SSD1963Data_8(0);SSD1963Data_8(0);SSD1963Data_8(corb);
			 
	     }
		 corb = (m*255)/YDP;

		 for(n = 0; n <XDP/4; n++)
  	     {
	         SSD1963Data_8(corr);SSD1963Data_8(corg);SSD1963Data_8(corb);
			 
	     }

	}
}

void CT(void)
{
	u16 m,n;

	LCD_SetDisplayWindow(0,(XDP-1),0,(YDP-1));
	LCD_WriteRAM_Prepare();

	for(m = 0; m <(YDP); m++)
	{
		
  	  for(n = 0; n <XDP; n++)
  	     {
	         
			 if((270<m<530)&&(160<n<320))
			{
				SSD1963Data_8(0x00);SSD1963Data_8(0x00);SSD1963Data_8(0x00);	
			}
			else
			{
				SSD1963Data_8(0x80);SSD1963Data_8(0x80);SSD1963Data_8(0x80);	
			}
	     }
	
		 
	}	
}

void All_Color(u8 corr,u8 corg,u8 corb)
{
  vu32 index =0 ;
  LCD_SetDisplayWindow(0,(XDP-1),0,(YDP-1));
  LCD_WriteRAM_Prepare();
 
  for(index = 0; index < (YDP*XDP); index++)
  {
	 SSD1963Data_8(corr);SSD1963Data_8(corg);SSD1963Data_8(corb);
  }
}
void LCD_Clear(void)
{
  All_Color(0,128,255);
}
/*******************************************************************************
* Function Name  : LCD_WriteBMP
* Description    : Displays a bitmap picture loaded in the internal Flash.
* Input          : - BmpAddress: Bmp picture address in the internal Flash.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteBMP(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey, const u8 *bitmap)
{
	u16 m,n;
	u8 *bitmap_ptr = (u8 *)bitmap;
	u16 point;
  
	LCD_SetDisplayWindow(add_sx, add_ex, add_sy, add_ey);
	LCD_WriteRAM_Prepare();
 
	for(m = 0; m <(add_ex-add_sx+1); m++)
	{
		for(n = 0; n <(add_ey-add_sy+1); n++)
  	    {
	         point= *bitmap_ptr++;  
			 SSD1963Data_8(point);

			 point= *bitmap_ptr++;
			 SSD1963Data_8(point);

			 point= *bitmap_ptr++;
			 SSD1963Data_8(point);
		}
	}
}



  /****************************************************************************
* 名    称：LCD_PutChar(u16 adx,u16 ady,const uint  *p)
* 功    能：在指定座标显示一个16x24点阵的ascii字符
* 入口参数：adx          列座标
*           ady          行座标
* 说    明：显示范围限定为可显示的ascii码
****************************************************************************/
void LCD_PutChar(u16 adx,u16 ady,const uint  *p)
{ //此函数作用为写入16*24大小字符
    u8 i,j;       u16 temp;
    u16 sx=adx;   u16 sy=ady;     
           
    LCD_SetDisplayWindow(sx, sx+16-1,sy, sy+24-1)	; 
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    
	for(i=0;i<24;i++)  //字体高度：24 Dots
	{
	    temp=*p;
	    for(j=0;j<16;j++) ////字体宽度：16 Dots
	       {                 
	           if(temp&1) 
                 {  SSD1963Data_8(FontR);SSD1963Data_8(FontG);SSD1963Data_8(FontB);} //字体色
	           else
                  { SSD1963Data_8(255);SSD1963Data_8(255);SSD1963Data_8(255); }  //背景色    
	           temp>>=1;
	       }
      p++;
     }
}
/****************************************************************************
* 名    称：DisplayStringLine
* 功    能：显示n个字符一行在LCD上  ;字库大小为16*24，故n=(行像素个数/16)
* 入口参数：addx：起始X地址，addy：起始Y地址;  *ptr指向字符串的指针 
* 出口参数：无
* 说    明：
* 调用方法：LCD_DisplayStringLine(Line0,"I Love you...",White,Blue);  
****************************************************************************/
void LCD_DisplayStringLine(u16 addx,u16 addy, unsigned char *p )
{
  unsigned char tep;    
    const u16 *fp;
    while(*p!='\0')
    {       
      tep=*p;
      tep-=32;
      fp=&ASCII_Table[tep * 24];
      LCD_PutChar(addx,addy,fp);
        addx+=16;
        p++;
    }  
}

void LCD_PutChar_A(u16 adx,u8 times_X,u16 ady,u8 times_Y,const uint  *p)
{ 
    u8 Font_H,Font_W;
	u8 i,j,k,k1;       u16 temp,temp1;
    u16 sx=adx;   u16 sy=ady; 
  
	Font_H = times_Y;	Font_W = times_X;
   
    LCD_SetDisplayWindow(sx, sx+16*Font_W-1,sy, sy+24*Font_H-1)	; 
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    
	for(i=0;i<24;i++)  		   //////////字体高度：24 Dots
	{
	    temp=*p;
		temp1 = temp;
		for(k1=0;k1<Font_H;k1++)
		{
	    temp = temp1;
		for(j=0;j<16;j++) ///		////字体宽度：16 Dots
	       {                 
	           if(temp&1) 
                 {  								 
					for(k=0;k<Font_W;k++)
					{
						SSD1963Data_8(FontR);SSD1963Data_8(FontG);SSD1963Data_8(FontB);		 ////////字体色
					}
				 } 
	           else
                  { 
						for(k=0;k<Font_W;k++)
						{
							SSD1963Data_8(255);SSD1963Data_8(255);SSD1963Data_8(255);		/////////背景色
						}
				   }     
	           temp>>=1;
	       }
		}
      p++;
     }
}

void LCD_DisplayStringLine_A(u16 addx,u16 addy, unsigned char *p )
{
	u16 FontFlag; 		u8 fontH,fontW;
	unsigned char tep;    		const u16 *fp;

	FontFlag=1;		
	fontH = 3; 			  /////// height *3
	fontW = 3;			  ///////  width *3
    while((*p!='\0')&&(FontFlag<(XDP-16*fontW)))
    {       
      tep=*p;
      tep-=32;
      fp=&ASCII_Table[tep * 24];
      LCD_PutChar_A(addx,fontW,addy,fontH,fp);
        addx+=16*fontW;
        p++;
		FontFlag += 16*fontW;;
    }  
}
/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines (FSMC Pins) in alternate function
                   Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
                                             | RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//禁止jtag，以空出PB3,PB4,PA15
 
  
  /*[把GPIOB配置成输出模式] */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO最高速度50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*[把PA4678配置成输出模式] */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO最高速度50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
