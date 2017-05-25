/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : lcd.h
* Author             : MCD Application Team
* Version            : V1.1.2
* Date               : 09/22/2008
* Description        : 
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H	  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
 #define uint u16 
//-------------*******--LCD端口定义-------*********** ---------------

#define	SSD1963Pin_CS	  (1<<2)   //SSD1963片选端口  	         
#define	LCDPin_RST	(1<<0)   //硬复位		 
#define	LCDPin_RS	(1<<3)  //数据/命令选择端口	Pa.3
#define	LCDPin_RD	(1<<4)  // 
#define	LCDPin_WR	(1<<5)  // 
#define	LCDPin_CS	(1<<1)  //

#define LCD_RS(x)   GPIOA->ODR=(GPIOA->ODR&~LCDPin_RS)|(x ? LCDPin_RS:0)
#define SSD1963_CS(x)   GPIOA->ODR=(GPIOA->ODR&~SSD1963Pin_CS)|(x ? SSD1963Pin_CS:0)													    
#define LCD_RST(x)  GPIOA->ODR=(GPIOA->ODR&~LCDPin_RST)|(x ? LCDPin_RST:0)	
#define LCD_RD(x)   GPIOA->ODR=(GPIOA->ODR&~LCDPin_RD) |(x ? LCDPin_RD:0)
#define LCD_WR(x)   GPIOA->ODR=(GPIOA->ODR&~LCDPin_WR)|(x ? LCDPin_WR:0)
#define LCD_CS(x)   GPIOA->ODR=(GPIOA->ODR&~LCDPin_CS)|(x ? LCDPin_CS:0)		
	  
#define DATAOUT(x) GPIO_Write(GPIOB,(x))
/* Exported macro ------------------------------------------------------------*/
void SSD1963GPIOOUTDATA_16 (u16 instru);  //通过SSD1963的GPIO 以三线串行方式送数据
void SSD1963GPIOOUTCOM (u8 instru_H);  //通过SSD1963的GPIO 以三线串行方式送指令
void SSD1963GPIOOUTDATA_8 (u8 instru_L,u8 instru_H);  //通过SSD1963的GPIO 以三线串行方式送数据
void SSD1963GPIOOUTDATA_18 (u8 instru_L);
void HX8369_cmd_4(u8 cmd,u8 num,u8 data,u8 data2);
void enter_sleep_mode(void);
void exit_sleep_mode(void);
void RGB_color(void);


/*	----------------------------------------------------------------
			for GPM1316B0
	
---------------------------------------------------------------------*/
void SSD1963INITSSD2825(void);   //通过SSD1963的GPIO以三线串行方式8bit对SSD2825进行初始化
void SSD2825INITIAL35510(void);	 //通过SSD1963的GPIO以三线串行方式8bit把数据送给SSD2825，通过SSD2825对模组进行初始化
void VIDEO_ON(void);
void ENTER_LP_mode(void);
void ENTER_READ_mode(void);

void SSD1963toSSD2825DATA_8BIT(u8 data);
void SSD2825_INIT_LCD(u8 cmd,u8 num,u8 parameter[64]);
void STM32_SSD1963_Init(void);   // 初始化SSD1963;
void LCD_Clear(void);
void All_Color(u8 corr,u8 corg,u8 corb);
void LCD_WriteBMP(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey, const u8 *bitmap);
void LCD_WriteArea(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey, u8 corr,u8 corg,u8 corb);
void Flicker_PIXEL(void);
void Flicker_sub_pixel(void);
void Flicker_COLUMN(void);
void CT(void);
void SSD2825INITIAL_HX8369B(void);
/* Exported functions ------------------------------------------------------- */

/*----------------------------------------------------------------
			for SSD2825/2828    SPI  READ function
	
--------------------------------------------------------------------*/
void SPISDASetInput(void);
void SPISDASetOutput(void);
u8 SSD2825_READ(u8 CMD);
u8 READ_IC(u16 cmd);
/*----------------------------------------------------------------
--------------------------------------------------------------------*/





/*----- Medium layer function -----*/

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);

void SSD1963Data_8(u8 sdata);
void SSD1963Command_8(u8 cmd);

//---------------以下五个函数非常重要，务必保证正确存在----------------------

void LCD_SetDisplayWindow(u16 add_sx, u16 add_ex, u16 add_sy, u16 add_ey);
void LCD_DisplayStringLine(u16 addx,u16 addy, unsigned char *p );
void LCD_PutChar(u16 adx,u16 ady,const u16  *p);
void LCD_WriteRAM_Prepare(void);   
void LCD_DrawPoint(u32 point);
void Disp_BMP(void);
void LCD_DisplayStringLine_A(u16 addx,u16 addy, unsigned char *p );
void LCD_PutChar_A(u16 adx,u8 times_X,u16 ady,u8 times_Y,const uint  *p);

/* LCD Registers */
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


/* LCD color */
#define   Black         0x0000      		/* 黑色：    0,   0,   0 */
#define   BLUE          0x001F                	/* 蓝色：    0,   0, 255 */
#define   GREEN         0x07E0                	/* 绿色：    0, 255,   0 */
#define   CYAN          0x07FF                 	/* 青色：    0, 255, 255 */
#define   RED           0xF800                	/* 红色：  255,   0,   0 */
#define   MAGENTA       0xF81F                	/* 品红：  255,   0, 255 */
#define   YELLOW        0xFFE0                	/* 黄色：  255, 255, 0   */
#define   White         0xFFFF      		/* 白色：  255, 255, 255 */
#define   NAVY          0x000F      		/* 深蓝色：  0,   0, 128 */
#define   DGREEN        0x03E0               	/* 深绿色：  0, 128,   0 */
#define   DCYAN         0x03EF                	/* 深青色：  0, 128, 128 */
#define   MAROON        0x7800                	/* 深红色：128,   0,   0 */
#define   PURPLE        0x780F                	/* 紫色：  128,   0, 128 */
#define   OLIVE         0x7BE0             	/* 橄榄绿：128, 128,   0 */
#define   LGRAY         0xC618            	/* 灰白色：192, 192, 192 */
#define   DGRAY         0x7BEF                	/* 深灰色：128, 128, 128 */

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

#endif /* __LCD_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
