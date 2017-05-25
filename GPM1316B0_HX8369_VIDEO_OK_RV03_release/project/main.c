/******************** (C) COPYRIGHT 2010 GIANTPLUS ********************
* File Name          : main.c
* Author             : Jason
* Version            : V1.0.0
* Date               : 2011.01.20
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "platform_config.h"
#include <stdio.h>
#include "FAT.h"
#include "sdcard.h"
#include "tsl2583.h"
#include "lcd.h"

GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;
static vu32 TimingDelay = 0;
extern BYTE FAT32_Enable; 
extern WORD SectorsPerClust;
extern u16 image_count ;
extern u16 image_all ;
SD_Error Status = SD_OK;
SD_CardInfo SDCardInfo;
u32  sd_Capacity;
SD_Error SD_InitAndConfig(void);
void RCC_Configuration(void);
void InterruptConfig(void);
void NVIC_Configuration(void);
void Delay(vu32 nCount);
void Decrement_TimingDelay(void);
void SysTick_Config(void);
void USART_Configuration(void);
void USART1GPIO_Configuration(void); 
//----------------以上内容不需要修改，详情参阅readme.txt---------------

//----------------------------------------
void DelayKEY (u32 k); //按住，画面暂停
void KEYGPIO_Init(void);///*[把PA78配置成输入模式] */
#define KEYA7      GPIOA->IDR&GPIO_Pin_7
#define KEYA8      GPIOA->IDR&GPIO_Pin_8
#define KEYC6      GPIOC->IDR&GPIO_Pin_6//============================== *** =============================
#include "PICC1.h"   //该芯片内部Flash容量为512KB，可保存小幅图片数据


//for  SD use only
extern u8 FontR,FontG,FontB;
u8   SDConfigFlag = 0;
u8   SDShowFlag=1; 
u8   SDShowTimes = 1;
void SDPicShow(u8 pic);

//----------------------------------------------------------------------------==

/*******************************************************************************/


void I2C_DATAL_COMPARE(void);
void I2C_MEMSURE1(int kk, int vcomdcdata );
void I2C_DATAL_COMPAREX(void);
void I2C_DATAL_COMPARE(void);


u16 DATA_SUM1[30],DATA_SUMY[30],count1=0,count2=0;
u16 SURE[30];
u16 vcomdc_value=0;
u8 VCOMDC = 0Xb5;	  /////
u16 VCOMDC1=0x0000;


void MTP(void);
		  
void SHOW_value(u16 addx,u16 addy,u32 Data);

/*******************************************************************************/
int main(void)
{ 
	u8 mm=1;
	u32 rand_val;
	u32 sdcap=1;
//	u8 temp[9];

  #ifdef DEBUG
  debug();
  #endif
  /* System Clocks Configuration */
	RCC_Configuration();	 	
  /* NVIC Configuration */
	NVIC_Configuration();   //中断管理初始化   
	SysTick_Config();		    //延时用  	
	  
  //-------------------------------------------------
  /* Initialize the LCD */
	STM32_SSD1963_Init();

	KEYGPIO_Init();
	TSL2583_init();


/////SD  卡初始化部分。
	if(SDConfigFlag == 0)
	{
		SDShowFlag=1;
	
		Status =SD_InitAndConfig();  /*设置SD接口,初始化SD卡*/
		sdcap=sd_Capacity>>20; //(sdcap=sd_Capacity/1024/1024)
		All_Color(255,255,255)   ;
			if(sdcap == 0)
			{
				FontR = 255;
				FontB = 20;
				FontG = 20;
				LCD_DisplayStringLine_A(0,Line0,"SD ERROR!");	

				
				while(1)
				{
					DelayKEY(100);
				}
			}
			else
			{
				FontR = FontG = FontB = 255;		//屏蔽部分字符时候使用此段；
				SDConfigFlag = 1;
			}
	}


	FontR = FontG = FontB = 255;			   //屏蔽字符
	  	if (FAT_Init())	//初始化FAT文件系统
		  {	
		  		All_Color(255,255,255)   ;
				FontR = 255;
				FontB = 20;
				FontG = 20;
				LCD_DisplayStringLine_A(0,Line0,"SD ERROR!");
				while(1)
				{
					DelayKEY(100);
				}

		  }
		LCD_DisplayStringLine(0,Line2,"Waiting....");

		//////////////////show the project ----------------------------------------------------
		FontR = FontG = FontB = 0;	
	LCD_DisplayStringLine_A(20,350,"M1316B0_A");
//	LCD_WriteBMP(0,479,350,350+105-1,picc1);   		 /////for picc use
//	DelayKEY(15); 

		FontR = FontG = FontB = 255;			   //屏蔽字符
		SearchInit();
		
		SDShowTimes = 0;

		while (SDShowFlag) //开始BMP文件显示
		{	  
		  Disp_BMP() ;
		  Delay(1);
			SDShowFlag = 0;	
		}
	
	///////SD卡初始化结束。。。	
  
		FontR = FontG = FontB = 0;
		Flicker_sub_pixel();				DelayKEY(35);

		   sdcap = 1;
		   sdcap = KEYA7;
		   if(sdcap==0)
		   {	
		   		LCD_WriteArea(3,10,10,20,0,255,0);		   ////start auto otp flow
		   		I2C_DATAL_COMPAREX();
				LCD_WriteArea(3,10,100,110,0,0,255);		///	DelayKEY(55);	  ////to show the best flicker。。。
				ENTER_LP_mode(); //enter  LP mode
				Delay(3);
				MTP();	
				exit_sleep_mode( );
		   }


LCDTest:
	All_Color(255,0,0)   ;                      			DelayKEY(55);
           

///////*-------enter sleep mode------- */ 
  	enter_sleep_mode( ) ;
	DelayKEY(100);
	exit_sleep_mode( );
//////*-------exit sleep mode------- */

  	All_Color(0,255,0)   ;					  		DelayKEY(55);
	All_Color(0,0,255)   ;					  		DelayKEY(55);
	All_Color(0,0,0)   ;					  		DelayKEY(55);
	All_Color(255,255,255)   ;					  	DelayKEY(55);
	All_Color(128,128,128)   ;					  	DelayKEY(55);
	RGB_color();									DelayKEY(55);


	SDPicShow(1);				 			   DelayKEY(55);		  //SD  first   pic
	SDPicShow(2);								DelayKEY(55);		  //SD  second   pic
	SDPicShow(3);				 			   DelayKEY(55);		  //SD	third   pic

 
 
 
   goto LCDTest;

}

void SHOW_value(u16 addx,u16 addy,u32 Data)
{
	u32 sdcap;
	u8 TCH[7];

	sdcap = Data;
	TCH[4]=0;
	TCH[3]=(sdcap%10+0x30);sdcap/=10;
	TCH[2]=(sdcap%10+0x30);sdcap/=10;
	TCH[1]=(sdcap%10+0x30);sdcap/=10;
	TCH[0]=(sdcap%10+0x30);

	LCD_DisplayStringLine(addx,addy,TCH);

}

void READ_SSD2825(u8 cmd)
{
	u16 R_data=0;
	u32 sdcap=0;
	u8 TCH[6];

	R_data = SSD2825_READ(cmd);


	sdcap=R_data;
	   TCH[5]=0;
	   TCH[4]=(sdcap%10+0x30);sdcap/=10;
	   TCH[3]=(sdcap%10+0x30);sdcap/=10;
	   TCH[2]=(sdcap%10+0x30);sdcap/=10;
	   TCH[1]=(sdcap%10+0x30);sdcap/=10;
	   TCH[0]=(sdcap%10+0x30);
	    
	   LCD_DisplayStringLine(95,40,TCH);
	   DelayKEY(30);
}


void SDPicShow(u8 pic)
{
	u8 k;
	k = 0;
		SDShowFlag = 1;
		SDShowTimes = 1;

		image_count = pic;

		while(SDShowFlag) 
	{	  
	  Disp_BMP() ;
	  DelayKEY(1);

   		if((image_count >= pic)&&(k <= 1))
		{
			image_count = pic;
			k++;
		}

			  //此段为了使用SD卡的指定图片调用；；
		if(k >= 1)
		{
			SDShowFlag=0;	
		} 

	}

}

//=================================以下程序尽量不要修改==================================
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  /* Enable USART1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
//------------zp2000--------------------------------
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//该函数调用了两次

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

}

/*******************************************************************************
* Function Name  : InterruptConfig
* Description    : Configures the used IRQ Channels and sets their priority.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InterruptConfig(void)
{ 
  /* Deinitializes the NVIC */
  NVIC_DeInit();

  NVIC_Configuration();

  /* Configure the Priority Group to 2 bits */
  //  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		

  /* Configure the SysTick handler priority */
  //为了使用SDIO中断，下面的中断优先级被我改低了
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 1, 1);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length (time base 10 ms).
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(u32 nCount)				 //////delay  10ms
{
  TimingDelay = nCount*10;
  /* Enable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);	
  SysTick_ITConfig(ENABLE);//ENABLE DISABLE		
  while(TimingDelay != 0)
  {;;}
  SysTick_ITConfig(DISABLE);//ENABLE DISABLE 
}



void MDelay(u32 nCount)						 ////delay 1ms		for auto OTP use
{
  TimingDelay = nCount;
  /* Enable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);	
  SysTick_ITConfig(ENABLE);//ENABLE DISABLE		
  while(TimingDelay != 0)
  {;;}
  SysTick_ITConfig(DISABLE);//ENABLE DISABLE 
}
/*******************************************************************************
* Function Name  : Decrement_TimingDelay
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void Decrement_TimingDelay(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/*******************************************************************************
* Function Name  : SysTick_Config
* Description    : Configure a SysTick Base time to 10 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Config(void)
{
  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
 
  /* SysTick interrupt each 100 Hz with HCLK equal to 72MHz */
  SysTick_SetReload(72000);

  /* Enable the SysTick Interrupt */
  SysTick_ITConfig(DISABLE);//ENABLE DISABLE

  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);
}
//----设定按键接口--------------

void KEYGPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStructure; 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); 
  
  /*[把KEYGPIO KEY7/8配置成输入模式] */
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;    // 
  GPIO_Init(GPIOC, &GPIO_InitStructure);  

  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;    // 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void DelayKEY (u32 k)
{ 
    volatile u16 m=1;
	volatile u32 j;

      for (j=0; j<k; j++)
         {  
		 	m=KEYC6;	
			Delay(2);
            while(m==0)
//			while(m!=0)
              {
			  	m=KEYC6; 
			  	Delay(2);

			  }            
         }	

}

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef  USART_ClockInitStructure;

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle 
        - USART LastBit: The clock pulse of the last data bit is not output to 
                         the SCLK pin
  */
USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
/* Configure the USART1 synchronous paramters */
USART_ClockInit(USART1, &USART_ClockInitStructure);

USART_InitStructure.USART_BaudRate = 115200;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No ;
USART_InitStructure.USART_HardwareFlowControl = 
USART_HardwareFlowControl_None;

USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* Configure USART1 basic and asynchronous paramters */
USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
  
}

/*******************************************************************************
* Function Name  : USART1GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/* Private functions ---------------------------------------------------------*/
SD_Error SD_InitAndConfig(void)
{
  Status = SD_Init();

  if (Status == SD_OK)
  {
    /*----------------- Read CSD/CID MSD registers ------------------*/
    Status = SD_GetCardInfo(&SDCardInfo);
  }
  
  if (Status == SD_OK)
  {
    /*----------------- Select Card --------------------------------*/
    Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
  }
  
  if (Status == SD_OK)
  {
    /*----------------- Set BusWidth ------------------------------*/
    Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
  }
  
  /* Set Device Transfer Mode to INTERRUPT to DMA */
  if (Status == SD_OK)
  {  
    Status = SD_SetDeviceMode(SD_DMA_MODE);//SD_DMA_MODE,SD_INTERRUPT_MODE
  }
  return Status;
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/

void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif




void I2C_MEMSURE1(int kk  , int vcomdcdata)
{
	int sss=0,yyy=0,aaa=0;
	u16 temp1 =0;
	for (yyy=0;yyy<kk;yyy++)
	{SURE[sss]=vcomdcdata+aaa;

	HX8369_cmd_4(0xb6,SURE[sss],SURE[sss],0X00);		  ////set vcom

////	/*------新增一部分光感ID检验功能，一旦光感失效，即停止在此处。-----*/
	temp1 = I2C_READ_BYTE(0x52,0x9e);  /////ADC channel 0 data0low
	while(temp1 != 0x70)
	{
		temp1 = I2C_READ_BYTE(0x52,0x9e);  ////ADC channel 0 data0low

		LCD_DisplayStringLine(20,50,"sensor  error");		////红色高亮显示sensor error，警示光感失灵，暂停在此处。

	}


	DATA_SUM1[count1] =	GET_FLICKER();	 ///get flicker value
	SHOW_value(3,125,DATA_SUM1[count1]);
	count1=count1+1;
	SHOW_value(3,90,sss);
	SHOW_value(3,50,vcomdcdata+aaa); 		DelayKEY(1);

	aaa=aaa+1;
	sss=sss+1;
	}
}
void I2C_DATAL_COMPAREX(void)
{
	TSL2583_init();			////must initial TSL2583,,,for AUTO OTP use..
	I2C_MEMSURE1(10 , VCOMDC);
    I2C_DATAL_COMPARE();
    count1=0;
/*在最小值附近停下来。。。后续开启OTP功能即可。。*/
		VCOMDC1 = SURE[count2];				 //挑选出最好的VCOM的值，下CODE，看效果。
//		SHOW_value(3,50,VCOMDC1);

		HX8369_cmd_4(0xb6,VCOMDC1,VCOMDC1,0X00);	   ////write the perfect    vcom value

//		VCOMDC1 = DATA_SUM1[count2];		////挑选出最小的FLICKER值，显示出来。
//		SHOW_value(3,125,VCOMDC1);
//		VCOMDC1 = GET_FLICKER();
//		SHOW_value(3,155,VCOMDC1);		  DelayKEY(100);
}
void I2C_DATAL_COMPARE(void)
{
		int yy=1,zz=0;
	while(yy<count1 && zz<count1)
	{
		if(	DATA_SUM1[zz]>DATA_SUM1[yy])
		{
			count2=yy;zz=zz+1;	
		}
		else
		{
			count2=zz;yy=yy+1;
		}
	}	
}

void MTP(void)
{
	///display off
			SSD1963GPIOOUTCOM(0xBC);		                
	SSD1963GPIOOUTDATA_16(0x0001);					
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0x28);	  //
	Delay(5);			 //delay 40ms

			SSD1963GPIOOUTCOM(0xBC);		                
	SSD1963GPIOOUTDATA_16(0x0002);						
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xf1);	  //
	SSD1963GPIOOUTDATA_18(0x03);
	MDelay(25);					////wait more 1us   ...set  Delay 2ms

			SSD1963GPIOOUTCOM(0xBC);		                
	SSD1963GPIOOUTDATA_16(0x0002);					
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	MDelay(25);			   ////delay 1ms

	//////////set otp_index[8:0]
			SSD1963GPIOOUTCOM(0xBC);		                
	SSD1963GPIOOUTDATA_16(0x0006);					
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0d);		   /////烧录otp的0x0d的VCOM_F1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);	
	
	///////////////set otp_mask[7:0]
			SSD1963GPIOOUTCOM(0xBC);		               
	SSD1963GPIOOUTDATA_16(0x0006);						 
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0d);		   /////烧录otp的0x0d的VCOM_F1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);		

	/////////////set otp address
			SSD1963GPIOOUTCOM(0xBC);		              
	SSD1963GPIOOUTDATA_16(0x0006);						 
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0d);		   /////烧录otp的0x0d的VCOM_F1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);

	/////////////set otp_prog = 1;
			SSD1963GPIOOUTCOM(0xBC);		             
	SSD1963GPIOOUTDATA_16(0x0006);						 
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0d);		   /////烧录otp的0x0d的VCOM_F1
	SSD1963GPIOOUTDATA_18(0x01);
	MDelay(85);		 		///DELAY 11ms

	/////////////set otp_index[8:0]
			SSD1963GPIOOUTCOM(0xBC);		              
	SSD1963GPIOOUTDATA_16(0x0006);						 
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0e);				/////烧录otp的0x0E的VCOM_B1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);

	/////////////set otp_mask[7:0]
			SSD1963GPIOOUTCOM(0xBC);		                
	SSD1963GPIOOUTDATA_16(0x0006);						
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0e);				/////烧录otp的0x0E的VCOM_B1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);

	/////////////set otp address
			SSD1963GPIOOUTCOM(0xBC);		                 
	SSD1963GPIOOUTDATA_16(0x0006);					
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0e);				/////烧录otp的0x0E的VCOM_B1
	SSD1963GPIOOUTDATA_18(0x00);
	MDelay(15);

	/////////////set otp_prog = 1;
			SSD1963GPIOOUTCOM(0xBC);		                 
	SSD1963GPIOOUTDATA_16(0x0006);						  
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xaa);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x00);
	SSD1963GPIOOUTDATA_18(0x0e);				/////烧录otp的0x0E的VCOM_B1
	SSD1963GPIOOUTDATA_18(0x01);
	MDelay(85);		 		///DELAY 11ms

   	//////MTP   OVER ,,,OTP  Disable
			SSD1963GPIOOUTCOM(0xBC);		                 
	SSD1963GPIOOUTDATA_16(0x0002);						  
			SSD1963GPIOOUTCOM(0xbf);
	SSD1963GPIOOUTDATA_18(0xb7);	  //
	SSD1963GPIOOUTDATA_18(0xff);
	MDelay(100);
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
