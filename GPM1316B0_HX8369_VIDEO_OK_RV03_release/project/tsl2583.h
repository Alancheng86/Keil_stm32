/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : TSL2583.h
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 09/22/2008
* Description        : 
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL2583_H
#define __TSL2583_H	  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"






#define slave_addr3 0x52;

void I2C_WRITE_BYTE(unsigned char slave_addr,unsigned char cmd_code,unsigned char data_byte)   ;
unsigned short I2C_READ_BYTE(unsigned char slave_addr,unsigned char cmd_code)   ;
void I2CSDASetInput(void);
void I2CSDASetOutput(void);
unsigned short GET_FLICKER(void);
void TSL2583_init(void);

/*      light sensor      SDA=GPIO_PIN3 SCL=GPIO_PIN5     */
#define RCC_APB2Periph_GPIO_TPIIC  RCC_APB2Periph_GPIOA
#define OTPSCL_L GPIO_ResetBits( GPIOA, GPIO_Pin_5)  //SCL
#define OTPSCL_H GPIO_SetBits  ( GPIOA, GPIO_Pin_5)
#define OTPSDA_L GPIO_ResetBits( GPIOA, GPIO_Pin_3)  //SDA
#define OTPSDA_H GPIO_SetBits  ( GPIOA, GPIO_Pin_3)
//////////////////////////////////////////////////////////////



#endif /* __TSL2583_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
