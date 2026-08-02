#ifndef __OLED_H
#define __OLED_H 

#include "main.h"
#include "stdlib.h"	

//-----------------OLED�˿ڶ���---------------- //
/*���Ը���Cube MXIO��ʼ��IO�� ����CLK SDA  ��ʼ��IO���ں���MX_GPIO_Init����!!!*/
#define OLED_SCL_Clr() HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_RESET)//SCL
#define OLED_SCL_Set() HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_SET)//SCL

#define OLED_SDA_Clr() HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_RESET)//SDA
#define OLED_SDA_Set() HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_SET)//SDA

/* Read SDA back, used for the ACK bit. Comments in this file are kept
 * ASCII on purpose: the file is GBK encoded, so UTF-8 Chinese added here
 * would show up as mojibake in the editor. */
#define OLED_SDA_Read() HAL_GPIO_ReadPin(SDA_GPIO_Port,SDA_Pin)

//#define OLED_RES_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//RES
//#define OLED_RES_Set() GPIO_SetBits(GPIOA,GPIO_Pin_2)


#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

/* SCL target frequency. SSD1306 datasheet limit is 400kHz.
 * Lower this (e.g. 200000) if the bus still glitches with motors running. */
#ifndef OLED_IIC_SCL_HZ
#define OLED_IIC_SCL_HZ 400000UL
#endif

/* Diagnostics, readable over the debug UART if you want to watch them:
 * oled_ack_shibai_cnt  - how many bytes came back without an ACK
 * oled_zijiu_cnt       - how many times the driver re-initialised the panel */
extern volatile u32 oled_ack_shibai_cnt;
extern volatile u32 oled_zijiu_cnt;

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);

#endif

