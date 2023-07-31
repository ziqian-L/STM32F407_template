#ifndef __MLX90614_H
#define __MLX90614_H

#include "sys.h"

void Temperature_measure_Init(void);
float Read_Temperature(uint8_t address);

#define TE_SDA_IN  {GPIOE->MODER &= ~GPIO_MODER_MODER14;GPIOE->MODER |= 0x00;}
#define TE_SDA_OUT {GPIOE->MODER &= ~GPIO_MODER_MODER14;GPIOE->MODER |= GPIO_MODER_MODER14_0;}

#define IIC_SCL    PEout(15) //SCL
#define IIC_SDA    PEout(14) //SDA	 
#define READ_SDA   PEin(14)  //输入SDA 

void IIC_Init(void);

void IIC_Write_Byte(uint8_t IIC_data);
uint8_t IIC_Read_Byte(unsigned char ack);

void IIC_Start(void);
void IIC_Stop(void);

uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

#endif
