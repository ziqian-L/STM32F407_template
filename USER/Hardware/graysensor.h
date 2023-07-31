#ifndef __GRAYSENSOR_H
#define __GRAYSENSOR_H

#include "sys.h"
#include "sw_i2c.h"
#include "gw_grayscale_sensor.h"

//选择使用5路灰度传感器还是感为的8路灰度传感器（带串行和IIC）
#define GW_GraySensor


#ifdef Five_Way_GraySensor
#define Gray_R2		PEin(7)
#define Gray_R1		PEin(8)
#define Gray_M		PEin(9)
#define Gray_L1		PEin(10)
#define Gray_L2		PEin(11)
#endif

#define Gray_Spot	PEin(12)

#ifdef GW_GraySensor
#define IIC2_SDA GPIO_Pin_0
#define IIC2_SCL GPIO_Pin_1
#define SDA_IN  {GPIOF->MODER&=~(3<<(0*2));GPIOF->MODER|=0<<0*2;}
#define SDA_OUT {GPIOF->MODER&=~(3<<(0*2));GPIOF->MODER|=1<<0*2;}

extern uint8_t gray_sensor[8];
void offset_alculation(int8_t Gray_offset);
void sda_out(uint8_t bit, void *user_data);
uint8_t sda_in(void *user_data);
void scl_out(uint8_t bit, void *user_data);
uint8_t i2c_scan(sw_i2c_interface_t *i2c_interface, uint8_t *scan_addr);
#endif

void GraySensor_Init(void);
float GraySensor_LinePatrol(void);

void Medicine_Spot(void);
#endif
