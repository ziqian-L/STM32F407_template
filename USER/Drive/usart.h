#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

	
//如果想串口中断接收，请不要注释以下宏定义


#define RINGBUFF_LEN			200
#define RINGBUFF_OK           	1     
#define RINGBUFF_ERR          	0 
typedef struct
{
	uint16_t Head;
	uint16_t Tail;
	uint16_t Length;
	uint8_t Ring_data[RINGBUFF_LEN];
}RingBuff_t;

extern RingBuff_t Uart2_RingBuff,Uart1_RingBuff,Uart3_RingBuff;

/*串口初始化*/
void USART1_Init(uint32_t bound);
void USART2_Init(uint32_t bound);

/*环形缓冲区存储数据*/
void RingBuff_Init(RingBuff_t *ringbuff);
uint8_t Write_RingBuff(RingBuff_t *ringbuff, uint8_t data);
uint8_t Read_RingBuff(RingBuff_t *ringbuff, uint8_t *rData);

int16_t Receive_Unpack(RingBuff_t *ringbuff,uint8_t *data1,uint8_t *data2);
void Send_Pack(USART_TypeDef* USARTx,uint8_t *DataArray);
#endif


