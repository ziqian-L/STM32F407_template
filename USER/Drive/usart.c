#include "sys.h"
#include "usart.h"	
#include "led.h"

RingBuff_t Uart2_RingBuff,Uart1_RingBuff,Uart3_RingBuff;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
/*
���ϸþ���д_ttywrch()����
���Ա������
..\OBJ\STM32F4.axf: Error: L6915E: Library reports error: 
__use_no_semihosting was requested, but _ttywrch was referenced
Not enough information to list load addresses in the image map.
�ο���https://blog.csdn.net/weixin_42911817/article/details/109539547
*/
//_ttywrch(int ch)
//{
//	ch = ch;
//}
#endif
 

//����1�жϷ������

//��ʼ������1 
//bound:������
void USART1_Init(uint32_t bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

#endif
	
}


///*****
// * ����1�жϷ�����
//*****/
//void USART1_IRQHandler(void)
//{
//	uint8_t rdata;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{
//		rdata = USART_ReceiveData(USART1);
//		if(Write_RingBuff(&Uart1_RingBuff,rdata) == RINGBUFF_ERR)
//			LED1=0;
//		else
//			LED1=1;
//	}
//} 


 //��ʼ��IO ����2
//bound:������
void USART2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //GPIOA��USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

    //PA2��PA3���á����ơ�100MHz������
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_High_Speed;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //GPIO����
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

    //��ʼ��USART2
    //8�ֳ���1ֹͣλ������żУ�顢�շ�ģʽ����Ӳ��������
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2,&USART_InitStructure);

    //����USART2�ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //�������ݽ��ռĴ����ǿ��ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//�������տ���ʱ�ж�
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    //ʹ�ܴ���2
    USART_Cmd(USART2,ENABLE);
}

///*****
// * ����2�жϷ�����
//*****/
//void USART2_IRQHandler(void)
//{
//	uint8_t rdata;
//    if (USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
//    {
//		rdata = USART_ReceiveData(USART2);
//		if(Write_RingBuff(&Uart2_RingBuff, rdata) == RINGBUFF_ERR){//������������
//			LED1=0;
//		}else{
//			LED1=1;
//		}
//    }
//}


/***************************************���λ��������***************************************/
/*****
 * ���λ�������ʼ��
*****/
void RingBuff_Init(RingBuff_t *ringbuff)
{
	//��ʼ�����λ�������ز���
	ringbuff->Head = 0;
	ringbuff->Tail = 0;
	ringbuff->Length = 0;
}

/*****
 * �����λ�����д������
 * �����жϻ������Ƿ�Ϊ����Ϊ�����ش���
 * ������ֵд�뻺����
 * ��β������β�Ի�������Сȡ�࣬��β=��������С��β����0����ֹԽ��
*****/
uint8_t Write_RingBuff(RingBuff_t *ringbuff, uint8_t data)
{
  if(ringbuff->Length >= RINGBUFF_LEN) //�жϻ������Ƿ�����
  {
    return RINGBUFF_ERR;
  }
  ringbuff->Ring_data[ringbuff->Tail]=data;
  ringbuff->Tail = (ringbuff->Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringbuff->Length++;
  return RINGBUFF_OK;
}

/*****
 * �ӻ��λ�������ȡ����
 * �����жϻ������Ƿǿգ�Ϊ�շ��ش���
 * ���������ݸ�ֵ����������ָ��
 * ��ͷ������ͷ�Ի�������Сȡ�࣬��ͷ=��������С��ͷ����0����ֹԽ��
*****/
uint8_t Read_RingBuff(RingBuff_t *ringbuff, uint8_t *rData)
{
  if(ringbuff->Length == 0)//�жϷǿ�
  {
    return RINGBUFF_ERR;
  }
  *rData = ringbuff->Ring_data[ringbuff->Head];//�Ƚ��ȳ�FIFO���ӻ�����ͷ��
  ringbuff->Head = (ringbuff->Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringbuff->Length--;
  return RINGBUFF_OK;
}

/***************************************�����ݰ����ж���***************************************/
//����֡ͷ��֡��ʶ��֡β��
#define DATA_FRAME_HEAD	0xAA
#define DATA_FRAME_MARK	0x01
#define DATA_FRAME_END	0xFF
#define Length 4

/***************************************ʹ��֡ͷ֡β�������ݴ���ͽ��***************************************/
/****
 * ��������ʱ�����ݽ��н��
******/
int16_t Receive_Unpack(RingBuff_t *ringbuff,uint8_t *data1,uint8_t *data2)
{
	static uint8_t uart_dec_count;
	static uint8_t uart_rec_data[5];
	uint8_t ret = 1;
	if(Read_RingBuff(ringbuff, &uart_rec_data[uart_dec_count]) == RINGBUFF_ERR){
		return 1;
	}
	if((uart_dec_count == 0)&&(uart_rec_data[uart_dec_count] != 0xAA))//����һ�������Ƿ�Ϊ0xAA
    {
		uart_rec_data[uart_dec_count] = 0;						
	} else if((uart_dec_count == 1)&&(uart_rec_data[uart_dec_count] != 0x01))//���ڶ��������Ƿ�Ϊ0x01
    {
		uart_rec_data[uart_dec_count] = 0;
		uart_rec_data[uart_dec_count-1] = 0;
		uart_dec_count = 0;
	} else if((uart_dec_count == 4)&&(uart_rec_data[uart_dec_count] != 0xFF))
    {
		uart_rec_data[uart_dec_count] = 0;
		uart_rec_data[uart_dec_count-1] = 0;
		uart_rec_data[uart_dec_count-2] = 0;
		uart_rec_data[uart_dec_count-3] = 0;
		uart_rec_data[uart_dec_count-4] = 0;
		uart_dec_count = 0;
	}else
    {
        if(uart_dec_count == 4)
        {
            *data1 = uart_rec_data[2];
			*data2 = uart_rec_data[3];
			ret = 0;
        }
    	uart_dec_count ++;
		if(uart_dec_count == 5)
		{
			uart_dec_count = 0;
		}
    }
    return ret;
}



/*****
 * ��������ʱ������ʹ��֡ͷ֡β���д������
 * ����
 * ��������
 * ���ݸ���
*****/
void Send_Pack(USART_TypeDef* USARTx,uint8_t *DataArray)
{
	uint8_t i;
	USART_SendData(USARTx,DATA_FRAME_HEAD);
	for (i=0;i<Length;i++)
	{
		USART_SendData(USARTx,DataArray[i]);
	}
	USART_SendData(USARTx,DATA_FRAME_END);
}




