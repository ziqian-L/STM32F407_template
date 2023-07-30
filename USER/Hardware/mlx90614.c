#include "mlx90614.h"
#include "delay.h"

void Temperature_measure_Init(void)
{
	IIC_Init();
}

float Read_Temperature(uint8_t address)
{
    uint8_t Temp_L = 0,Temp_H = 0,Temp_PE = 0;
    uint16_t Temp = 0;		//�Ĵ�����ֵ
	float Temperature = 0;	//���ص��¶�ֵ
	//���Ͷ�ȡ�ļĴ����ĵ�ַ
    IIC_Start();
    IIC_Write_Byte(0x00);
    IIC_Wait_Ack();
    IIC_Write_Byte(address);
    IIC_Wait_Ack();
	/*------------------*/
	//��ȡ�Ĵ���ֵ
    IIC_Start();
    IIC_Write_Byte(0x01);
	IIC_Wait_Ack();
	//���� 16 λ�����ݺ� 8 λ PE
    Temp_L = IIC_Read_Byte(1);
    Temp_H = IIC_Read_Byte(1);
    Temp_PE = IIC_Read_Byte(1);
    IIC_Stop();
    Temp = (Temp_H<<8) | Temp_L;
    Temperature = (((float)Temp * 2) - 27315)/100;
	return Temperature;
}

/******
IICЭ�鴫����̣���ʼ�ź�->7λ�豸��ַ+R/W->R/W�ļĴ���->R/W->ֹͣ�ź�
д�����ݣ���ʼ�ź�->7λ�豸��ַ+0->д��ļĴ���->д��->ֹͣ�ź�
��ȡ���ݣ���ʼ�ź�->7λ�豸��ַ+0->��ȡ����->���ͷ�Ӧ���ź�(1)->ֹͣ�ź�
******/

/******************************************���IIC��ʼ��******************************************/
/******
 * IIC��ʼ��
 * PB6��SCL
 * PB7��SDA
******/
void IIC_Init(void)
{
    //ʹ��ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    //GPIO��ʼ��
    GPIOE->MODER    |= (GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOE->OTYPER   |= 0x00;
    GPIOE->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR14_1 | GPIO_OSPEEDER_OSPEEDR15_1);
    GPIOE->PUPDR    |= (GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR15_0);
    //������������
    IIC_SCL = 1;
	IIC_SDA = 1;
}

/******************************************���IICЭ��******************************************/
/******
 * ������ʼ�ź�
******/
void IIC_Start(void)
{
	TE_SDA_OUT;     //sda�����
	IIC_SDA=1;	
	delay_us(4);	
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(4);
}

/******
 * д��һ���ֽ�
******/
void IIC_Write_Byte(uint8_t IIC_data)
{
    uint8_t i;
    //ȷ��SDAΪ���
    TE_SDA_OUT;
    //SCLΪ��ʱ������SDA�ı仯
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_us(10);
        IIC_SDA = (IIC_data&0x80)>>7;
        IIC_data<<=1;
        delay_us(10);
		IIC_SCL=1;
		delay_us(10); 
    }
    //׼���������
    IIC_SCL = 0;
}
/******
 * �ȴ�Ӧ���ź�
******/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ErrTime = 0;
    //ȷ��SDAΪ���룬SCL��SDA����
    TE_SDA_IN;
    IIC_SDA = 1;delay_us(6);
    IIC_SCL = 1;delay_us(6);
    //�͵�ƽ��ʾӦ�𣬸ߵ�ƽ��ʾ��Ӧ��
    while (READ_SDA)
    {
        ErrTime++;
        if (ErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    //׼���������
    IIC_SCL = 0;
    return 0;
}

/******
 * ��ȡһ���ֽ�
 * ack=1ʱ������Ack��ack=0������NAck
******/
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive = 0;
    //ȷ��SDAΪ����
    TE_SDA_IN;
    //��SCLΪ��ʱ����ȡSDA
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_us(10);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
		delay_us(5);         
    }
    //ack=1ʱ������Ack��ack=0������NAck
    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return receive;
}

/******
 * ����Ӧ���ź�
******/
void IIC_Ack(void)
{
    //��9��ʱ������֮ǰ�ĵ͵�ƽ�ڼ佫SDA������
    //����ȷ���ڸ�ʱ�ӵĸߵ�ƽ�ڼ�Ϊ�ȶ��ĵ͵�ƽ
    IIC_SCL = 0;
    //ȷ��SDAΪ���
    TE_SDA_OUT;
    IIC_SDA = 0;
    delay_us(10);
    IIC_SCL = 1;
    delay_us(10);
    //׼���������
    IIC_SCL = 0;
}

/******
 * ������Ӧ���ź�
******/
void IIC_NAck(void)
{
    //��9��ʱ������֮ǰ�ĵ͵�ƽ�ڼ佫SDA������
    //����ȷ���ڸ�ʱ�ӵĸߵ�ƽ�ڼ�Ϊ�ȶ��ĸߵ�ƽ
    IIC_SCL = 0;
    //ȷ��SDAΪ���
    TE_SDA_OUT;
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    //׼���������
    IIC_SCL = 0;
}

/******
 * ����ֹͣ�ź�
******/
void IIC_Stop(void)
{
    //ȷ��SDAΪ�����SCL��SDA����
    TE_SDA_OUT;
    IIC_SCL = 0;
    IIC_SDA = 0;
    delay_us(4);
    //SCLΪ��ʱ��SDA�ɵ����䵽��
    IIC_SCL = 1;
    IIC_SDA = 1;
    delay_us(4);
}


