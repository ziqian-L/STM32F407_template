#include "graysensor.h"
#include "control.h"
#include "pid.h"
#include "led.h"
#include "delay.h"

#ifdef Five_Way_GraySensor
struct
{
    float Mid_yaw = 0;          //�Ҷ�ѭ��Ŀ��yawֵ
	float Yaw_Error = 0;        //Ŀ��yawֵ�뵱ǰyawֵ�Ĳ��yaw��ƫ�ƣ������ж�С��ƫ���˺��߶���
    uint8_t Debounce = 0;       //�ж���ʼ��ʱ����
    uint8_t Circle = 0;         //��ʻȦ��
    uint16_t Circle_count = 0;  //Ȧ������ֵ
    uint16_t Debounce_count = 0;//��������ֵ
    double Gray_Count_Error = 0;//�Ҷȼ����ۼƵĲ�ֵ
	uint8_t Gray_Location_Information = 0;  //�Ҷȴ�������λ����Ϣ
} FiveWay;

/*****
 * 5·�Ҷȴ���������-��������
 * ʹ��IO�ڣ�PE7��PE8��PE9��PE10��PE11
 * PE12���ں���Թܣ�������ҩС��ʶ������
*****/
void GraySensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //GPIOEʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

    //PE7��PE8��PE9��PE10��PE11��PE12���롢���ơ�100MHz������
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE,&GPIO_InitStruct);
}


/*****
 * 5·�Ҷȴ�����Ѳ��
*****/
int16_t GraySensor_LinePatrol(void)
{
	FiveWay.Yaw_Error = Yaw - FiveWay.Mid_yaw;
	//ͨ����ȡGPIOE���������ݼĴ�����ȡ�Ҷȴ�����λ����Ϣ
    FiveWay.Gray_Location_Information = (uint8_t)(GPIOE->IDR>>7);
	//�����ݸ���λ�����ֹ����
	FiveWay.Gray_Location_Information &= ~(uint8_t)0xE0;
	//λ��	8	7	6	5	4	3	2	1
	//ֵ��	0	0	0	L1	L1	M	R1	R2
/*************************·��ʶ��*************************/
	if (FiveWay.Gray_Location_Information == 0x0A)//01010 - 0x0A
	{
		if (crossing == 0)
		{
			FiveWay.Gray_Location_Information = 0x08;
			FiveWay.Gray_Count_Error = 0;
			FiveWay.Yaw_Error = 0;
		}
		else if(crossing == 1)
		{
			FiveWay.Gray_Location_Information = 0x02;
			FiveWay.Gray_Count_Error = 0;
			FiveWay.Yaw_Error = 0;
		}
	}
/*************************ѭ������*************************/
    switch (FiveWay.Gray_Location_Information)
    {
    /*����������ⲻ������ʱ*/
    case 0x00:
        /************����ƫ��************/
        if (FiveWay.Yaw_Error < 0)
        {
            //����λ��L2��L1֮�䣬��ʱ����ƫ��
            if (FiveWay.Yaw_Error > -22 && FiveWay.Yaw_Error < -12)
            {
                FiveWay.Gray_Count_Error+=0.004;
                Limit_double_Value(&FiveWay.Gray_Count_Error,2.4,3.4);
            }
            //����Խ����ⷶΧ����ʱ����ƫ��
            else if (FiveWay.Yaw_Error < -34) 
				FiveWay.Gray_Count_Error+=0.01;
        }
        /************����ƫ��************/
        else if (FiveWay.Yaw_Error > 0)
        {
            //����λ��R1��R2֮��,��ʱ����ƫ��
            if (FiveWay.Yaw_Error > 12 && FiveWay.Yaw_Error < 22)
            {
                FiveWay.Gray_Count_Error-=0.004;
                Limit_double_Value(&FiveWay.Gray_Count_Error,-3.4,-2.4);
            }
            //����Խ����ⷶΧ����ʱ����ƫ��
            else if (FiveWay.Yaw_Error > 34)
				FiveWay.Gray_Count_Error-=0.01;
        }
        break;

    /*�����������ڻҶȴ�������ʱ*/
//    case 0x01://00001 - 0x01
//        //����һ����������⵽����
//        //������Gray_R2�£���ʱ����ƫ��
//        FiveWay.Gray_Count_Error-=0.005;
//        Limit_double_Value(&FiveWay.Gray_Count_Error,-4.8,-3.4);
//        break;

    case 0x02://00010 - 0x02
        //����һ����������⵽����
        //������Gray_R1�£���ʱ����ƫ��
		FiveWay.Gray_Count_Error-=0.002;
        Limit_double_Value(&FiveWay.Gray_Count_Error,-2.3,-1.2);
        break;

    case 0x06://00110 - 0x06
        //��������������⵽����
        //����λ��M��R1֮��,��ʱ����ƫ��
        FiveWay.Gray_Count_Error-=0.001;
        Limit_double_Value(&FiveWay.Gray_Count_Error,-1.2,-0.7);
        break;

    case 0x04://00100 - 0x04
        //����һ����������⵽����
        //������Gray_M�£���ʱ����λ���м�
        Limit_double_Value(&FiveWay.Gray_Count_Error,-0.7,0.7);
		FiveWay.Gray_Count_Error = 0;
        //����Ŀ��yaw
        FiveWay.Mid_yaw = Yaw;
        //������ʼλ��ʶ��������
        FiveWay.Debounce_count = 0;
        break;

    case 0x0C://01100 - 0x0C
        //��������������⵽����
        //����λ��L1��M֮�䣬��ʱ����ƫ��
        FiveWay.Gray_Count_Error+=0.001;
        Limit_double_Value(&FiveWay.Gray_Count_Error,0.7,1.2);
        break;

    case 0x08://01000 - 0x8
        //����һ����������⵽����
        //������Gray_L1�£���ʱ����ƫ��
		FiveWay.Gray_Count_Error+=0.002;
        Limit_double_Value(&FiveWay.Gray_Count_Error,1.2,2.3);
        break;

//    case 0x10://10000 - 0x10
//        //����һ����������⵽����
//        //������Gray_L2�£���ʱ����ƫ��
//        FiveWay.Gray_Count_Error+=0.005;
//        Limit_double_Value(&FiveWay.Gray_Count_Error,3.4,4.8);
//        break;

/*************************ֹͣ��ʶ�𲿷�*************************/
	case 0x0E://01110 - 0x0E
		if (FiveWay.Gray_Count_Error > -0.71 && FiveWay.Gray_Count_Error < 0.71)
		{
            FiveWay.Debounce_count++;
            if (FiveWay.Debounce_count >= FiveWay.Debounce)
            {
                FiveWay.Circle_count++;
                if (FiveWay.Circle_count  >= FiveWay.Circle)
                {
                    PID_Encoders_SetPoint(0,0);
                    FiveWay.Circle_count = 0;
                }
                FiveWay.Debounce_count = 0;
            }
		}
		break;
	case 0x1F://11111 - 0x1F
		PID_Encoders_SetPoint(0,0);
		break;
    default:
        break;
    }
    return Incremental_PID_Contorl(&LinePatrol,FiveWay.Gray_Count_Error);
}

/*****
 * �޸�Ѳ��Ȧ��
*****/
void GraySensor_Circle(uint8_t Circle)
{
    FiveWay.Circle = Circle;
}


#endif

/*******************************************************************************************/

/*******************************************************************************************/

/*******************************************************************************************/

#ifdef GW_GraySensor

/* ���ɨ�赽�ĵ�ַ */
uint8_t scan_addr[128] = {0};
volatile uint8_t count;
uint8_t ping_response;
uint8_t gray_sensor[8];
uint8_t digital_data;

struct
{
	uint8_t Target;//Ŀ��״̬,ֻ��������
    uint8_t Turn_times;
    double offset;
}GWGray;

/* �������IIC���� */
sw_i2c_interface_t i2c_interface = 
{
	.sda_in = sda_in,
	.scl_out = scl_out,
	.sda_out = sda_out,
	.user_data = 0, //�û����ݣ������������������õ�
};

/*****
 * ��ֲ�ڸ�Ϊ8·�Ҷȴ���������-���IIC(�ɸ�����Ҫ���и�ΪӲ��IIC)
 * ʹ��IO�ڣ���5V���̵�PF0(SDA)��PF1(SCL)
*****/
void GraySensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //GPIOE��GPIOFʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

    //PF0��PF1�������©��100MHz������
    GPIO_InitStruct.GPIO_Pin    = IIC2_SDA|IIC2_SCL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_UP;
    GPIO_Init(GPIOF,&GPIO_InitStruct);

    GPIO_SetBits(GPIOF,IIC2_SDA|IIC2_SCL);

	/* ��һ��IICͨѶ��ʧ�ܣ���Ϊ���IIC������start�����ֶ�����stopҲ������ */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	/* ����IICͨѶ�������� */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	
	/* ɨ�迪ʼ */
	count = i2c_scan(&i2c_interface, scan_addr);

    GWGray.Target = 0xE7;
    GWGray.Turn_times = 0;
}



float GraySensor_LinePatrol(void)
{
	uint32_t i;
    uint8_t Segments = 1;//����
    uint8_t Segments_Length[9]={0};//ÿ�εĳ���
    uint8_t L_Target = 0;
    uint8_t R_Target = 0;

    //��ȡ����������
    sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data ��1~8��̽ͷ��������
    //���ֽ����8���������浽�˸����������Ϊgray_sensor[0] ~ gray_sensor[7],Ҳ�����Ǳ���val1 ~ val8, ��Ϊ�Ǻ궨��
    SEP_ALL_BIT8(digital_data, 
        gray_sensor[0], //̽ͷ1
        gray_sensor[1], //̽ͷ2
        gray_sensor[2], //̽ͷ3
        gray_sensor[3], //̽ͷ4
        gray_sensor[4], //̽ͷ5
        gray_sensor[5], //̽ͷ6
        gray_sensor[6], //̽ͷ7
        gray_sensor[7]  //̽ͷ8
    );

    //digital_data �����λ��̽ͷ1�����ݣ�digital_data �����λ��̽ͷ8������
    //����λ��8   7   6   5   4   3   2   1
    //̽ͷ�ţ�8   7   6   5   4   3   2   1
    //ƫ��λ��R3  R2  R1  R0  L0  L1  L2  L3
/*************************��������*************************/
    //�������������ݣ��õ��������γ���֮��Ϳ���ͨ��̽ͷ1����ɫ�õ�ÿ���ε���ɫ�ͳ�
    for (i = 0; i < 7; i++)
    {
        //����ʵ�ʿ���������
        if (gray_sensor[i] != gray_sensor[i+1])
            Segments++;
        Segments_Length[Segments]++;
    }
    //��������������
    Segments_Length[1]+=1;
    //����Ŀ�꿪��������
    for (i = 0; i < 8; i++)
    {
        if (((GWGray.Target>>i) & 0x01) != 0)  L_Target++;
        else break;
    }
    for (i = 0; i < 8; i++)
    {
        if (((GWGray.Target<<i) & 0x80) != 0) R_Target++;
        else break;
    }
/*************************ѭ������*************************/
    //����ƫ����
    if (gray_sensor[0]==1 && Segments == 3)
    {
        //ʹ��һ������
        offset_alculation(Segments_Length[1]-L_Target);
        // offset_alculation(Segments_Length[3]-R_Target);
    }
    //û��ƫ����ʱ��������λ��
    if ((Segments_Length[1]-L_Target) == 0 && (Segments_Length[3]-R_Target) == 0)
    {
        GWGray.offset = 0;
    }
    //PID����
	return 0;
}

/*****
 * ��ԣ�Segments_Length[1]-L_Target������ƫ����ʹ�õ��ǣ�Segments_Length[3]-R_Target������Ҫ�޸�
*****/
void offset_alculation(int8_t Gray_offset)
{
    switch (Gray_offset)
    {
        /************����ƫ��************/
        case 3:GWGray.offset-=0.005;Limit_double_Value(&GWGray.offset,-3.2,-2.5);break;
        case 2:GWGray.offset-=0.004;Limit_double_Value(&GWGray.offset,-2.5,-1.2);break;
        case 1:GWGray.offset-=0.003;Limit_double_Value(&GWGray.offset,-1.2,-0.4);break;
        /************����ƫ��************/
        case  -1:GWGray.offset+=0.003;Limit_double_Value(&GWGray.offset,0.4,1.2);break;
        case  -2:GWGray.offset+=0.004;Limit_double_Value(&GWGray.offset,1.2,2.5);break;
        case  -3:GWGray.offset+=0.005;Limit_double_Value(&GWGray.offset,2.5,3.2);break;
    }
}



/* ����sda������� bit=0Ϊ�͵�ƽ bit=1Ϊ�ߵ�ƽ */
void sda_out(uint8_t bit, void *user_data)
{
    SDA_OUT;

	GPIO_WriteBit(GPIOF, IIC2_SDA, (BitAction)bit);
	
	/* IIC����ӳ� */
	delay_us(10);
}

/* ����sda��ȡ���� bit Ϊ���صĵ�ƽֵ */
uint8_t sda_in(void *user_data)
{
	uint8_t bit;
    
    SDA_IN;

	bit = (uint8_t)GPIO_ReadInputDataBit(GPIOF, IIC2_SDA);
	
	/* IIC����ӳ� */
	delay_us(10);
	return bit;
}

/* ����sclʱ��������� bit=0Ϊ�͵�ƽ bit=1Ϊ�ߵ�ƽ */
void scl_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(GPIOF, IIC2_SCL, (BitAction)bit);
	
	/* IIC����ӳ� */
	delay_us(10);
}

/**
 * i2c��ַɨ��
 * @param scan_addr ɨ������ĵ�ַ���,��ֵ��Ϊ0��Ϊɨ�赽�ĵ�ַ��ɨ���ĵ�ַ�ᰤ�������������ǰ��
 * @return ����ɨ�赽���豸����, 0Ϊ���豸����
 */
uint8_t i2c_scan(sw_i2c_interface_t *i2c_interface, uint8_t *scan_addr)
{
	int i;
	uint8_t count = 0;
	uint8_t data;
	int8_t ret;
	
	for (i = 1; i < 127; ++i) {
		ret = sw_i2c_read(i2c_interface, i << 1, &data, 1);
		if (ret == 0) {
			scan_addr[count] = i;
			++count;
		}
	}
	
	return count;
}
#endif


