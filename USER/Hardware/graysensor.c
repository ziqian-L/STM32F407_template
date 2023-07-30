#include "graysensor.h"
#include "control.h"
#include "pid.h"
#include "led.h"
#include "delay.h"

#ifdef Five_Way_GraySensor
struct
{
    float Mid_yaw = 0;          //灰度循迹目标yaw值
	float Yaw_Error = 0;        //目标yaw值与当前yaw值的差，即yaw的偏移，用于判断小车偏移了黑线多少
    uint8_t Debounce = 0;       //判断起始线时消抖
    uint8_t Circle = 0;         //行驶圈数
    uint16_t Circle_count = 0;  //圈数计数值
    uint16_t Debounce_count = 0;//消抖计数值
    double Gray_Count_Error = 0;//灰度计数累计的差值
	uint8_t Gray_Location_Information = 0;  //灰度传感器的位置信息
} FiveWay;

/*****
 * 5路灰度传感器驱动-并行输入
 * 使用IO口：PE7、PE8、PE9、PE10、PE11
 * PE12用在红外对管，用于送药小车识别物体
*****/
void GraySensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

    //PE7、PE8、PE9、PE10、PE11、PE12输入、挽推、100MHz、浮空
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE,&GPIO_InitStruct);
}


/*****
 * 5路灰度传感器巡线
*****/
int16_t GraySensor_LinePatrol(void)
{
	FiveWay.Yaw_Error = Yaw - FiveWay.Mid_yaw;
	//通过读取GPIOE的输入数据寄存器获取灰度传感器位置信息
    FiveWay.Gray_Location_Information = (uint8_t)(GPIOE->IDR>>7);
	//将数据高三位清零防止干扰
	FiveWay.Gray_Location_Information &= ~(uint8_t)0xE0;
	//位：	8	7	6	5	4	3	2	1
	//值：	0	0	0	L1	L1	M	R1	R2
/*************************路口识别*************************/
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
/*************************循迹部分*************************/
    switch (FiveWay.Gray_Location_Information)
    {
    /*当传感器检测不到黑线时*/
    case 0x00:
        /************车身偏右************/
        if (FiveWay.Yaw_Error < 0)
        {
            //黑线位于L2与L1之间，此时车身偏右
            if (FiveWay.Yaw_Error > -22 && FiveWay.Yaw_Error < -12)
            {
                FiveWay.Gray_Count_Error+=0.004;
                Limit_double_Value(&FiveWay.Gray_Count_Error,2.4,3.4);
            }
            //黑线越出检测范围，此时车身偏右
            else if (FiveWay.Yaw_Error < -34) 
				FiveWay.Gray_Count_Error+=0.01;
        }
        /************车身偏左************/
        else if (FiveWay.Yaw_Error > 0)
        {
            //黑线位于R1与R2之间,此时车身偏左
            if (FiveWay.Yaw_Error > 12 && FiveWay.Yaw_Error < 22)
            {
                FiveWay.Gray_Count_Error-=0.004;
                Limit_double_Value(&FiveWay.Gray_Count_Error,-3.4,-2.4);
            }
            //黑线越出检测范围，此时车身偏左
            else if (FiveWay.Yaw_Error > 34)
				FiveWay.Gray_Count_Error-=0.01;
        }
        break;

    /*当黑线正好在灰度传感器下时*/
//    case 0x01://00001 - 0x01
//        //仅有一个传感器检测到黑线
//        //黑线在Gray_R2下，此时车身偏左
//        FiveWay.Gray_Count_Error-=0.005;
//        Limit_double_Value(&FiveWay.Gray_Count_Error,-4.8,-3.4);
//        break;

    case 0x02://00010 - 0x02
        //仅有一个传感器检测到黑线
        //黑线在Gray_R1下，此时车身偏左
		FiveWay.Gray_Count_Error-=0.002;
        Limit_double_Value(&FiveWay.Gray_Count_Error,-2.3,-1.2);
        break;

    case 0x06://00110 - 0x06
        //有两个传感器检测到黑线
        //黑线位于M与R1之间,此时车身偏左
        FiveWay.Gray_Count_Error-=0.001;
        Limit_double_Value(&FiveWay.Gray_Count_Error,-1.2,-0.7);
        break;

    case 0x04://00100 - 0x04
        //仅有一个传感器检测到黑线
        //黑线在Gray_M下，此时车身位于中间
        Limit_double_Value(&FiveWay.Gray_Count_Error,-0.7,0.7);
		FiveWay.Gray_Count_Error = 0;
        //更新目标yaw
        FiveWay.Mid_yaw = Yaw;
        //不对起始位置识别做消抖
        FiveWay.Debounce_count = 0;
        break;

    case 0x0C://01100 - 0x0C
        //有两个传感器检测到黑线
        //黑线位于L1和M之间，此时车身偏右
        FiveWay.Gray_Count_Error+=0.001;
        Limit_double_Value(&FiveWay.Gray_Count_Error,0.7,1.2);
        break;

    case 0x08://01000 - 0x8
        //仅有一个传感器检测到黑线
        //黑线在Gray_L1下，此时车身偏右
		FiveWay.Gray_Count_Error+=0.002;
        Limit_double_Value(&FiveWay.Gray_Count_Error,1.2,2.3);
        break;

//    case 0x10://10000 - 0x10
//        //仅有一个传感器检测到黑线
//        //黑线在Gray_L2下，此时车身偏右
//        FiveWay.Gray_Count_Error+=0.005;
//        Limit_double_Value(&FiveWay.Gray_Count_Error,3.4,4.8);
//        break;

/*************************停止线识别部分*************************/
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
 * 修改巡线圈数
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

/* 存放扫描到的地址 */
uint8_t scan_addr[128] = {0};
volatile uint8_t count;
uint8_t ping_response;
uint8_t gray_sensor[8];
uint8_t digital_data;

struct
{
	uint8_t Target;//目标状态,只能是三段
    uint8_t Turn_times;
    double offset;
}GWGray;

/* 设置软件IIC驱动 */
sw_i2c_interface_t i2c_interface = 
{
	.sda_in = sda_in,
	.scl_out = scl_out,
	.sda_out = sda_out,
	.user_data = 0, //用户数据，可在输入输出函数里得到
};

/*****
 * 移植于感为8路灰度传感器驱动-软件IIC(可根据需要自行改为硬件IIC)
 * 使用IO口：带5V容忍的PF0(SDA)、PF1(SCL)
*****/
void GraySensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //GPIOE、GPIOF时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

    //PF0、PF1输出、开漏、100MHz、上拉
    GPIO_InitStruct.GPIO_Pin    = IIC2_SDA|IIC2_SCL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_UP;
    GPIO_Init(GPIOF,&GPIO_InitStruct);

    GPIO_SetBits(GPIOF,IIC2_SDA|IIC2_SCL);

	/* 第一次IIC通讯会失败（因为软件IIC触发了start），手动发个stop也能消除 */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	/* 后面IIC通讯是正常的 */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	
	/* 扫描开始 */
	count = i2c_scan(&i2c_interface, scan_addr);

    GWGray.Target = 0xE7;
    GWGray.Turn_times = 0;
}



float GraySensor_LinePatrol(void)
{
	uint32_t i;
    uint8_t Segments = 1;//段数
    uint8_t Segments_Length[9]={0};//每段的长度
    uint8_t L_Target = 0;
    uint8_t R_Target = 0;

    //读取开关量数据
    sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data 有1~8号探头开关数据
    //把字节里的8个开关量存到八个变量里，这里为gray_sensor[0] ~ gray_sensor[7],也可以是变量val1 ~ val8, 因为是宏定义
    SEP_ALL_BIT8(digital_data, 
        gray_sensor[0], //探头1
        gray_sensor[1], //探头2
        gray_sensor[2], //探头3
        gray_sensor[3], //探头4
        gray_sensor[4], //探头5
        gray_sensor[5], //探头6
        gray_sensor[6], //探头7
        gray_sensor[7]  //探头8
    );

    //digital_data 的最低位是探头1的数据，digital_data 的最高位是探头8的数据
    //数据位：8   7   6   5   4   3   2   1
    //探头号：8   7   6   5   4   3   2   1
    //偏移位：R3  R2  R1  R0  L0  L1  L2  L3
/*************************处理数据*************************/
    //遍历开关量数据，得到段数、段长，之后就可以通过探头1的颜色得到每个段的颜色和长
    for (i = 0; i < 7; i++)
    {
        //处理实际开关量数据
        if (gray_sensor[i] != gray_sensor[i+1])
            Segments++;
        Segments_Length[Segments]++;
    }
    //修正遍历的数据
    Segments_Length[1]+=1;
    //处理目标开关量数据
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
/*************************循迹部分*************************/
    //计算偏移量
    if (gray_sensor[0]==1 && Segments == 3)
    {
        //使用一个即可
        offset_alculation(Segments_Length[1]-L_Target);
        // offset_alculation(Segments_Length[3]-R_Target);
    }
    //没有偏移量时更新中心位置
    if ((Segments_Length[1]-L_Target) == 0 && (Segments_Length[3]-R_Target) == 0)
    {
        GWGray.offset = 0;
    }
    //PID计算
	return 0;
}

/*****
 * 针对（Segments_Length[1]-L_Target）计算偏差，如果使用的是（Segments_Length[3]-R_Target），需要修改
*****/
void offset_alculation(int8_t Gray_offset)
{
    switch (Gray_offset)
    {
        /************车身偏左************/
        case 3:GWGray.offset-=0.005;Limit_double_Value(&GWGray.offset,-3.2,-2.5);break;
        case 2:GWGray.offset-=0.004;Limit_double_Value(&GWGray.offset,-2.5,-1.2);break;
        case 1:GWGray.offset-=0.003;Limit_double_Value(&GWGray.offset,-1.2,-0.4);break;
        /************车身偏右************/
        case  -1:GWGray.offset+=0.003;Limit_double_Value(&GWGray.offset,0.4,1.2);break;
        case  -2:GWGray.offset+=0.004;Limit_double_Value(&GWGray.offset,1.2,2.5);break;
        case  -3:GWGray.offset+=0.005;Limit_double_Value(&GWGray.offset,2.5,3.2);break;
    }
}



/* 定义sda输出函数 bit=0为低电平 bit=1为高电平 */
void sda_out(uint8_t bit, void *user_data)
{
    SDA_OUT;

	GPIO_WriteBit(GPIOF, IIC2_SDA, (BitAction)bit);
	
	/* IIC软件延迟 */
	delay_us(10);
}

/* 定义sda读取函数 bit 为返回的电平值 */
uint8_t sda_in(void *user_data)
{
	uint8_t bit;
    
    SDA_IN;

	bit = (uint8_t)GPIO_ReadInputDataBit(GPIOF, IIC2_SDA);
	
	/* IIC软件延迟 */
	delay_us(10);
	return bit;
}

/* 定义scl时钟输出函数 bit=0为低电平 bit=1为高电平 */
void scl_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(GPIOF, IIC2_SCL, (BitAction)bit);
	
	/* IIC软件延迟 */
	delay_us(10);
}

/**
 * i2c地址扫描
 * @param scan_addr 扫描出来的地址存放,数值不为0的为扫描到的地址，扫到的地址会挨个放在数组的最前面
 * @return 返回扫描到的设备数量, 0为无设备发现
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


