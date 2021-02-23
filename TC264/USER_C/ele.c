/*
 * EM_init.c
 *
 *  Created on: 2021年2月22日
 *      Author: Spike
 */
#include "headfile.h"
#include "ele.h"
#pragma section all "cpu0_dsram"


typedef struct {
    float offset;     //偏移量
    float road;
} PathInfo_t;

PathInfo_t path;//跑道的信息
uint8 R_80_flag=0;//R80标志位
uint16 ELE_data[7][10];//电感值
uint16 ELE_max[7][6];//存储电感的最大值
int fuhao_path;//用于存储最近6次的偏移量的变化
float path_message[6]={0};
float path_change[6]={0};//存储偏移量的变化
uint32  Path_Distance;
uint16  path_left_top=0;
uint16  path_right_top=0;
uint16  path_left_bottom=0;
uint16  path_right_bottom=0;
uint8 Road_date=0;
uint8   zhidao_num=10;
uint8 RightTurn_number=0;//用来记录右弯个数
uint8 LeftTurn_number=0;//用来记录右弯个数
uint8 RightTurn_number_aftercrossroad=0;//用来记录十字后右弯个数
uint8 LeftTurn_number_aftercrossroad=0;//用来记录十字后右弯个数
float Circle_Max;
float message[7][5]={0};//存储电感值
int fuhao1[7]={0};
float change[7][5]={0};//存增减量
uint8 max_num;
uint8 min_num;
uint8 last_max_num;
uint8 d_max_num;
float crossroad_Yaw_init=0;
float ELE_Value[7];//存储十次平均后的电感值
float ELE_PREVIOUS[7][3];//加权滤波时的存储值
float ELE_compensate[7];
float ELE_num[7];
float ELE_FINAL[5];//用于左右水平与斜电感的加成
int ELE_Circle_Max=0;//圆环电感峰峰值
uint8 encoder_L=0;
uint8 encoder_R=0;
int16   podao_flag=0;
uint8 After_ramp=0;//出了坡道了
uint8 After_circle=0;//出了圆环了
uint8   STOP_flag;//停车标志位
uint16  stop_car_begin_time;//起跑一定时间内不会识别
uint32  Stop_distance;
uint8   Track_flag;
uint8   ten_flag;
uint8   huankou_n;//记录到达环口的次数
float   Sum_huan=3;//环的数量
float   huan_Big1=0;//大环所在位置
float   huan_Big2=5;
float   huan_Big3=6;
uint8   huan_flag;
uint32  huan_system;
uint8   Circle_big_speed;
uint8 crossroad_count=0;//十字计数
float EM_speed_temp=-1.0f;
int EM_out=-1;
uint8 speed_down_flag=0;
uint8 offset_add_flag=0;
uint8 EM_outparking_flag=0;//车库变量
//************电磁环岛变量***************//
int EM_circle_flag=0;//电磁环岛标志
int EM_encoder=0;//出环编码器积分防止再次入环
float EM_turn_angle_init=0.0f; //转角积分进环初始参考值
float EM_turn_angle=0.0f;//环转角积分
float EM_podao_angle_init;  //坡道初始俯仰角
float EM_podao_angle;  //坡道俯仰角
uint8 EM_circle_direction=0;//进环方向，1为左，2为右，否则为0
uint8 EM_circle_data;//获取拨码的路况，包含方向和大小
/*****************************************/

//////////////////////////////////////////////////////////////////////////////////////////
float ELE_L_MAX=0.926;//左边水平   ####100/108
float ELE_M_MAX=0.667;//中间
float ELE_R_MAX=0.926;//右边水平
float ELE_HL_MAX=0.926;//左斜
float ELE_HR_MAX=0.926;//右斜
//参考电压
int Ref_AD_L=65;
int Ref_AD_R=65;
//////////////////////////////////////////////////////////////////////////////////////////




float MY_Abs(float x)
{
    if(x>=0)
        return x;
    else
        return -x;
}

//*************************************************************************************************************
//                                          环岛最大值获取
//返回值：  无
//函数说明：
//*************************************************************************************************************
void Findmax(void)
{
    if(ELE_compensate[2]>=Circle_Max)
        Circle_Max=ELE_compensate[2];
}

//*************************************************************************************************************
//                                          电感模块初始化
//返回值：  无
//函数说明：
//*************************************************************************************************************
void EM_init(void) //
{
    adc_init(ADC_0, ADC0_CH0_A0);//左1
    adc_init(ADC_0, ADC0_CH1_A1);//左2
    adc_init(ADC_0, ADC0_CH2_A2);//右1
    adc_init(ADC_0, ADC0_CH3_A3);//右2
    adc_init(ADC_0, ADC0_CH4_A4);//中

    //以下先采集一次
    int i,j,temp;
    //5个AD通道采集电感
    ELE_get();
    for(j=0;j<5;j++)
    {
        for(i=0;i<5;i++)
        {
            temp+=ELE_data[j][i];
        }
        ELE_Value[j]=temp/5.0;
        temp=0;
    }
//----------------电感值归一化----------------------//
    ELE_num[0]=ELE_Value[0]*ELE_HL_MAX;
    ELE_num[1]=ELE_Value[1]*ELE_L_MAX;
    ELE_num[4]=ELE_Value[2]*ELE_M_MAX;
    ELE_num[2]=ELE_Value[3]*ELE_R_MAX;
    ELE_num[3]=ELE_Value[4]*ELE_HR_MAX;

    //存储之前的电感值
    for(j=0;j<5;j++)
    {
        for(i=0;i<2;i++)
         {ELE_PREVIOUS[j][i]=ELE_PREVIOUS[j][i+1];}

        ELE_PREVIOUS[j][2]=ELE_num[j];
    }

    //加权滤波
    for(j=0;j<5;j++)
    {ELE_compensate[j]=0.1*ELE_PREVIOUS[j][0]+0.1*ELE_PREVIOUS[j][1]+0.8*ELE_PREVIOUS[j][2];}

    ELE_Store();  //存储电感值并计算电感值的变化正反性
}


void ELE_Store(void)
{
    uint8 i=0,j=0;
    uint8 fuhaoz[6]={0};
    uint8 fuhaof[6]={0};
//--------电感值存入message数组,存入6个最新电感值----------------//
//只保存中间电感的值
    for(i=4;i<5;i++)
    {
        for(j=4;j>0;j--)
        {
            message[i][j+1]=message[i][j];
        }

        message[i][1]=message[i][0];
        message[i][0]=ELE_compensate[i];
    }
//--------计算电感值的增大或减小，fuhaoz存入增大次数，fuhaof存入减小次数-----------//
    for(i=4;i<5;i++)
    {
        for(j=0;j<5;j++)
        {
            change[i][j]=message[i][j]-message[i][j+1];
            if(change[i][j]>0)
                    fuhaoz[i]++;
            else if(change[i][j]<0)
                    fuhaof[i]++;
        }
    }
    //----------判断电感增大或减小，5次电感值变化中，有3次以上增大为增大，有4次以上减小为减小，其余为不变------------//
    for(i=4;i<5;i++)
    {
    if(fuhaoz[i]>=3)
        fuhao1[i]=1;
    else if(fuhaof[i]>=4)
        fuhao1[i]=-1;
    else
        fuhao1[i]=0;
    }
}

//*************************************************************************************************************
//                                          获取电感值函数
//返回值：无
//函数说明：获取一次adc转换的值
//电感排布顺序：0  1    4     2    3
//*************************************************************************************************************
void ELE_get(void) //
{
    for(int i=0;i<5;i++)
    {
        ELE_data[0][i]=adc_convert(ADC_0, ADC0_CH0_A0, ADC_12BIT);//左1
        ELE_data[1][i]=adc_convert(ADC_0, ADC0_CH1_A1, ADC_12BIT);//左2
        ELE_data[2][i]=adc_convert(ADC_0, ADC0_CH2_A2, ADC_12BIT);//右2
        ELE_data[3][i]=adc_convert(ADC_0, ADC0_CH3_A3, ADC_12BIT);//右1
        ELE_data[4][i]=adc_convert(ADC_0, ADC0_CH4_A4, ADC_12BIT);//中
    }
}

//*************************************************************************************************************
//                                              电感最大值存储函数
//返回值：为各个电感的最大值
//函数说明：开车前，将检测各个电感的最大值，为后续做归一化处理
//得出ELE_HL_MAX为这个电感这一次获取的五个值中最大的
//*************************************************************************************************************
void ELE_Standard(void)
{
    uint8 i=0,j=0,k=0;
    float temp=0;
//5个AD通道采集电感
    ELE_get();
    for(j=0;j<5;j++)
    {
        for(i=0;i<5;i++)
        {
            temp+=ELE_data[j][i];
        }
        ELE_Value[j]=temp/5.0;
        temp=0;
    }
        //下面循环后ELE_max内五个排序从大到小。
    for(j=0;j<5;j++)
    {
        for(i=0;i<5;i++)
        {
            if(ELE_Value[j]>=ELE_max[j][i])
            {
                if(ELE_Value[j]>ELE_max[j][i])
                {
                    for(k=4;k>i;k--)
                    {
                        ELE_max[j][k]=ELE_max[j][k-1];
                    }
                    ELE_max[j][i]=ELE_Value[j];
                }
                break;
            }
        }
    }
    ELE_HL_MAX=100.0/ELE_max[0][1];     //左斜
    ELE_L_MAX =100.0/ELE_max[1][1];     //左边水平
    ELE_M_MAX =100.0/ELE_max[4][1]; //中间
    ELE_R_MAX =100.0/ELE_max[2][1];     //右边水平
    ELE_HR_MAX=100.0/ELE_max[3][1]; //右斜
//  ELE_SR_MAX=0;
}
