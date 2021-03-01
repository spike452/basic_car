/*
 * EM_init.c
 *
 *  Created on: 2021年2月22日
 *      Author: Spike
 */
#include "headfile.h"
#include "ele.h"
#pragma section all "cpu0_dsram"

float message[7][5]={0};//存储电感值
int fuhao1[7]={0};
float change[7][5]={0};//存增减量

uint16 ELE_data[7][10];//电感值
float ELE_Value[7];//存储十次平均后的电感值
float ELE_num[7];
float ELE_PREVIOUS[7][3];//加权滤波时的存储值
float ELE_compensate[7];

float Circle_Max;

float ELE_L_MAX=0.926;//左边水平   ####100/108
float ELE_M_MAX=0.667;//中间
float ELE_R_MAX=0.926;//右边水平
float ELE_HL_MAX=0.926;//左斜
float ELE_HR_MAX=0.926;//右斜


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
