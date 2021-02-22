/*
 * PID.c
 *
 *  Created on: 2021年2月22日
 *      Author: Spike
 */
#include "PID.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"

int L_SpeedArray[5]={0,0,0,0,0},R_SpeedArray[5]={0,0,0,0,0};//读取编码器的数据
float encoder_l,encoder_r;  //滤波后编码器的读数
float avert_speed=0.0;//左右轮平均速度
float L_Speed,R_Speed;//左右轮实测速度
float perimeter=0.1885f;//后轮一圈的周长
float cycle=0.005f;//0.005f;//周期
int wheel_pulse=2260;//512线：1175;//后轮转一圈编码器的脉冲数
//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机舵机初始化
//  @param      void
//  @return     void
//  @note       注意电机和舵机的初始化频率
//-------------------------------------------------------------------------------------------------------------------
void Motor_init(void)
{
//    uart_init(USART_1,115200,UART1_TX_B6,UART1_RX_B7);    //串口初始用来看脉冲


           gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6); //编码器初始化
           gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDA_P00_9);

           gtm_pwm_init(ATOM0_CH1_P33_9, 50,3570); //舵机初始化

        //电机初始化
           gtm_pwm_init(ATOM0_CH0_P21_2, 17000,0);//ATOM 0模块的通道0 使用P21_2引脚输出PWM  PWM频率50HZ  占空比百分之0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX宏定义在zf_gtm_pwm.h
           gtm_pwm_init(ATOM0_CH1_P21_3, 17000,0);
           gtm_pwm_init(ATOM0_CH2_P21_4, 17000,0);
           gtm_pwm_init(ATOM0_CH3_P21_5, 17000,0);

}

//  @brief      获取编码器脉冲数
//  @param      void
//  @return     void
//  @note       实际速度=（实际编码器脉冲数*后轮周长）/（后轮转一圈编码器脉冲数*周期）
//-------------------------------------------------------------------------------------------------------------------
void encoder_get(void)
{
/////////////////////平均滤波/////////////////////////////
//        L_SpeedArray[1]=L_SpeedArray[0];
//        R_SpeedArray[1]=R_SpeedArray[0];
//        L_SpeedArray[0] = -(qtimer_quad_get(QTIMER_2,QTIMER2_TIMER0_C0));
//        R_SpeedArray[0] = qtimer_quad_get(QTIMER_2,QTIMER2_TIMER2_C2);
//        encoder_l = (L_SpeedArray[0]+L_SpeedArray[1])/2;
//        encoder_r = (R_SpeedArray[0]+R_SpeedArray[1])/2;
////////////////////////////////////////////////////////////
////////////////////加权滤波///////////////////////////////
          L_SpeedArray[4]=L_SpeedArray[3];
          L_SpeedArray[3]=L_SpeedArray[2];
          L_SpeedArray[2]=L_SpeedArray[1];
          L_SpeedArray[1]=L_SpeedArray[0];
          R_SpeedArray[4]=R_SpeedArray[3];
          R_SpeedArray[3]=R_SpeedArray[2];
          R_SpeedArray[2]=R_SpeedArray[1];
          R_SpeedArray[1]=R_SpeedArray[0];

          R_SpeedArray[0] = gpt12_get(GPT12_T2);
          L_SpeedArray[0]=-(gpt12_get(GPT12_T2));

          encoder_l=(L_SpeedArray[0]*5.0+L_SpeedArray[1]*4.0+L_SpeedArray[2]*3.0+L_SpeedArray[3]*2.0+L_SpeedArray[4])/15.0f;
          encoder_r=(R_SpeedArray[0]*5.0+R_SpeedArray[1]*4.0+R_SpeedArray[2]*3.0+R_SpeedArray[3]*2.0+R_SpeedArray[4])/15.0f;
///////////////////////////////////////////////////////////

          L_Speed=(encoder_l*perimeter)/((float)(wheel_pulse*cycle));
          R_Speed=(encoder_r*perimeter)/((float)(wheel_pulse*cycle));
          avert_speed=(L_Speed+R_Speed)/2.0f;

          gpt12_clear(GPT12_T2);
          gpt12_clear(GPT12_T4);
}

