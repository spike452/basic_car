/*
 * PID.c
 *
 *  Created on: 2021��2��22��
 *      Author: Spike
 */
#include "PID.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"

int L_SpeedArray[5]={0,0,0,0,0},R_SpeedArray[5]={0,0,0,0,0};//��ȡ������������
float encoder_l,encoder_r;  //�˲���������Ķ���
float avert_speed=0.0;//������ƽ���ٶ�
float L_Speed,R_Speed;//������ʵ���ٶ�
float perimeter=0.1885f;//����һȦ���ܳ�
float cycle=0.005f;//0.005f;//����
int wheel_pulse=2260;//512�ߣ�1175;//����תһȦ��������������
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������ʼ��
//  @param      void
//  @return     void
//  @note       ע�����Ͷ���ĳ�ʼ��Ƶ��
//-------------------------------------------------------------------------------------------------------------------
void Motor_init(void)
{
//    uart_init(USART_1,115200,UART1_TX_B6,UART1_RX_B7);    //���ڳ�ʼ����������


           gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6); //��������ʼ��
           gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDA_P00_9);

           gtm_pwm_init(ATOM0_CH1_P33_9, 50,3570); //�����ʼ��

        //�����ʼ��
           gtm_pwm_init(ATOM0_CH0_P21_2, 17000,0);//ATOM 0ģ���ͨ��0 ʹ��P21_2�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
           gtm_pwm_init(ATOM0_CH1_P21_3, 17000,0);
           gtm_pwm_init(ATOM0_CH2_P21_4, 17000,0);
           gtm_pwm_init(ATOM0_CH3_P21_5, 17000,0);

}

//  @brief      ��ȡ������������
//  @param      void
//  @return     void
//  @note       ʵ���ٶ�=��ʵ�ʱ�����������*�����ܳ���/������תһȦ������������*���ڣ�
//-------------------------------------------------------------------------------------------------------------------
void encoder_get(void)
{
/////////////////////ƽ���˲�/////////////////////////////
//        L_SpeedArray[1]=L_SpeedArray[0];
//        R_SpeedArray[1]=R_SpeedArray[0];
//        L_SpeedArray[0] = -(qtimer_quad_get(QTIMER_2,QTIMER2_TIMER0_C0));
//        R_SpeedArray[0] = qtimer_quad_get(QTIMER_2,QTIMER2_TIMER2_C2);
//        encoder_l = (L_SpeedArray[0]+L_SpeedArray[1])/2;
//        encoder_r = (R_SpeedArray[0]+R_SpeedArray[1])/2;
////////////////////////////////////////////////////////////
////////////////////��Ȩ�˲�///////////////////////////////
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

