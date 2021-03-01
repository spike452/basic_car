/*
 * PID.c
 *
 *  Created on: 2021��2��22��
 *      Author: Spike
 */
#include "PID.h"
#include "ele.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"

 typedef struct
 {
     int kp;
     float ki;
     int skp;//ͣ����
     float ski;//ͣ����
     float kd;
     int bang_error_max;//bangbang��ֵ
     int ki_error_max;//���ַ�����ֵ
     int ki_value_max;//�����޷�
     int motor_value_max;//pwm�޷�
 }Pid_t;

 typedef struct
  {
      float speed;//Ŀ���ٶ�
      Pid_t speed_pid;//���PID�йز���
      float EDS_Coefficient_Out;//���٣����֣�
      float EDS_Coefficient_In;//���٣����֣�
      int16 Fuzzy_Servo_Rule[7][7];//ģ����
      int16 Fuzzy_Servo_Rule_1[7][7];
      int16 Fuzzy_Servo_Rule_2[7][7];
      int16 Fuzzy_Servo_Rule_3[7][7];
  }Speed;




 float encoder_l,encoder_r;  //�˲���������Ķ���
 float avert_speed=0.0;//������ƽ���ٶ�
 int L_SpeedArray[5]={0,0,0,0,0},R_SpeedArray[5]={0,0,0,0,0};//��ȡ������������
 float L_Speed,R_Speed;//������ʵ���ٶ�
 float L_error,R_error;//������ʵ���ٶ���Ԥ���ٶȵ�ƫ��

 uint8 brake_flag=0; //��ɲ��־λ
 float perimeter=0.1885f;//����һȦ���ܳ�
 float cycle=0.005f;//0.005f;//����
 int wheel_pulse=2260;//512�ߣ�1175;//����תһȦ��������������

 int L_ki_value=0,R_ki_value=0;//���kiֵ
 int L_kp_value=0,R_kp_value=0;//���kpֵ
 int L_kd_value=0,R_kd_value=0;//���kdֵ

 float speedl_set=0.0,speedr_set=0.0;//���ٴ�����Ԥ���ٶ�

 //�������ǲ�����ر���
 float EDS_Coefficient_Out,EDS_Coefficient_In;
 float angle_t=0.0;
 float Dif=10.0f;
 float Dif_array[3]={0.0,0.0,0.0};
 float B_w=15.5;
 float L_l=21;
 float tann=0.0;

 //������pwm
 int L_out=0;
 int R_out=0;

 uint8 speed_run_mode=1;//�����ܶ�ģʽ
 Speed *thisRunMode;//��ǰ�����ܶ�ģʽ
 float initial_speed=0.0;//��ʼʱ��Ԥ���ٶ�

 //΢�ָ���
 float LTrack=0,LDif_Track=0;
 float RTrack=0,RDif_Track=0;
 void Dif_Tracker(float *u1,float *u2,float error);

 //�ٶȵ�������ز���
 float last_final_speed=0.0f,step_speed=0.0f,last_step_speed=0.0f;
 uint8 count_time=0;

 //���PID������ʱ����
 int temp_kp;
 float temp_ki,temp_kd;

 //�ض�Ԫ���ٶ�Ԥ��ֵ���µ�������S��R80����������⵽������ʱ�����⣬ֱ�����٣�
 float podao_speed=2.3f,SS_speed=3.35f,R80_speed=3.50f,Circle_speed=3.0f,stop_speed=1.8f,outpark_speed=3.0f,straight_speed=3.6f;
 //�ض�Ԫ�ص�PI����
 int R80_kp=475;
 float R80_ki=2.8f;

 //�ض�Ԫ�صĲ��ٲ���������S��������ͣ����
 float eds_ss=0.00f,eds_circle=0.10f,eds_stop=0.95f;
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
          L_SpeedArray[0]=-(gpt12_get(GPT12_T4));

          encoder_l=(L_SpeedArray[0]*5.0+L_SpeedArray[1]*4.0+L_SpeedArray[2]*3.0+L_SpeedArray[3]*2.0+L_SpeedArray[4])/15.0f;
          encoder_r=(R_SpeedArray[0]*5.0+R_SpeedArray[1]*4.0+R_SpeedArray[2]*3.0+R_SpeedArray[3]*2.0+R_SpeedArray[4])/15.0f;
///////////////////////////////////////////////////////////

          L_Speed=(encoder_l*perimeter)/((float)(wheel_pulse*cycle));
          R_Speed=(encoder_r*perimeter)/((float)(wheel_pulse*cycle));
          avert_speed=(L_Speed+R_Speed)/2.0f;

          gpt12_clear(GPT12_T2);
          gpt12_clear(GPT12_T4);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����λ��ʽPI����
//  @param      void
//  @return     void
//  @note       bangbang�㷨+���ַ���+����΢����+PI
//-------------------------------------------------------------------------------------------------------------------
void Left_control(Pid_t PID)
{
       L_error=speedl_set-encoder_l;
       if(L_error>PID.bang_error_max)//BANG-BANG
         {
             L_out=PID.motor_value_max;
         }
         else if(L_error<-PID.bang_error_max)
         {
             L_out=-PID.motor_value_max;
         }
         else
         {
             L_kp_value=L_error*temp_kp;
             Dif_Tracker(&LTrack,&LDif_Track,L_error);//΢��
             if(L_error>PID.ki_error_max||L_error<-PID.ki_error_max || (podao_flag==1 && L_error<-30)|| (podao_flag==1 && L_error>30)) //���ַ���
             {
                 L_ki_value=0;
             }
             else
             {
                 L_ki_value+=L_error*temp_ki;
               if(L_ki_value>=PID.ki_value_max)  L_ki_value=PID.ki_value_max;
               if(L_ki_value<=-PID.ki_value_max)  L_ki_value=-PID.ki_value_max;
             }
             L_out=L_kp_value+L_ki_value+LDif_Track*PID.kd;
             if(L_out>=PID.motor_value_max) L_out=PID.motor_value_max;
             if(L_out<=-PID.motor_value_max) L_out=-PID.motor_value_max;
         }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����λ��ʽPI����
//  @param      void
//  @return     void
//  @note       bangbang�㷨+���ַ���+����΢����+PI
//-------------------------------------------------------------------------------------------------------------------
void Right_control(Pid_t PID)
{
       R_error=speedr_set-encoder_r;
       if(R_error>PID.bang_error_max)//BANG-BANG
         {
             R_out=PID.motor_value_max;
         }
         else if(R_error<-PID.bang_error_max)
         {
             R_out=-PID.motor_value_max;
         }
         else
         {
             R_kp_value=R_error*temp_kp;
             Dif_Tracker(&RTrack,&RDif_Track,R_error);//΢��
             if(R_error>PID.ki_error_max||R_error<-PID.ki_error_max|| (podao_flag==1 && L_error<-30)|| (podao_flag==1 && L_error>30)) //���ַ���
             {
                 R_ki_value=0;
             }
             else
             {
                 R_ki_value+=R_error*temp_ki;
               if(R_ki_value>=PID.ki_value_max)  R_ki_value=PID.ki_value_max;
               if(R_ki_value<=-PID.ki_value_max)  R_ki_value=-PID.ki_value_max;
             }
             R_out=R_kp_value+R_ki_value+RDif_Track*PID.kd;
             if(R_out>=PID.motor_value_max) R_out=PID.motor_value_max;
             if(R_out<=-PID.motor_value_max) R_out=-PID.motor_value_max;
          }
}
