/*
 * PID.c
 *
 *  Created on: 2021年2月22日
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
     int skp;//停车用
     float ski;//停车用
     float kd;
     int bang_error_max;//bangbang阈值
     int ki_error_max;//积分分离阈值
     int ki_value_max;//积分限幅
     int motor_value_max;//pwm限幅
 }Pid_t;

 typedef struct
  {
      float speed;//目标速度
      Pid_t speed_pid;//电机PID有关参数
      float EDS_Coefficient_Out;//差速（外轮）
      float EDS_Coefficient_In;//差速（内轮）
      int16 Fuzzy_Servo_Rule[7][7];//模糊表
      int16 Fuzzy_Servo_Rule_1[7][7];
      int16 Fuzzy_Servo_Rule_2[7][7];
      int16 Fuzzy_Servo_Rule_3[7][7];
  }Speed;




 float encoder_l,encoder_r;  //滤波后编码器的读数
 float avert_speed=0.0;//左右轮平均速度
 int L_SpeedArray[5]={0,0,0,0,0},R_SpeedArray[5]={0,0,0,0,0};//读取编码器的数据
 float L_Speed,R_Speed;//左右轮实测速度
 float L_error,R_error;//左右轮实测速度与预设速度的偏差

 uint8 brake_flag=0; //点刹标志位
 float perimeter=0.1885f;//后轮一圈的周长
 float cycle=0.005f;//0.005f;//周期
 int wheel_pulse=2260;//512线：1175;//后轮转一圈编码器的脉冲数

 int L_ki_value=0,R_ki_value=0;//电机ki值
 int L_kp_value=0,R_kp_value=0;//电机kp值
 int L_kd_value=0,R_kd_value=0;//电机kd值

 float speedl_set=0.0,speedr_set=0.0;//差速处理后的预设速度

 //阿克曼角差速相关变量
 float EDS_Coefficient_Out,EDS_Coefficient_In;
 float angle_t=0.0;
 float Dif=10.0f;
 float Dif_array[3]={0.0,0.0,0.0};
 float B_w=15.5;
 float L_l=21;
 float tann=0.0;

 //电机输出pwm
 int L_out=0;
 int R_out=0;

 uint8 speed_run_mode=1;//车的跑动模式
 Speed *thisRunMode;//当前车的跑动模式
 float initial_speed=0.0;//初始时的预设速度

 //微分跟踪
 float LTrack=0,LDif_Track=0;
 float RTrack=0,RDif_Track=0;
 void Dif_Tracker(float *u1,float *u2,float error);

 //速度调节器相关参数
 float last_final_speed=0.0f,step_speed=0.0f,last_step_speed=0.0f;
 uint8 count_time=0;

 //电机PID参数临时变量
 int temp_kp;
 float temp_ki,temp_kd;

 //特定元素速度预设值（坡道，连续S，R80，环岛，检测到斑马线时，出库，直道加速）
 float podao_speed=2.3f,SS_speed=3.35f,R80_speed=3.50f,Circle_speed=3.0f,stop_speed=1.8f,outpark_speed=3.0f,straight_speed=3.6f;
 //特定元素的PI参数
 int R80_kp=475;
 float R80_ki=2.8f;

 //特定元素的差速参数（连续S，环岛，停车）
 float eds_ss=0.00f,eds_circle=0.10f,eds_stop=0.95f;
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
//  @brief      左轮位置式PI控制
//  @param      void
//  @return     void
//  @note       bangbang算法+积分分离+跟踪微分器+PI
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
             Dif_Tracker(&LTrack,&LDif_Track,L_error);//微分
             if(L_error>PID.ki_error_max||L_error<-PID.ki_error_max || (podao_flag==1 && L_error<-30)|| (podao_flag==1 && L_error>30)) //积分分离
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
//  @brief      右轮位置式PI控制
//  @param      void
//  @return     void
//  @note       bangbang算法+积分分离+跟踪微分器+PI
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
             Dif_Tracker(&RTrack,&RDif_Track,R_error);//微分
             if(R_error>PID.ki_error_max||R_error<-PID.ki_error_max|| (podao_flag==1 && L_error<-30)|| (podao_flag==1 && L_error>30)) //积分分离
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
