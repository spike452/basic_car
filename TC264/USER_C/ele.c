/*
 * EM_init.c
 *
 *  Created on: 2021��2��22��
 *      Author: Spike
 */
#include "headfile.h"
#include "ele.h"
#pragma section all "cpu0_dsram"


typedef struct {
    float offset;     //ƫ����
    float road;
} PathInfo_t;

PathInfo_t path;//�ܵ�����Ϣ
uint8 R_80_flag=0;//R80��־λ
uint16 ELE_data[7][10];//���ֵ
uint16 ELE_max[7][6];//�洢��е����ֵ
int fuhao_path;//���ڴ洢���6�ε�ƫ�����ı仯
float path_message[6]={0};
float path_change[6]={0};//�洢ƫ�����ı仯
uint32  Path_Distance;
uint16  path_left_top=0;
uint16  path_right_top=0;
uint16  path_left_bottom=0;
uint16  path_right_bottom=0;
uint8 Road_date=0;
uint8   zhidao_num=10;
uint8 RightTurn_number=0;//������¼�������
uint8 LeftTurn_number=0;//������¼�������
uint8 RightTurn_number_aftercrossroad=0;//������¼ʮ�ֺ��������
uint8 LeftTurn_number_aftercrossroad=0;//������¼ʮ�ֺ��������
float Circle_Max;
float message[7][5]={0};//�洢���ֵ
int fuhao1[7]={0};
float change[7][5]={0};//��������
uint8 max_num;
uint8 min_num;
uint8 last_max_num;
uint8 d_max_num;
float crossroad_Yaw_init=0;
float ELE_Value[7];//�洢ʮ��ƽ����ĵ��ֵ
float ELE_PREVIOUS[7][3];//��Ȩ�˲�ʱ�Ĵ洢ֵ
float ELE_compensate[7];
float ELE_num[7];
float ELE_FINAL[5];//��������ˮƽ��б��еļӳ�
int ELE_Circle_Max=0;//Բ����з��ֵ
uint8 encoder_L=0;
uint8 encoder_R=0;
int16   podao_flag=0;
uint8 After_ramp=0;//�����µ���
uint8 After_circle=0;//����Բ����
uint8   STOP_flag;//ͣ����־λ
uint16  stop_car_begin_time;//����һ��ʱ���ڲ���ʶ��
uint32  Stop_distance;
uint8   Track_flag;
uint8   ten_flag;
uint8   huankou_n;//��¼���ﻷ�ڵĴ���
float   Sum_huan=3;//��������
float   huan_Big1=0;//������λ��
float   huan_Big2=5;
float   huan_Big3=6;
uint8   huan_flag;
uint32  huan_system;
uint8   Circle_big_speed;
uint8 crossroad_count=0;//ʮ�ּ���
float EM_speed_temp=-1.0f;
int EM_out=-1;
uint8 speed_down_flag=0;
uint8 offset_add_flag=0;
uint8 EM_outparking_flag=0;//�������
//************��Ż�������***************//
int EM_circle_flag=0;//��Ż�����־
int EM_encoder=0;//�������������ַ�ֹ�ٴ��뻷
float EM_turn_angle_init=0.0f; //ת�ǻ��ֽ�����ʼ�ο�ֵ
float EM_turn_angle=0.0f;//��ת�ǻ���
float EM_podao_angle_init;  //�µ���ʼ������
float EM_podao_angle;  //�µ�������
uint8 EM_circle_direction=0;//��������1Ϊ��2Ϊ�ң�����Ϊ0
uint8 EM_circle_data;//��ȡ�����·������������ʹ�С
/*****************************************/

//////////////////////////////////////////////////////////////////////////////////////////
float ELE_L_MAX=0.926;//���ˮƽ   ####100/108
float ELE_M_MAX=0.667;//�м�
float ELE_R_MAX=0.926;//�ұ�ˮƽ
float ELE_HL_MAX=0.926;//��б
float ELE_HR_MAX=0.926;//��б
//�ο���ѹ
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
//                                          �������ֵ��ȡ
//����ֵ��  ��
//����˵����
//*************************************************************************************************************
void Findmax(void)
{
    if(ELE_compensate[2]>=Circle_Max)
        Circle_Max=ELE_compensate[2];
}

//*************************************************************************************************************
//                                          ���ģ���ʼ��
//����ֵ��  ��
//����˵����
//*************************************************************************************************************
void EM_init(void) //
{
    adc_init(ADC_0, ADC0_CH0_A0);//��1
    adc_init(ADC_0, ADC0_CH1_A1);//��2
    adc_init(ADC_0, ADC0_CH2_A2);//��1
    adc_init(ADC_0, ADC0_CH3_A3);//��2
    adc_init(ADC_0, ADC0_CH4_A4);//��

    //�����Ȳɼ�һ��
    int i,j,temp;
    //5��ADͨ���ɼ����
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
//----------------���ֵ��һ��----------------------//
    ELE_num[0]=ELE_Value[0]*ELE_HL_MAX;
    ELE_num[1]=ELE_Value[1]*ELE_L_MAX;
    ELE_num[4]=ELE_Value[2]*ELE_M_MAX;
    ELE_num[2]=ELE_Value[3]*ELE_R_MAX;
    ELE_num[3]=ELE_Value[4]*ELE_HR_MAX;

    //�洢֮ǰ�ĵ��ֵ
    for(j=0;j<5;j++)
    {
        for(i=0;i<2;i++)
         {ELE_PREVIOUS[j][i]=ELE_PREVIOUS[j][i+1];}

        ELE_PREVIOUS[j][2]=ELE_num[j];
    }

    //��Ȩ�˲�
    for(j=0;j<5;j++)
    {ELE_compensate[j]=0.1*ELE_PREVIOUS[j][0]+0.1*ELE_PREVIOUS[j][1]+0.8*ELE_PREVIOUS[j][2];}

    ELE_Store();  //�洢���ֵ��������ֵ�ı仯������
}


void ELE_Store(void)
{
    uint8 i=0,j=0;
    uint8 fuhaoz[6]={0};
    uint8 fuhaof[6]={0};
//--------���ֵ����message����,����6�����µ��ֵ----------------//
//ֻ�����м��е�ֵ
    for(i=4;i<5;i++)
    {
        for(j=4;j>0;j--)
        {
            message[i][j+1]=message[i][j];
        }

        message[i][1]=message[i][0];
        message[i][0]=ELE_compensate[i];
    }
//--------������ֵ��������С��fuhaoz�������������fuhaof�����С����-----------//
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
    //----------�жϵ��������С��5�ε��ֵ�仯�У���3����������Ϊ������4�����ϼ�СΪ��С������Ϊ����------------//
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
//                                          ��ȡ���ֵ����
//����ֵ����
//����˵������ȡһ��adcת����ֵ
//����Ų�˳��0  1    4     2    3
//*************************************************************************************************************
void ELE_get(void) //
{
    for(int i=0;i<5;i++)
    {
        ELE_data[0][i]=adc_convert(ADC_0, ADC0_CH0_A0, ADC_12BIT);//��1
        ELE_data[1][i]=adc_convert(ADC_0, ADC0_CH1_A1, ADC_12BIT);//��2
        ELE_data[2][i]=adc_convert(ADC_0, ADC0_CH2_A2, ADC_12BIT);//��2
        ELE_data[3][i]=adc_convert(ADC_0, ADC0_CH3_A3, ADC_12BIT);//��1
        ELE_data[4][i]=adc_convert(ADC_0, ADC0_CH4_A4, ADC_12BIT);//��
    }
}

//*************************************************************************************************************
//                                              ������ֵ�洢����
//����ֵ��Ϊ������е����ֵ
//����˵��������ǰ������������е����ֵ��Ϊ��������һ������
//�ó�ELE_HL_MAXΪ��������һ�λ�ȡ�����ֵ������
//*************************************************************************************************************
void ELE_Standard(void)
{
    uint8 i=0,j=0,k=0;
    float temp=0;
//5��ADͨ���ɼ����
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
        //����ѭ����ELE_max���������Ӵ�С��
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
    ELE_HL_MAX=100.0/ELE_max[0][1];     //��б
    ELE_L_MAX =100.0/ELE_max[1][1];     //���ˮƽ
    ELE_M_MAX =100.0/ELE_max[4][1]; //�м�
    ELE_R_MAX =100.0/ELE_max[2][1];     //�ұ�ˮƽ
    ELE_HR_MAX=100.0/ELE_max[3][1]; //��б
//  ELE_SR_MAX=0;
}
