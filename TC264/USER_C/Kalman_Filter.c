/*
 * Kalman_Filter.c
 *
 *  Created on: 2020��7��27��
 *      Author: 23062
 */
#include <math.h>
#include "Kalman_Filter.h"


void kalman_init(kalman_typedef* kalman)
{
	kalman->lastTimePredVal = 0;		//�ϴι���ֵ
	kalman->lastTimePredCovVal = 0.1;	//�ϴι���Э����
	kalman->lastTimeRealCovVal = 0.1;	// �ϴ�ʵ��Э����
	kalman->kg = 0;
}

// val: ���β���ֵ
float kalman_calc(kalman_typedef* kalman, float val)
{
    float currPredVal = 0;                             // ���ι���ֵ
    float currRealVal = val;                           // ����ʵ��ֵ
    float currPredCovVal = kalman->lastTimePredCovVal;         // ���ι���Э����ֵ
    float currRealCovVal = kalman->lastTimeRealCovVal;         // ����ʵ��Э����ֵ

    // ���㱾�ι���ֵ�������±����ϴ�Ԥ��ֵ�ı���
    currPredVal = kalman->lastTimePredVal + kalman->kg * (currRealVal - kalman->lastTimePredVal);
    kalman->lastTimePredVal = currPredVal;

    //���㿨��������
    kalman->kg = sqrt(pow(kalman->lastTimePredCovVal, 2) / (pow(kalman->lastTimePredCovVal, 2) + pow(kalman->lastTimeRealCovVal, 2)));

    // �����´ι��ƺ�ʵ��Э����
    kalman->lastTimePredCovVal = sqrt(1.0 - kalman->kg) * currPredCovVal;
    kalman->lastTimeRealCovVal = sqrt(1.0 - kalman->kg) * currRealCovVal;

    // ���ر��εĹ���ֵ,Ҳ�����˲����ֵ
    return currPredVal;
}



/******************************************************�������˲�************************************************/
static float angle,angle_dot; 		//�ⲿ��Ҫ���õı���
static const float Q_angle=0.04,//0.04,  //0.001
                   Q_gyro=0.005,//0.05, //0.003
                   R_angle=0.05,//0.05,  //0.05
                   dt=0.005;      //��ʱ��5ms
			//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;
static float P[2][2] = {
							{ 1, 0 },
							{ 0, 1 }
						};
static float Pdot[4] ={0,0,0,0};
static const char C_0 = 1;
static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
void Kalman_Filter(float angle_m,float gyro_m)			//gyro_m:gyro_measure
{
	angle+=(gyro_m-q_bias) * dt;          //�������

	Pdot[0]=Q_angle - P[0][1] - P[1][0];  //����������Э�����΢��
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;

	P[0][0] += Pdot[0] * dt;    //����������Э����Ļ���=����������Э����
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	angle_err = angle_m - angle;  //�������

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;    //����������Э����
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;

	angle	+= K_0 * angle_err;  //�������
	q_bias	+= K_1 * angle_err;  //�������
	angle_dot = gyro_m-q_bias;   //���ֵ(�������)��΢��=���ٶ�
}
