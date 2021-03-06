/*
 * Kalman_Filter.c
 *
 *  Created on: 2020年7月27日
 *      Author: 23062
 */
#include <math.h>
#include "Kalman_Filter.h"


void kalman_init(kalman_typedef* kalman)
{
	kalman->lastTimePredVal = 0;		//上次估计值
	kalman->lastTimePredCovVal = 0.1;	//上次估计协方差
	kalman->lastTimeRealCovVal = 0.1;	// 上次实际协方差
	kalman->kg = 0;
}

// val: 本次测量值
float kalman_calc(kalman_typedef* kalman, float val)
{
    float currPredVal = 0;                             // 本次估计值
    float currRealVal = val;                           // 本次实际值
    float currPredCovVal = kalman->lastTimePredCovVal;         // 本次估计协方差值
    float currRealCovVal = kalman->lastTimeRealCovVal;         // 本次实际协方差值

    // 计算本次估计值，并更新保留上次预测值的变量
    currPredVal = kalman->lastTimePredVal + kalman->kg * (currRealVal - kalman->lastTimePredVal);
    kalman->lastTimePredVal = currPredVal;

    //计算卡尔曼增益
    kalman->kg = sqrt(pow(kalman->lastTimePredCovVal, 2) / (pow(kalman->lastTimePredCovVal, 2) + pow(kalman->lastTimeRealCovVal, 2)));

    // 计算下次估计和实际协方差
    kalman->lastTimePredCovVal = sqrt(1.0 - kalman->kg) * currPredCovVal;
    kalman->lastTimeRealCovVal = sqrt(1.0 - kalman->kg) * currRealCovVal;

    // 返回本次的估计值,也就是滤波输出值
    return currPredVal;
}



/******************************************************卡尔曼滤波************************************************/
static float angle,angle_dot; 		//外部需要引用的变量
static const float Q_angle=0.04,//0.04,  //0.001
                   Q_gyro=0.005,//0.05, //0.003
                   R_angle=0.05,//0.05,  //0.05
                   dt=0.005;      //定时器5ms
			//注意：dt的取值为kalman滤波器采样时间;
static float P[2][2] = {
							{ 1, 0 },
							{ 0, 1 }
						};
static float Pdot[4] ={0,0,0,0};
static const char C_0 = 1;
static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
void Kalman_Filter(float angle_m,float gyro_m)			//gyro_m:gyro_measure
{
	angle+=(gyro_m-q_bias) * dt;          //先验估计

	Pdot[0]=Q_angle - P[0][1] - P[1][0];  //先验估计误差协方差的微分
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;

	P[0][0] += Pdot[0] * dt;    //先验估计误差协方差的积分=先验估计误差协方差
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	angle_err = angle_m - angle;  //先验估计

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;    //后验估计误差协方差
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;

	angle	+= K_0 * angle_err;  //后验估计
	q_bias	+= K_1 * angle_err;  //后验估计
	angle_dot = gyro_m-q_bias;   //输出值(后验估计)的微分=角速度
}
