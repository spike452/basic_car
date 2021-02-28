/*
 * Kalman_Filter.h
 *
 *  Created on: 2020��7��27��
 *      Author: 23062
 */

#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H


typedef struct
{
    float lastTimePredVal;            // �ϴι���ֵ
    float lastTimePredCovVal;         // �ϴι���Э����
    float lastTimeRealCovVal;         // �ϴ�ʵ��Э����
    float kg;
}kalman_typedef;



void kalman_init(kalman_typedef* kalman);


float kalman_calc(kalman_typedef* kalman, float val);

















#endif /* SRC_0_APPSW_TRICORE_APP_DRIVER_KALMAN_FILTER_H_ */
