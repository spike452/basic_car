/*
 * Kalman_Filter.h
 *
 *  Created on: 2020年7月27日
 *      Author: 23062
 */

#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H


typedef struct
{
    float lastTimePredVal;            // 上次估计值
    float lastTimePredCovVal;         // 上次估计协方差
    float lastTimeRealCovVal;         // 上次实际协方差
    float kg;
}kalman_typedef;



void kalman_init(kalman_typedef* kalman);


float kalman_calc(kalman_typedef* kalman, float val);

















#endif /* SRC_0_APPSW_TRICORE_APP_DRIVER_KALMAN_FILTER_H_ */
