/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#pragma section all "cpu0_dsram"

uint16 adc_result;

int core0_main(void)
{
	disableInterrupts();
	get_clk();//获取时钟频率  务必保留
	adc_init(ADC_0, ADC0_CH0_A0);//初始化ADC0 通道0 使用A0引脚
    //用户在此处调用各种初始化函数等
	gtm_pwm_init(ATOM0_CH4_P02_4, 50,    0);//ATOM 0模块的通道4 使用P02_4引脚输出PWM  PWM频率50HZ  占空比百分之0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX宏定义在zf_gtm_pwm.h
	gtm_pwm_init(ATOM0_CH5_P02_5, 1000,  0);
	gtm_pwm_init(ATOM0_CH6_P02_6, 10000, 0);
	gtm_pwm_init(ATOM0_CH7_P02_7, 30000, 0);

	gtm_pwm_init(ATOM1_CH7_P00_8, 30000, 5000);
	//每个通道都可以输出不同频率的PWM

	pwm_duty(ATOM0_CH4_P02_4, 5000);//设置占空比为百分之5000/GTM_ATOM0_PWM_DUTY_MAX*100
	pwm_duty(ATOM0_CH5_P02_5, 5000);
	pwm_duty(ATOM0_CH6_P02_6, 5000);
	pwm_duty(ATOM0_CH7_P02_7, 5000);

    enableInterrupts();

    while (TRUE)
    {
        //通过将A0引脚接到GND或者核心板VCC  查看FSS窗口中打印出来的ADC结果
                //A0接到GND时 结果应该在0附近
                //A0接到VCC时 结果应该在4095附近
                //悬空时 结果可能是任意数，没有任何意义，所以大家不要问悬空的时候为什么结果是这么多呢？
                adc_result = adc_mean_filter(ADC_0, ADC0_CH0_A0, ADC_12BIT, 10);//采集10次求平均  分辨率12位
                //将结果通过串口打印，可以先学习printf例程，了解如何使用printf
                printf("adc_result: %d\n", adc_result);
                systick_delay_ms(STM0, 100);
    }
}

#pragma section all restore
