/**
  ******************************************************************************
  * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
  * @file    DiffCalculator.h
  * @author  张至睿
  * @brief   巴特沃斯二阶低通滤波器
  * @date    2023-07-01
  * @version 1.0
  *
  ==============================================================================
					        How to use this library 
  ==============================================================================
    @note
			- 构造函数输入：截止频率，采样频率
            - 使用SecondOrderButterworthLPF::f(float input)进行滤波
  	@warning  
  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _SECOND_BUTTERWORTH_LPF_H
#define _SECOND_BUTTERWORTH_LPF_H
#include "math.h"

#ifdef __cplusplus
#define M_PI 3.141592653589793f

//二阶巴特沃斯低通滤波器
class SecondOrderButterworthLPF {
public:
    // 构造函数初始化：截止频率，采样频率
    SecondOrderButterworthLPF(float cutoff_freq, float sampling_freq) {
        wc = 2.f * M_PI * cutoff_freq / sampling_freq;
        calculateCoefficients();
        reset();
    }

    // 对于输入信号进行滤波处理
    float f(float input) {
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }

    // Reset the filter state
    void reset() {
        x1 = 0.f;
        x2 = 0.f;
        y1 = 0.f;
        y2 = 0.f;
    }

private:
    float wc; // 截止频率
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;

    // 计算滤波器参数
    void calculateCoefficients() {
        float sqrt2 = sqrtf(2.f);
        float sqrt2wc = sqrtf(2.f) * wc;
        float wc2 = wc * wc;

        b0 = wc2 / (wc2 + sqrt2wc + 1.f);
        b1 = 2.f * b0;
        b2 = b0;
        a1 = 2.f * (wc2 - 1.f) / (wc2 + sqrt2wc + 1.f);
        a2 = (wc2 - sqrt2wc + 1.f) / (wc2 + sqrt2wc + 1.f);
    }
};
#endif  /* __cplusplus */

#endif  /* _SECOND_BUTTERWORTH_LPF_H */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

