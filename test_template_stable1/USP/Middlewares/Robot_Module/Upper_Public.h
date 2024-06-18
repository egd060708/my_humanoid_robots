#ifndef _UPPER_PUBLIC_H_
#define _UPPER_PUBLIC_H_

/*lqr线性拟合参数*/
typedef struct _Fit_Params
{
    float a = 0;//三次项
    float b = 0;//二次项
    float c = 0;//一次项
    float d = 0;//常数项
}Fit_Params;

namespace upper
{
    template<typename F, typename F1>
    F constrain(F input, F1 threshold)
    {
        if (threshold < 0)
            threshold = -threshold;

        if (input <= -threshold)
            return -threshold;
        else if (input >= threshold)
            return threshold;

        return input;
    }

    template<typename F, typename F1>
    F constrain(F input, F1 threshold_1, F1 threshold_2)
    {
        F min, max;
        if (threshold_1 > threshold_2)
        {
            max = threshold_1;
            min = threshold_2;
        }
        else
        {
            min = threshold_1;
            max = threshold_2;
        }

        if (input <= min)
            return min;
        else if (input >= max)
            return max;
        return input;
    }
}



#endif
