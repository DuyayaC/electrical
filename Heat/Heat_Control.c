#include "Heat_Control.h"
#include <math.h>
#include <string.h>



void HeatPredict_Init(HeatPredict* hp, HeatMode mode, float dt)
{
    hp->state = HEAT_STOP;
    hp->mode = mode;
    hp->dt = dt;

    hp->Qnow = 0.0f;

    hp->suspect_time = 0.0f;

    hp->fire_heat = 10.0f; // 17mm
    hp->current_th = 1.4f;
    
    //步兵的数据
    if (mode == HEAT_MODE_BURST)
    {
        hp->Qmax = 230.0f;
        hp->Qcd = 14.0f;
    }
    else
    {
        hp->Qmax = 88.0f;
        hp->Qcd = 24.0f;
    }
    
}

void HeatPredict_Update(HeatPredict* hp,
                        float wheel_current,
                        float wheel_rpm,
                        float target_rpm)
{
    const float dt = hp->dt;

    switch (hp->state)
    {
    case HEAT_STOP:
        if (wheel_rpm > target_rpm * 0.9f)
        {
            hp->state = HEAT_PRECHECK;
            hp->suspect_time = 0.0f;
        }
        break;

    case HEAT_PRECHECK:
    //电流超过阈值，怀疑发射，
        if (wheel_current >= hp->current_th)
        {
            hp->state = HEAT_SUSPECT;
            hp->suspect_time = 0.0f;
        }
        break;

    case HEAT_SUSPECT:
        if (wheel_current < hp->current_th)
        {
            hp->suspect_time += dt;
            //不适用于快速发射，如15发/s情况下检测
            if (hp->suspect_time > 0.020f){
                hp->state = HEAT_CONFIRM;
                hp->suspect_time = 0.0f;
            }
        }
        else
        {
             hp->suspect_time = 0.0f;
        }
        break;


    case HEAT_CONFIRM:
        hp->Qnow += hp->fire_heat;
        if (hp->Qnow > hp->Qmax)
        {
            hp->Qnow = hp->Qmax;
        }
        
        hp->state = HEAT_PRECHECK;
        hp->suspect_time = 0.0f;
        break;

    default:
        hp->state = HEAT_STOP;
        hp->suspect_time = 0.0f;
        break;
    }

    // 热量衰减
    hp->Qnow -= hp->Qcd * dt;
    if (hp->Qnow < 0.0f)
    {
        hp->Qnow = 0.0f;
    }
}

/*根据当前热量预测，未来1s内不会超过热量的最大安全射速*/
float HeatPredict_ComputeFireRate(const HeatPredict* hp,
                                  float desire_rate,
                                  float max_rate)
{
    const float d = hp->fire_heat; //每发子弹增加的热量
    const float Q = hp->Qnow;   //当前热量
    const float Qmax = hp->Qmax; //最大热量
    const float a = hp->Qcd;   //冷却速度
    const float horizon = 0.1f; //裁判解算周期

    if (Q >= Qmax - 1e-3f)
    {
        return 0.0f;
    }
    
    float r = desire_rate;
    if (r > max_rate)
    {
        r = max_rate;
    }
    if (r < 0.0f)
    {
        r = 0.0f;
    }

    if (d * r <= a)
    {
        return r;
    }
    float numerator = Qmax - Q + a * horizon;
    float denominator = d * horizon;
    
    float r_safe = 0.0f;
    if (denominator > 1e-6f)
    {
        r_safe = numerator / denominator;
    }
    if (r_safe < 0.0f)
    {
        r_safe = 0.0f;
    }
    if (r > r_safe )
    {
        r = r_safe;
    }
    return r;
    
    


    /*
    //剩余热量 > 200 :最大射速
    if (Qres >= 200.0f)
    {
        return max_rate;
    }
    //剩余热量在50--200 :线性下降到持续射速
    else if (Qres >= 50.0f)
    {
        float k = (Qres - 50.0f) / (200.0f - 50.0f);
        if (k <0)
        {
            k = 0;
        }
        if (k > 1)
        {
            k = 1;
        }
        float rate = sustain_rate + (max_rate - sustain_rate) * k;

        return rate;
    }

    //剩余热量在 20-50 :固定射速 = 冷却速度
    if (Qres >= 20.0f)
    {
        return sustain_rate;
    }
    
    //剩余热量 < 20 :禁止射击
    return 0.0f;
    */
}