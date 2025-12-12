#include "ZeroCheck.h"

void zerocheckecddji(float *target, float *now)
{
    float target = *target;
    float now = *now;
    
    float diff = target - now;

    if (diff > 4096.0f)  //半圈编码器读数: 8192 / 2
    {
        now += 8192;
    }
    else if (diff < -4092.0f)
    {
        now -= 8192;
    }
}

void zerocheckangle(float *target_angle, float *now_angle)
{
    float target = *target_angle;
    float now = *now_angle;

    float diff = target - now;

    if (diff > 180.0f)
    {
        *target_angle -= 360.0f; 
    }
    else if (diff < -180.0f)
    {
        *target_angle += 360.0f;
    }
}

void zerocheckecd8115(float *target, float *now)
{
    float target = *target;
    float now = *now;
    
    float diff = target - now;

    if (diff > 4096.0f)  //半圈编码器读数: 8192 / 2
    {
        now += 8192;
    }
    else if (diff < -4092.0f)
    {
        now -= 8192;
    }
}