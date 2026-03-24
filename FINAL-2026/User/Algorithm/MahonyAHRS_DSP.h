#include "arm_math.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
//=====================================================================================================

#define PI (3.14159265f)

#ifndef MahonyAHRS_H
#define MahonyAHRS_H

typedef struct
{
    float twoKp;          // 2 * proportional gain (Kp)
    float twoKi;          // 2 * integral gain (Ki)
    float q[4];// quaternion of sensor frame relative to auxiliary frame
    float integralFB[3];    // integral error terms scaled by Ki; XYZ
    float halfv[3]; //XYZ
    float halfe[3]; //XYZ
    float g[3];
    float a[3];
    float YAW;
    float PITCH;
    float ROLL;
    float sampleFreq;
    float dt;
		float g_smc[3]; //SMC计算用角速度数据
} MahonyAHRS;

extern MahonyAHRS mahony;

extern void MahonyAHRSInit(float Kp, float Ki, float dt, MahonyAHRS *mahony);
extern void MahonyAHRSUpdateIMU(MahonyAHRS *mahony);
extern void Get_Angle1(MahonyAHRS *mahony);
extern void MahonyAHRS_Reset(MahonyAHRS *mahony);
extern float invsqrt_DSP(float x);

#endif
//=====================================================================================================