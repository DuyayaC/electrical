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
} MahonyAHRS;

extern MahonyAHRS mahony;

extern void MahonyAHRSInit(float Kp, float Ki, MahonyAHRS *mahony);
extern void MahonyAHRSUpdateIMU(float dt, MahonyAHRS *mahony);
extern void Get_Angle1(MahonyAHRS *mahony);
void normalize(MahonyAHRS* mahony, uint32_t size);
extern void MahonyAHRS_Reset();

#endif
//=====================================================================================================