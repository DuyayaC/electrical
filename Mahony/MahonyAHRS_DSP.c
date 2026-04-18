#include "MahonyAHRS_DSP.h"

MahonyAHRS mahony;

float invsqrt_DSP(float x)
{
    float32_t result;
    arm_sqrt_f32(x, &result);
    return 1.0f / result;
}

void normalize(float32_t *vec, uint32_t size)
{
    float32_t norm;
    float32_t prod;

    arm_dot_prod_f32(vec, vec, size, &prod);
    arm_sqrt_f32(prod, &norm);
    if (norm > 0.0f)
    {
        arm_scale_f32(vec, 1.0f/norm, vec, size);
    }
}

void MahonyAHRSInit(float Kp, float Ki, MahonyAHRS *mahony)
{
    mahony->twoKp = 2.0f * Kp;
    mahony->twoKi = 2.0f * Ki;
		mahony->q[0] = 1.0f;
		mahony->q[1] = 0.0f;
		mahony->q[2] = 0.0f;
		mahony->q[3] = 0.0f;
}

void MahonyAHRSUpdateIMU(float dt, MahonyAHRS *mahony)
{
    float recipNorm;
    mahony->dt = dt;
    if (!((mahony->a[0] == 0.0f) && (mahony->a[1] == 0.0f) && (mahony->a[2] == 0.0f)))
    {
        normalize(mahony->a, 3);

        mahony->halfv[0] = mahony->q[1] * mahony->q[3] - mahony->q[0] * mahony->q[2];
        mahony->halfv[1] = mahony->q[0] * mahony->q[1] + mahony->q[2] * mahony->q[3];
        mahony->halfv[2] = mahony->q[0] * mahony->q[0] + mahony->q[3] * mahony->q[3] - 0.5f;
        
        mahony->halfe[0] = mahony->a[1] * mahony->halfv[2] - mahony->a[2] * mahony->halfv[1];
        mahony->halfe[1] = mahony->a[2] * mahony->halfv[0] - mahony->a[0] * mahony->halfv[2];
        mahony->halfe[2] = mahony->a[0] * mahony->halfv[1] - mahony->a[1] * mahony->halfv[0];

        if (mahony->twoKi > 0.0f)
        {
            mahony->integralFB[0] += mahony->twoKi * mahony->halfe[0] * mahony->dt;
            mahony->integralFB[1] += mahony->twoKi * mahony->halfe[1] * mahony->dt;
            mahony->integralFB[2] += mahony->twoKi * mahony->halfe[2] * mahony->dt;

            mahony->g[0] += mahony->integralFB[0];
            mahony->g[1] += mahony->integralFB[1];
            mahony->g[2] += mahony->integralFB[2];
        }
        else
        {
            mahony->integralFB[0] = 0.0f;
            mahony->integralFB[1] = 0.0f;
            mahony->integralFB[2] = 0.0f;
        }

        mahony->g[0] += mahony->twoKp * mahony->halfe[0];
        mahony->g[1] += mahony->twoKp * mahony->halfe[1];
        mahony->g[2] += mahony->twoKp * mahony->halfe[2];
    }

    mahony->g[0] *= (0.5f * mahony->dt);
    mahony->g[1] *= (0.5f * mahony->dt);
    mahony->g[2] *= (0.5f * mahony->dt);
    mahony->q[0] += (-mahony->q[1] * mahony->g[0] - mahony->q[2] * mahony->g[1] - mahony->q[3] * mahony->g[2]);
    mahony->q[1] += (mahony->q[0] * mahony->g[0] + mahony->q[2] * mahony->g[2] - mahony->q[3] * mahony->g[1]);
    mahony->q[2] += (mahony->q[0] * mahony->g[1] - mahony->q[1] * mahony->g[2] + mahony->q[3] * mahony->g[0]);
    mahony->q[3] += (mahony->q[0] * mahony->g[2] + mahony->q[1] * mahony->g[1] - mahony->q[2] * mahony->g[0]);

    normalize(mahony->q, 4);
}

void Get_Angle1(MahonyAHRS *mahony)
{
    mahony->YAW = atan2f((mahony->q[0] * mahony->q[3] + mahony->q[1] * mahony->q[2]), (mahony->q[0] * mahony->q[0] + mahony->q[1] * mahony->q[1] - 0.5f)) * 180.0f / PI;
    mahony->PITCH = asinf(2 * mahony->q[0] * mahony->q[2] - 2 * mahony->q[1] * mahony->q[3]) * 180.0f/PI;
    mahony->ROLL = atan2f((mahony->q[0] * mahony->q[1] + mahony->q[2] * mahony->q[3]), (mahony->q[0] * mahony->q[0] + mahony->q[3] * mahony->q[3] - 0.5f)) * 180.0f/PI;
}

void MahonyAHRS_Reset(MahonyAHRS *mahony) {
    float q_init[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float zeros3[3] = {0.0f, 0.0f, 0.0f};
    
    memcpy(mahony->q, q_init, sizeof(q_init));
    memcpy(mahony->integralFB, zeros3, sizeof(zeros3));
    memcpy(mahony->halfv, zeros3, sizeof(zeros3));
    memcpy(mahony->halfe, zeros3, sizeof(zeros3));
}