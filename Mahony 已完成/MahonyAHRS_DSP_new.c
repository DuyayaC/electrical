#include "MahonyAHRS_DSP.h"

MahonyAHRS mahony;

/**
 * @brief 使用 DSP 库进行向量规格化
 */
void normalize(float32_t *vec, uint32_t size) {
    float32_t norm;
    arm_rms_f32(vec, size, &norm); // 均方根加速
    norm *= sqrtf(size); // 还原为模长
    
    if (norm > 0.0f) {
        arm_scale_f32(vec, 1.0f / norm, vec, size);
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

/**
 * @brief 使用 DSP 库加速 Mahony IMU 更新
 */
void MahonyAHRSUpdateIMU(float dt, MahonyAHRS *mahony) {
    mahony->dt = dt;
    
    // 1. 检查加速度计是否有有效数据
    if (!((mahony->a[0] == 0.0f) && (mahony->a[1] == 0.0f) && (mahony->a[2] == 0.0f))) {
        
        // 规格化加速度计向量
        normalize(mahony->a, 3);

        // 2. 计算参考重力方向 v = R(q)^T * [0, 0, 1]^T
        // 这里的矩阵运算展开如下，依然手动展开以减少函数调用开销
        mahony->halfv[0] = mahony->q[1] * mahony->q[3] - mahony->q[0] * mahony->q[2];
        mahony->halfv[1] = mahony->q[0] * mahony->q[1] + mahony->q[2] * mahony->q[3];
        mahony->halfv[2] = mahony->q[0] * mahony->q[0] + mahony->q[3] * mahony->q[3] - 0.5f;

        // 3. 计算误差 e = a x v (叉乘)
        // 使用手动展开或封装一个小函数，因为 CMSIS 没有直接提供 3D Cross Product
        mahony->halfe[0] = mahony->a[1] * mahony->halfv[2] - mahony->a[2] * mahony->halfv[1];
        mahony->halfe[1] = mahony->a[2] * mahony->halfv[0] - mahony->a[0] * mahony->halfv[2];
        mahony->halfe[2] = mahony->a[0] * mahony->halfv[1] - mahony->a[1] * mahony->halfv[0];

        // 4. 误差积分与反馈补偿
        if (mahony->twoKi > 0.0f) {
            float32_t ki_dt = mahony->twoKi * mahony->dt;
            float32_t temp_err[3];
            
            arm_scale_f32(mahony->halfe, ki_dt, temp_err, 3);
            arm_add_f32(mahony->integralFB, temp_err, mahony->integralFB, 3);
            arm_add_f32(mahony->g, mahony->integralFB, mahony->g, 3);
        } else {
            memset(mahony->integralFB, 0, sizeof(mahony->integralFB));
        }

        // Kp 补偿
        float32_t kp_gain[3];
        arm_scale_f32(mahony->halfe, mahony->twoKp, kp_gain, 3);
        arm_add_f32(mahony->g, kp_gain, mahony->g, 3);
    }

    // 5. 四元数微分方程 q_dot = 0.5 * q * omega
    // 将其转化为 4x4 矩阵运算形式 q = q + (q_dot * dt)
    float32_t gx = mahony->g[0] * 0.5f * mahony->dt;
    float32_t gy = mahony->g[1] * 0.5f * mahony->dt;
    float32_t gz = mahony->g[2] * 0.5f * mahony->dt;

    // 这里使用临时变量存储当前四元数
    float32_t qa = mahony->q[0], qb = mahony->q[1], qc = mahony->q[2], qd = mahony->q[3];

    // 四元数更新 (利用 DSP 库的向量加法/缩放可以，但此处线性组合更直接)
    mahony->q[0] += (-qb * gx - qc * gy - qd * gz);
    mahony->q[1] += (qa * gx + qc * gz - qd * gy);
    mahony->q[2] += (qa * gy - qb * gz + qd * gx);
    mahony->q[3] += (qa * gz + qb * gy - qc * gx);

    // 6. 再次规格化四元数
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