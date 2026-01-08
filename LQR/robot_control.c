#include "robot_control.h"


static float v_integral = 0;
const float K[4] = {-73.90768f, -12.73487f, -20.00000f, -16.17029f};


/**
 * @brief 四元�? -> 欧拉�? (Pitch/Roll)，并给出 Pitch 角速度
 * @param sensor_data  传感器数�?（q[] 四元数、gyro[] 角速度�?
 * @param robot_state  机器人状态（输出：pitch/roll/pitch_vel�?
 * @note  pitch_vel 这里直接取陀螺仪某一轴（gyro[1]），轴向需根据 IMU 安�?�方向确�?
 */
void Update_Euler_from_Quat(Robot_Sensors_t *sensor_data, Robot_State_t *robot_state)
{
    // 读取四元�? (w, x, y, z)
    float w = sensor_data->q[0];
    float x = sensor_data->q[1];
    float y = sensor_data->q[2];
    float z = sensor_data->q[3];

    // 计算�?仰�?? (pitch)
    robot_state->pitch = atan2f(2.0f * (w * y - x * z), 1.0f - 2.0f * (y * y + z * z));
    // 计算�?滚�?? (roll)
    robot_state->roll = asinf(2.0f * (w * x + y * z)); 
    //角速度反�?�项
    robot_state->pitch_vel = sensor_data->gyro[1];

    //机�?�安装�??�?补偿
    //需要实际情况继�?调试
    const float pitch_offset = 0.698906904f * PI / 180.0f;
    robot_state->pitch -= pitch_offset;
}


/**
 * @brief 正向运动学�?�算 (FK)
 * @param robot_state 指向机器人状态结构体的指�?
 * @param alpha 前大腿电机�?? (rad)
 * @param beta  后大腿电机�?? (rad)
 * @param height  输出：腿部“高�?/长度”（以底盘中点为参考）
 * @param theta1  输出：中间连杆�?? theta1 (rad)
 * @param theta2  输出：中间连杆�?? theta2 (rad)
 * @note  用于从电机�?��?�算腿部当前几何形态（高度），�? VMC/高度控制使用
 */
void Calculate_FK(float alpha, float beta, float *height, float *theta1, float *theta2)
{
    // A,C点坐�?
    float xa = L1 * cosf(alpha);
    float ya = L1 * sinf(alpha);
    float xc = L5 + L4 * cosf(beta);
    float yc = L4 * sinf(beta);

    //AC距�??
    float Lac_sq = powf(xc - xa, 2) + powf(yc - ya, 2);

    //�?间变�?
    float a = 2.0f * (xa - xc) * L2;
    float b = 2.0f * (ya - yc) * L2;
    float c = L3 * L3 - L2 * L2 - Lac_sq;

    //求解theta (预防根号小于0)
    float delta = a*a + b*b - c*c;
    *theta1 = 2.0f * atan2f(b + sqrtf(fmaxf(delta, 0.0f)), a + c);

    //足部�?�?
    float bx = xa + L2 * cosf(*theta1);
    float by = ya + L2 * sinf(*theta1);

    *theta2 = atan2f(yc - by, xc - bx);

    //计算腿部高度
    // 如果重心在底盘中点，CoM_x_offset = L5 / 2.0f
    *height = sqrtf(powf(bx - L5/2.0f, 2) + powf(by, 2));

}

/**
 * @brief Jacobian 映射（VMC：将期望竖直�? Fy 映射到两电机力矩�?
 * @param alpha  前电机�??
 * @param beta   后电机�??
 * @param theta1 FK 解出来的�?间�?? theta1
 * @param theta2 FK 解出来的�?间�?? theta2
 * @param Fy     期望竖直�?撑力（VMC 输出�?
 * @param tf     输出：前电机力矩
 * @param tb     输出：后电机力矩
 */
void Jacobian_Map(float alpha, float beta, float theta1, float theta2, float Fy, float *tf, float *tb)
{
    float detJ = sinf(theta1 - theta2);
    if (fabsf(detJ) < 0.0001f) detJ = 0.0001f; 

    *tf = (L1 * sinf(theta2) * sinf(alpha - theta1) / detJ) * Fy; 
    *tb = (L4 * sinf(theta1) * sinf(beta - theta2) / detJ) * Fy;  

}

/**
 * @brief 机器人主控制�?�?：IMU姿态解�? + 腿部FK + VMC高度/速度 + LQR平衡 + 力矩分配
 * @param sensor_data  输入：传感器数据（包�?腿电机�?�、速度等）
 * @param robot_state  输出/更新：机器人状态（姿态�?�、高度等�?
 * @param motor_output 输出：最终下发到电机的力矩（�?/�?�?
 * @param target_h     �?标腿部高�?
 * @param target_v     �?标前进速度
 * @note  IMU 数据来自 dm_imu 全局变量 imu，这里每�?控制周期同�?�一次到 sensor_data
 */
void Robot_Control_Loop(Robot_Sensors_t *sensor_data,Robot_State_t *robot_state, Motor_Output_t *motor_output, float target_h, float target_v)
{
    //更新IMU数据
    sensor_data->q[0] = imu.q[0];
    sensor_data->q[1] = imu.q[1]; 
    sensor_data->q[2] = imu.q[2];
    sensor_data->q[3] = imu.q[3];

    sensor_data->gyro[0] = imu.gyro[0];
    sensor_data->gyro[1] = imu.gyro[1]; 
    sensor_data->gyro[2] = imu.gyro[2];

    //姿态解�?
    Update_Euler_from_Quat(sensor_data, robot_state);

    //运动学解�?
    Calculate_FK(sensor_data->alpha_L, sensor_data->beta_L, &robot_state->h_L, &robot_state->theta1_L, &robot_state->theta2_L);
    Calculate_FK(sensor_data->alpha_R, sensor_data->beta_R, &robot_state->h_R, &robot_state->theta1_R, &robot_state->theta2_R);

    //VMC:速度PI控制
    float v_error = target_v - sensor_data->x_vel;
    v_integral = fmaxf(-0.5f, fminf(0.5f, v_integral + v_error * 0.001f)); //�?分限�?
    float th_offset = 0.5f * v_error + 0.1f * v_integral;  //这里�?0.5 �?0.1 �?需要后期调试的参数

    //VMC:高度控制Fy
    float Fy_L = 150.0f * (target_h - robot_state->h_L) + TOTAL_MASS * G;
    float Fy_R = 150.0f * (target_h - robot_state->h_R) + TOTAL_MASS * G;

    //LQR
    // 这里选择不取信于x�? K[2] * x项�?�为0
    // 正确�? LQR: u = -(k1*theta + k2*theta_dot + k3*x + k4*x_dot)
    float tau_bal = -(K[0] * (robot_state->pitch - th_offset) + 
                      K[1] * robot_state->pitch_vel + 
                      K[2] * 0 + // 不取信于位移
                      K[3] * sensor_data->x_vel); 

    //雅可比映射到电机力矩
    Jacobian_Map(sensor_data->alpha_L, sensor_data->beta_L, robot_state->theta1_L, robot_state->theta2_L, Fy_L, &motor_output->leg_LF, &motor_output->leg_LB);
    Jacobian_Map(sensor_data->alpha_R, sensor_data->beta_R, robot_state->theta1_R, robot_state->theta2_R, Fy_R, &motor_output->leg_RF, &motor_output->leg_RB);

    //�?子力矩分�?
    motor_output->wheel_L = -tau_bal / 2.0f;
    motor_output->wheel_R = tau_bal / 2.0f;
}