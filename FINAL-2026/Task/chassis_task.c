#include "chassis_task.h"

PID_t chassis[4];
chassis_motion_value_t chassis_motion;
extern VTM_Data_t key;

static void theta_calculate(chassis_motion_value_t *chassis_motion);
static void output_map(float *value);
static void km_control(void);
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion);
static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion);
static void rc_control(void);

void CHASSIS_TASK(void const * argument)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;
    PID_Clear_Error(&chassis[0]);
    PID_Clear_Error(&chassis[1]);
    PID_Clear_Error(&chassis[2]);
    PID_Clear_Error(&chassis[3]);

    PID_Init(&chassis[0], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[1], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[2], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[3], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
		
	  chassis_motion.gimbal_target_vx = 0.0f;
		chassis_motion.gimbal_target_vy = 0.0f;
		chassis_motion.chassis_target_omega = 0.0f;
    while(1)
    {
					chassis_motion.now_ecd = get_yaw_gimbal_motor_measure_point()->ecd;
					theta_calculate(&chassis_motion);
					
					for (int i = 0; i < 4; i++)
					{
							chassis_motion.wheel_now_omega[i] = get_chassis_motor_measure_point(i)->speed_rpm * 0.1047f;
					}
					if (key.sw == 1)
					{
						km_control();
					}
					else if (key.sw == 0)
					{
						rc_control();
					}
			
					gimbal_coordinate_resolution(chassis_motion.theta, &chassis_motion);
			if ((chassis_motion.chassis_target_vx == 0) && (chassis_motion.chassis_target_vy == 0) && (chassis_motion.chassis_target_omega == 0))
			{
				CAN_cmd_chassis(0, 0, 0, 0);
			}

			else
			{
				PID_Set_Values(&chassis[0], chassis_motion.wheel_target_omega[0], chassis_motion.wheel_now_omega[0]);
				PID_Set_Values(&chassis[1], chassis_motion.wheel_target_omega[1], chassis_motion.wheel_now_omega[1]);
				PID_Set_Values(&chassis[2], chassis_motion.wheel_target_omega[2], chassis_motion.wheel_now_omega[2]);
				PID_Set_Values(&chassis[3], chassis_motion.wheel_target_omega[3], chassis_motion.wheel_now_omega[3]);

				PID_Calculate(&chassis[0], &chassis_motion.motor_output_value[0]);
				PID_Calculate(&chassis[1], &chassis_motion.motor_output_value[1]);
				PID_Calculate(&chassis[2], &chassis_motion.motor_output_value[2]);
				PID_Calculate(&chassis[3], &chassis_motion.motor_output_value[3]);

				output_map(&chassis_motion.motor_output_value[0]);
				output_map(&chassis_motion.motor_output_value[1]);
				output_map(&chassis_motion.motor_output_value[2]);
				output_map(&chassis_motion.motor_output_value[3]);
					CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
								(int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
			}
				osDelayUntil(&xLastWakeTime,xFrequency);
		}
			    
}

static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion)
{   
    float32_t sin_theta, cos_theta;
    arm_sin_cos_f32(theta, &sin_theta, &cos_theta); 
    chassis_motion->chassis_target_vx = cos_theta * chassis_motion->gimbal_target_vx - sin_theta * chassis_motion->gimbal_target_vy;
    chassis_motion->chassis_target_vy = sin_theta * chassis_motion->gimbal_target_vx + cos_theta * chassis_motion->gimbal_target_vy;
    chassis_wheel_inverse_resolution(chassis_motion);
}

static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //1
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //2
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //3
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //4
}

static void theta_calculate(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->theta = (chassis_motion->now_ecd - 6128.0f) * 360.0f / 8192.0f;
}

static void output_map(float *value)
{
    *value *= 819.2f; 
}

static void km_control(void)
{
	  chassis_motion.gimbal_target_vx = (key.key_w - key.key_s) * 1.5f;
    chassis_motion.gimbal_target_vy = (key.key_a - key.key_d) * 1.5f;
    chassis_motion.chassis_target_omega = key.key_shift * 2 * PI;
}

static void rc_control(void)
{
	chassis_motion.gimbal_target_vx = (key.ch0 - 1024) / 660.0f * 1.0f;
	chassis_motion.gimbal_target_vy = (1024 - key.ch1) / 660.0f * 1.0f;
	chassis_motion.chassis_target_omega = key.pause* 2 * PI;
}