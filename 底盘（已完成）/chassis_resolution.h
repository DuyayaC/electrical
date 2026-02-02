#include "Robot_Property.h"
#include "arm_math.h"

#if (CHASSIS_TYPE == 'MECANUM')
#define WHEEL_RADIUS
#define GEAR_RATIO 19.20320856f
#define dx 
#define dy 
#endif

#if (CHASSIS_TYPE == 'OMNI')
#define WHEEL_TO_CORE_DISTANCE 0.22873f
#define WHEEL_RADIUS 0.075f
#define GEAR_RATIO 19.20320856f
#endif

typedef struct 
{   
    float chassis_gimbal_deltatheta; 
    float wheel_target_omega[4];
    float chassis_omega; 
    float chassis_target_omega;
    float chassis_target_vx;
    float chassis_target_vy;
    float motor_target_omega[4];
    float motor_output_value[4];
}chassis_motion_value_t __attribute__((aligned(4)));

extern chassis_motion_value_t chassis_motion;

extern void chassis_coordinate_resolution(float target_vx, float target_vy, chassis_motion_value_t *chassis_motion);
extern void gimbal_coordinate_resolution(float target_vx, float target_vy, float target_omega, chassis_motion_value_t *chassis_motion);
extern void top_stand_still(float chassis_target_omega, chassis_motion_value_t *chassis_motion);
extern void top_moving(float gimbal_target_vx, float gimbal_target_vy, float chassis_target_omega, float theta, chassis_motion_value_t *chassis_motion);
