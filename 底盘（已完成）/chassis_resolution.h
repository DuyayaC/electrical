//CHASSIS_TYPE == 'MECANUM'
#define WHEEL_RADIUS
#define GEAR_RATIO 19.20320856f
#define dx //轮子中心到底盘x轴的长度
#define dy //轮子中心到底盘y轴的长度


//CHASSIS_TYPE == 'OMNI'
#define WHEEL_TO_CORE_DISTANCE 0.22873f //轮子中心到底盘中心的距离
#define WHEEL_RADIUS 0.075f
#define GEAR_RATIO 19.20320856f

typedef struct 
{   
    float gimbal_target_vx;
    float gimbal_target_vy;
    float wheel_target_omega[4]; 
    float chassis_target_omega;
    float chassis_target_vx;
    float chassis_target_vy;
    float motor_output_value[4];
	float wheel_now_omega[4];
	float now_ecd;
	float theta;
}__attribute__((aligned(4))) chassis_motion_value_t ;
extern chassis_motion_value_t chassis_motion;
