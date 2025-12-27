#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
	
typedef enum
{
	HEAT_MODE_BURST = 0, /*爆发优先模式*/
	HEAT_MODE_COOLING    /*冷却优先模式*/
}HeatMode;

	
	
typedef enum {
    HEAT_STOP = 0,       // 停机
    HEAT_PRECHECK,       // 准备检测
    HEAT_SUSPECT,        // 发射嫌疑
    HEAT_CONFIRM         // 确认发射
} HeatState;

typedef struct {
    HeatState state;
    HeatMode mode;

    float Qnow;             // 本地预测热量
	float Qmax;             // 最大热量
    float Qcd;              // 冷却速度（热量每秒下降多少）

    float suspect_time;     // 电流持续超过阈值的时间

    float fire_heat;        // 每发子弹增加的热量（17mm = 10）
    float current_th;       // 判定“推弹”的电流阈值（如 1.4A）
    float dt;                 // 控制周期 
} HeatPredict;

	
	
static float fc_scale_threshold(float value_base, float base_limit, float new_limit);
void HeatPredict_Init(HeatPredict* hp, HeatMode mode, float dt);
void HeatPredict_Update(HeatPredict* hp, float wheel_current, float wheel_rpm, float target_rpm);
float HeatPredict_ComputeFireRate(const HeatPredict* hp, float desire_rate, float max_rate);
	
	

#ifdef __cplusplus
}
#endif

#endif /* Heat_CONTROL_H */
