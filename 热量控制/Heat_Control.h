#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "dvc_referee.h" // 引入裁判系统头文件

/* 机器人兵种定义 */
typedef enum {
    ROBOT_HERO = 0,     // 英雄机器人 (42mm)
    ROBOT_INFANTRY,     // 步兵机器人 (17mm)
    ROBOT_SENTRY        // 哨兵机器人 (17mm)
} RobotType_e;

/* 步兵机器人专属：发射机构类型配置 (初始兜底用) */
typedef enum {
    INFANTRY_HEAT_MODE_BURST = 0, // 爆发优先模式
    INFANTRY_HEAT_MODE_COOLING    // 冷却优先模式
} InfantryHeatMode_e;

/* 拨弹轮状态机 */
typedef enum {
    HEAT_STOP = 0,       // 停机
    HEAT_PRECHECK,       // 准备检测
    HEAT_SUSPECT,        // 发射嫌疑
    HEAT_CONFIRM         // 确认发射
} HeatState_e;

/* 热量预测与校准控制结构体 */
typedef struct {
    HeatState_e state;
    RobotType_e robot_type;

    // 热量相关数据
    float Qnow;             // 本地预测当前热量 (核心依赖)
    float Qmax;             // 最大热量上限 (从裁判系统动态读取)
    float Qcd;              // 冷却速度 (从裁判系统动态读取)
    
    // 本地预测检测参数
    float idle_time;        // 距离上次本地判定发弹的时间 (用于触发裁判系统校准)
    float suspect_time;     // 电流持续超过阈值的时间
    float fire_heat;        // 每发子弹增加的热量（17mm=10, 42mm=100）
    float current_th;       // 判定“推弹”的电流阈值（如 1.4A）
    float dt;               // 控制周期 (秒)
} HeatPredict_t;

/**
 * @brief  热量预测初始化
 * @param  hp: 热量预测结构体指针
 * @param  type: 机器人兵种 (英雄/步兵/哨兵)
 * @param  default_infantry_mode: 若为步兵，初始假设模式；后续会被裁判系统真实数据覆盖
 * @param  dt: 任务运行周期（单位：秒，例如 1ms = 0.001f）
 * @param  current_th: 拨弹电机电流检测阈值
 */
void HeatPredict_Init(HeatPredict_t* hp, RobotType_e type, InfantryHeatMode_e default_infantry_mode, float dt, float current_th);

/**
 * @brief  【核心】热量预测与裁判系统数据融合更新
 * @param  hp: 热量预测结构体指针
 * @param  ref: 裁判系统结构体指针 (将提供真实的上限、冷却和当前热量)
 * @param  wheel_current: 拨弹电机当前电流 (A)
 * @param  wheel_rpm: 拨弹电机当前转速 (RPM)
 * @param  target_rpm: 拨弹电机目标转速 (RPM)
 */
void HeatPredict_Update(HeatPredict_t* hp, Referee_System_t* ref, float wheel_current, float wheel_rpm, float target_rpm);

/**
 * @brief  根据当前融合后的热量预测，计算未来安全射速
 * @param  hp: 热量预测结构体指针
 * @param  desire_rate: 操作手期望射速 (发/秒)
 * @param  max_rate: 物理极限射速 (发/秒)
 * @retval 安全的指令射速 (发/秒)，将此值转化为拨弹轮的目标转速即可
 */
float HeatPredict_ComputeSafeFireRate(const HeatPredict_t* hp, float desire_rate, float max_rate);

#ifdef __cplusplus
}
#endif

#endif /* HEAT_CONTROL_H */
