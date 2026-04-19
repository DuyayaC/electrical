#include "heat_control.h"

RobotType_e type;
HeatPredict_t hp;

void HeatPredict_Init(HeatPredict_t* hp, RobotType_e type, InfantryHeatMode_e default_infantry_mode, float dt, float current_th)
{
    hp->state = HEAT_STOP;
    hp->robot_type = type;
    hp->dt = dt;
    
    hp->Qnow = 0.0f;
    hp->idle_time = 0.0f;
    hp->suspect_time = 0.0f;
    hp->current_th = current_th;

    /* 赋予初始默认值，防止比赛刚开始裁判系统还没连上时热量控制失效 */
    switch (type)
    {
        case ROBOT_HERO:
            hp->fire_heat = 100.0f; // 42mm
            hp->Qmax = 200.0f;      
            hp->Qcd = 24.0f;        
            break;

        case ROBOT_INFANTRY:
            hp->fire_heat = 10.0f;  // 17mm
            if (default_infantry_mode == INFANTRY_HEAT_MODE_BURST) {
                hp->Qmax = 230.0f; hp->Qcd = 14.0f;
            } else {
                hp->Qmax = 88.0f;  hp->Qcd = 24.0f;
            }
            break;

        case ROBOT_SENTRY:
            hp->fire_heat = 10.0f;  // 17mm
            hp->Qmax = 260.0f;      
            hp->Qcd = 30.0f;        
            break;
            
        default:
            hp->fire_heat = 10.0f;
            hp->Qmax = 88.0f;
            hp->Qcd = 24.0f;
            break;
    }
}

void HeatPredict_Update(HeatPredict_t* hp, Referee_System_t* ref, float wheel_current, float wheel_rpm, float target_rpm)
{
    const float dt = hp->dt;
    hp->idle_time += dt;

    /* ==================================================================
     * 1. 动态读取裁判系统参数 
     * ================================================================== */
    uint16_t ref_real_heat = 0;

    if (ref != NULL && ref->Referee_Status == Referee_Status_ENABLE)
    {
        // 读取裁判系统真实的枪口热量
        if (hp->robot_type == ROBOT_HERO) {
            ref_real_heat = ref->Robot_Power_Heat.shooter_42mm_barrel_heat;
        } else {
            ref_real_heat = ref->Robot_Power_Heat.shooter_17mm_barrel_heat;
        }
        /* ==================================================================
         * 2. 数据融合与信任校准逻辑
         * ================================================================== */
        
        // 规则A: 防漏检补齐。如果本地预测的热量居然比裁判系统还低，说明本地漏算了，立即拉高！
        if (hp->Qnow < (float)ref_real_heat) {
            hp->Qnow = (float)ref_real_heat;
        }

        // 规则B: 防误判消除。如果已经 0.25 秒没有检测到发弹了(裁判系统数据通常延迟 0.1秒)，
        // 则认为裁判系统的数据已经是最新的稳态数据。强制将本地热量同步为裁判系统真实热量，消除本地积分误差。
        if (hp->idle_time > 0.30f) {
            hp->Qnow = (float)ref_real_heat;
        }
    }

    /* ==================================================================
     * 3. 本地拨弹检测状态机 (提前于裁判系统预知热量增加)
     * ================================================================== */
    switch (hp->state)
    {
        case HEAT_STOP:
            // 拨弹轮正常转动，开始检测
            if (wheel_rpm > target_rpm * 0.85f) {
                hp->state = HEAT_PRECHECK;
                hp->suspect_time = 0.0f;
            }
            break;

        case HEAT_PRECHECK:
            // 检测到拨弹阻力电流，进入嫌疑态
            if (wheel_current >= hp->current_th) {
                hp->state = HEAT_SUSPECT;
                hp->suspect_time = 0.0f;
            }
            break;

        case HEAT_SUSPECT:
            // 电流回落，确认子弹挤出
            if (wheel_current < hp->current_th) {
                hp->suspect_time += dt;
                // 根据射速微调此确认时间，通常10ms~20ms
                if (hp->suspect_time > 0.015f) {
                    hp->state = HEAT_CONFIRM;
                    hp->suspect_time = 0.0f;
                }
            } else {
                hp->suspect_time = 0.0f; // 仍在挤压中
            }
            break;

        case HEAT_CONFIRM:
            // 确认打出一发，立刻加上热量！
            hp->Qnow += hp->fire_heat;
            hp->idle_time = 0.0f;  // 重置发弹空闲时间，重新积累信任度
            
            // 循环检测下一发
            hp->state = HEAT_PRECHECK;
            hp->suspect_time = 0.0f;
            break;

        default:
            hp->state = HEAT_STOP;
            hp->suspect_time = 0.0f;
            break;
    }

    /* ==================================================================
     * 4. 本地热量衰减计算
     * ================================================================== */
    hp->Qnow -= hp->Qcd * dt;
    if (hp->Qnow < 0.0f) {
        hp->Qnow = 0.0f;
    }
}

float HeatPredict_ComputeSafeFireRate(const HeatPredict_t* hp, float desire_rate, float max_rate)
{
    const float d = hp->fire_heat;  
    const float Q = hp->Qnow;       // 融合了裁判系统与本地预测的最新最优热量
    const float Qmax = hp->Qmax;    
    const float a = hp->Qcd;        
    // 1. 动态余量：根据弹丸类型设置缓冲区
    // 步兵一发10点，预留1.5发=15点；英雄一发100点，预留105点
    float safety_margin = (hp->robot_type == ROBOT_HERO) ? 105.0f : 15.0f;

    // 2. 软限速：如果进入危险区，直接降速到刚好抵消冷却的射速，或者停射
    if (Q >= Qmax - safety_margin) {
        if (Q >= Qmax - (d * 0.5f)) return 0.0f; // 极度危险，彻底停射
        return (a / d) * 0.8f; // 仅维持冷却平衡的 80%，极慢射击
    }

    // 3. 悲观预测：去掉未来的预测冷却 (a * horizon)，只看当前空间
    // 这样能有效对冲裁判系统 100ms 离散结算的风险
    float r_safe = (Qmax - Q - 5.0f) / (d * 0.1f); 
    
    float r = desire_rate;
    if (r > r_safe) r = r_safe;
    if (r > max_rate) r = max_rate;
    if (r < 0.0f) r = 0.0f;

    return r;
}
