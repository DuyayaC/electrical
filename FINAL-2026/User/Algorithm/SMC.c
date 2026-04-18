#include "SMC.h"

static float SMC_Signal(float s);
static float SMC_Sat(Sliding *smc, float s);

void SMC_Init(Sliding *smc) {
    if (!smc) return;
    memset(smc, 0, sizeof(Sliding));
    smc->flag = EXPONENT;
}

// EXPONENT 参数设定
void SMC_SetParam_Exponent(Sliding *smc, float J, float K, float c, float epsilon, float limit, float u_max, float pos_esp) {
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c = c;
    smc->param.epsilon = epsilon;
    smc->u_max = u_max;
    smc->limit = limit;
    smc->error.pos_error_eps = pos_esp;
    smc->flag = EXPONENT;
}

// EISMC 参数设定
void SMC_SetParam_EISMC(Sliding *smc, float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_esp) {
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c1 = c1;
    smc->param.c2 = c2;
    smc->param.epsilon = epsilon;
    smc->u_max = u_max;
    smc->limit = limit;
    smc->error.pos_error_eps = pos_esp;
    smc->flag = EISMC;
}

void SMC_ErrorUpdate(Sliding *smc, float target, float pos_now, float vol_now) {
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last) / SAMPLE_PERIOD;
    smc->error.tar_differential_second = (smc->error.tar_differential - smc->error.tar_differential_last) / SAMPLE_PERIOD;

    smc->error.p_error = pos_now - target;
    smc->error.v_error = vol_now - smc->error.tar_differential;
    
    // 累计位置误差积分
    smc->error.p_error_integral += (smc->error.p_error * SAMPLE_PERIOD);

    smc->error.tar_last = smc->error.tar_now;
    smc->error.tar_differential_last = smc->error.tar_differential;
}

float SMC_Calculate(Sliding *smc) {
    float u = 0;
    float sat_s = 0;

    // 精度死区判断
    if (fabsf(smc->error.p_error) < smc->error.pos_error_eps) {
        smc->error.p_error = 0;
        smc->u = 0;
        return 0;
    }

    switch (smc->flag) {
        case EXPONENT:
            // 滑模面 s = c*e + edot
            smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
            sat_s = SMC_Sat(smc, smc->s);
            // 指数趋近律控制量
            u = smc->param.J * ((-smc->param.c * smc->error.v_error) 
                               - smc->param.K * smc->s 
                               - smc->param.epsilon * sat_s);
            break;

        case EISMC:
            // 比例积分滑模面 s = c1*e + edot + c2*∫e
            smc->s = smc->param.c1 * smc->error.p_error 
                     + smc->error.v_error 
                     + smc->param.c2 * smc->error.p_error_integral;
            sat_s = SMC_Sat(smc, smc->s);
            // 控制量计算
            u = smc->param.J * ((-smc->param.c1 * smc->error.v_error) 
                               - smc->param.c2 * smc->error.p_error 
                               - smc->param.K * smc->s 
                               - smc->param.epsilon * sat_s);
            break;
    }

    // 输出限幅
    if (u > smc->u_max) u = smc->u_max;
    else if (u < -smc->u_max) u = -smc->u_max;

    smc->u = u;
    return u;
}

void SMC_Clear(Sliding *smc) {
    memset(&smc->error, 0, sizeof(RError));
    smc->u = 0;
    smc->s = 0;
}

void SMC_IntegralClear(Sliding *smc) {
    smc->error.p_error_integral = 0;
}

static float SMC_Signal(float s) {
    return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
}

static float SMC_Sat(Sliding *smc, float s) {
    float y = s / smc->param.epsilon;
    if (fabsf(y) <= smc->limit) return y;
    else return SMC_Signal(y);
}