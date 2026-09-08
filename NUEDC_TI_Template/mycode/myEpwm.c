#include "myEpwm.h"

// period 为 PWM 周期计数器值 (TBPRD)
// 可以在 main 中通过 EPWM_getTimeBasePeriod(EPWM_A_BASE) 获取

void EPWM_Start(void)
{
    EPWM_clearTripZoneFlag(EPWMA_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWMB_BASE, EPWM_TZ_FLAG_OST);
    //EPWM_clearTripZoneFlag(EPWMC_BASE, EPWM_TZ_FLAG_OST);
    Open = 1;
}

void EPWM_Start_DEF(void)
{
    EPWM_clearTripZoneFlag(EPWMD_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWME_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWMF_BASE, EPWM_TZ_FLAG_OST);
    Open = 2;
}

void EPWM_Stop_DEF(void)
{
    EPWM_forceTripZoneEvent(EPWMD_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWME_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWMF_BASE, EPWM_TZ_FORCE_EVENT_OST);
    Open = 1;
}

void EPWM_Stop(void)
{
    EPWM_forceTripZoneEvent(EPWMA_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWMB_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWMC_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWMD_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWME_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWMF_BASE, EPWM_TZ_FORCE_EVENT_OST);
    Open = 0;
}

void SPWM(float D)
{
    if (D >= 0.99f)
        D = 0.99f;
    else if (D <= -0.99f)
        D = -0.99f;

    EPWM_setCounterCompareValue(EPWMA_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)((0.5f + 0.5f * D) * PWM_PRD));
    EPWM_setCounterCompareValue(EPWMB_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)((0.5f - 0.5f * D) * PWM_PRD));
}

//! \brief          SVPWM 调制，采用最大最小值法注入零序分量 运行约385 clks = 3.85us
//!
//! \details        本函数实现 Carrier-based SVPWM。输入为三相归一化正弦参考量，
//!                 通过三相参考量的最大值和最小值生成零序分量，然后将零序分量
//!                 注入到三相参考量中，最后转换为三相 ePWM 占空比。
//!
//!                 零序分量计算公式为：
//!
//!                     Uzero = -0.5f * (Umax + Umin)
//!
//!                 注入零序后的三相参考量为：
//!
//!                     Ua' = Ua_pu + Uzero
//!                     Ub' = Ub_pu + Uzero
//!                     Uc' = Uc_pu + Uzero
//!
//!                 占空比计算公式为：
//!
//!                     Da = 0.5f + 0.5f * Ua'
//!                     Db = 0.5f + 0.5f * Ub'
//!                     Dc = 0.5f + 0.5f * Uc'
//!
//!                 在线性调制区内，该方法与传统七段式 SVPWM 等效，但不需要
//!                 扇区判断和矢量作用时间计算。
//!
//! \param[in]      Ua_pu  A 相归一化正弦参考量，范围 [-1.1547, 1.1547]
//! \param[in]      Ub_pu  B 相归一化正弦参考量，范围 [-1.1547, 1.1547]
//! \param[in]      Uc_pu  C 相归一化正弦参考量，范围 [-1.1547, 1.1547]
//!
//! \return         None
//!
RAMFUNC void CB_SVPWM_3Ph(float Ua_pu, float Ub_pu, float Uc_pu)
{
    float Umax, Umin, Uzero;
    float Ua_svpwm, Ub_svpwm, Uc_svpwm;
    float Da, Db, Dc;

    // 输入限幅，防止外部控制器输出异常
    DCL_runClamp_C1(&Ua_pu, 1.1547f, -1.1547f);
    DCL_runClamp_C1(&Ub_pu, 1.1547f, -1.1547f);
    DCL_runClamp_C1(&Uc_pu, 1.1547f, -1.1547f);


    Umax = __fmax(Ua_pu, __fmax(Ub_pu, Uc_pu));
    Umin = __fmin(Ua_pu, __fmin(Ub_pu, Uc_pu));

    // 零序分量注入
    Uzero = -0.5f * (Umax + Umin);

    Ua_svpwm = Ua_pu + Uzero;
    Ub_svpwm = Ub_pu + Uzero;
    Uc_svpwm = Uc_pu + Uzero;

    // 归一化 [-1, 1] -> [0, 1]
    Da = 0.5f + 0.5f * Ua_svpwm;
    Db = 0.5f + 0.5f * Ub_svpwm;
    Dc = 0.5f + 0.5f * Uc_svpwm;

    // 这里修改EPWM
    EPWM_setCounterCompareValue(EPWMA_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Da * PWM_PRD));
    EPWM_setCounterCompareValue(EPWMB_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Db * PWM_PRD));
    EPWM_setCounterCompareValue(EPWMC_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Dc * PWM_PRD));
}

RAMFUNC void CB_SVPWM_3Ph_DEF(float Ua_pu, float Ub_pu, float Uc_pu, float ZERO)
{
    float Umax, Umin, Uzero;
    float Ua_svpwm, Ub_svpwm, Uc_svpwm;
    float Da, Db, Dc;

    // 输入限幅，防止外部控制器输出异常
    DCL_runClamp_C1(&Ua_pu, 1.1547f, -1.1547f);
    DCL_runClamp_C1(&Ub_pu, 1.1547f, -1.1547f);
    DCL_runClamp_C1(&Uc_pu, 1.1547f, -1.1547f);
    DCL_runClamp_C1(&ZERO, 0.05f, -0.05f);

    Umax = __fmax(Ua_pu, __fmax(Ub_pu, Uc_pu));
    Umin = __fmin(Ua_pu, __fmin(Ub_pu, Uc_pu));

    // 零序分量注入
    Uzero = -0.5f * (Umax + Umin);

    Ua_svpwm = Ua_pu + Uzero;
    Ub_svpwm = Ub_pu + Uzero;
    Uc_svpwm = Uc_pu + Uzero;

    // 归一化 [-1, 1] -> [0, 1]
    Da = 0.5f + ZERO + 0.5f * Ua_svpwm;
    Db = 0.5f + ZERO + 0.5f * Ub_svpwm;
    Dc = 0.5f + ZERO + 0.5f * Uc_svpwm;

    // 这里修改EPWM
    EPWM_setCounterCompareValue(EPWMD_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Da * PWM_PRD));
    EPWM_setCounterCompareValue(EPWME_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Db * PWM_PRD));
    EPWM_setCounterCompareValue(EPWMF_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Dc * PWM_PRD));
}
