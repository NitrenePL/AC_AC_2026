#include "Control.h"
#include "DCLF32.h"
#include "KeyBoard.h"
#include "LPF_COEFF.h"
#include "OLED_Display.h"
#include "PLL.h"
#include "QPR.h"
#include "RMS.h"
#include "global.h"
#include "myEpwm.h"
#include "park_trans.h"

// OLED屏幕刷新率, 当前15Hz
#define OLED_REFRESH_RATE_HZ 15U
#define SLOW_TASK_RATE_HZ    1000U

#pragma SET_DATA_SECTION("controlVariables")
// 高速RAM变量放在这里, 例如电流环、滤波器等
float32_t theta_ref = 0.f; // 参考相位
float32_t M = 0.2f;
float32_t Uin,Iin,Udc;
float32_t Ud, Uq;
float32_t Id, Iq;
float32_t Ualpha, Ubeta;
float32_t Ialpha, Ibeta;

#pragma SET_DATA_SECTION()

#pragma SET_DATA_SECTION("logVariables")
// 低速RAM变量放在这里, 例如RMS数组、状态数据等

uint16_t PWM_PRD;    // 假设默认周期值
uint8_t Open = 0;        // 默认关闭
uint8_t MODE = 0;        // 默认模式0
uint8_t RECT_ENABLE = 0; // 开启整流
uint8_t LAST_MODE = 0;

float32_t U_data[400];
float32_t I_data[400];

RMS_Obj Uan_RMS;
float32_t Uan_rms;
#pragma SET_DATA_SECTION()

// Timer0 中断服务函数 慢速任务 1kHz
__interrupt void INT_myCPUTIMER0_ISR(void)
{
    static uint16_t ledTaskCnt = 0;
    static uint16_t oledRefreshAcc = 0;

    oledRefreshAcc = (uint16_t)(oledRefreshAcc + OLED_REFRESH_RATE_HZ);
    if (oledRefreshAcc >= SLOW_TASK_RATE_HZ)
    {
        oledRefreshAcc = (uint16_t)(oledRefreshAcc - SLOW_TASK_RATE_HZ);
        OLED_Display_RequestRefresh();
    }

    if (++ledTaskCnt >= 400U)
    {
        ledTaskCnt = 0U;
        LED_TOGGLE1();
    }

    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// ADC_INT 中断服务函数 主控中断 20kHz
RAMFUNC __interrupt void ADC_SamplingISR(void)
{
    static int i;
    /*
        测量的拟合，对于一个单相整流，我们需要交流电压测量提供电网相位、作为控制前馈
        需要交流电流测量做双环控制，直流电压测量控制母线电压为后级三相逆变做准备。
        而后级三相逆变简单点可以只使用有效值控制，那么只需要一到两个交流电压测量。

        拟合完交流电压测量。然后就是交流电流测量。
        单相底板上有两个交流电流测量，只需要用一个，选择INL的那个。
        利用锁相环获取开环逆变的电压相位，然后利用这个相位对采样电流进行dq变换
        将获得的d轴q轴电流显示在屏幕上。d轴对应电流峰值，q轴接近于0.

        关于电流测量的校准，前一步中完善了电压的极性，那么我们以电压测量获得的相位（这个相位是和theta_ref一致的）
        对电流进行变换，对于阻性负载，q轴就应该接近于0

        而后面我们将进行整流，那么规定整流的电流方向为正，那么在逆变时我们便要保证变换出来的d轴电流是负的
        （功率流向所决定的），并且d轴电流的绝对值对应交流电流的峰值

        IOUT_L+接了B2,IOUT_L-接了B3
    */
    
    float32_t temp = (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    Udc = 0.016934788f*temp - 0.125286457f;
    float32_t inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    float32_t inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    Uin = 0.016232948f*(inst_H - inst_L) + 0.004130499f;

    inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);
    Iin = (inst_L - inst_H);
    //逆变核验此时d轴是负的，因此不需要改，保持L-H
    PLL_Calc(Uin);
    Ualpha = Uin;
    Ialpha = Iin;
    U_data[i] = Ualpha;
    I_data[i] = Ialpha;
    //获取beta轴
    if (i > 100)
    {
        Ubeta = U_data[i - 100];
        Ibeta = I_data[i - 100];
    }
    else
    {
        Ubeta = U_data[i + 300];
        Ibeta = I_data[i + 300];
    }
    //变换
    //ab_to_dq(Ualpha, Ubeta, theta_ref, &Ud, &Uq);
    ab_to_dq(Ialpha, Ibeta, theta, &Id, &Iq);
    //注意是theta
    static float32_t filter,last_filter;
    filter = 0.01f*Id + 0.99f*last_filter;
    last_filter = filter;
    Id = filter;
    //进行滤波方便拟合测量，闭环时应去掉。
    if (MODE == 1)
    {
        theta_ref += CONST_2PI_32 * 0.0025f;
        if (theta_ref >= 2.0f * CONST_2PI_32)
        {
            theta_ref -= 2.0f * CONST_2PI_32;
        }
        SPWM(M * __cos(theta_ref));
        EPWM_Start();
    }
    else if (MODE == 2)
    {
    }
    else if (MODE == 0)
    {
        EPWM_Stop();
    }
    i++;
    if (i >= 400)
    {
        i = 0;
    }
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void Loop(void)
{
    OLED_Display_Task(); // 10ms刷新周期 由CPUTIMER0调度
}
void Setup(void)
{
    PWM_PRD = EPWM_getTimeBasePeriod(EPWM_A_BASE);
    EPWM_Stop();

    // 控制器参数计算&初始化

    // 锁相环初始化
    PLL_Init();

    Keyboard_Init();       // 初始化键盘GPIO
    KEYBOARD_TIMER_Init(); // 初始化键盘CPUTIMER1中断

    Interrupt_enable(INT_ADCA1); // 使能主控中断

    OLED_Display_Init();

    RMS_Init(&Uan_RMS);
}

void MyProtect(void)
{
}
