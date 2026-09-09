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

        拟合完直流电压测量后，便拟合交流电压测量。在开环逆变下可以采用有效值拟合。
        将获得的有效值显示在屏幕上。

        需要建立RMS结构体（工程初始化时已存在）
        在Setup()中初始化
        在中断中计算有效值

        UAC+接了B0,UAC-接了B1
    */
    //static float32_t filter,last_filter;
    float32_t temp = (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    Udc = 0.016934788f*temp - 0.125286457f;
    float32_t inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    float32_t inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    Uin = (inst_L - inst_H);
    Uan_rms = RMS_Calc(&Uan_RMS, Uin);//也需要在global.h中声明这个变量
    inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);
    Iin = (inst_L - inst_H);
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
