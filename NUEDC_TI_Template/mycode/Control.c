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
    float32_t temp = (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    Udc = 0.016934788f*temp - 0.125286457f;
    float32_t inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    float32_t inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    Uin = 0.016232948f*(inst_H - inst_L) + 0.004130499f;
    inst_H = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    inst_L = (float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);
    Iin = 0.001856507f*(inst_L- inst_H) + 0.000015907f;
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
    ab_to_dq(Ualpha, Ubeta, theta, &Ud, &Uq);
    ab_to_dq(Ialpha, Ibeta, theta, &Id, &Iq);
    //注意是theta
    // static float32_t filter,last_filter;
    // filter = 0.01f*Id + 0.99f*last_filter;
    // last_filter = filter;
    // Id = filter;
    //进行滤波方便拟合测量，闭环时应去掉。
    if (MODE == 1 && PLL_Locked == 1)//锁相成功后开波
    {
        /*
        单相整流侧控制采用传统的dq解耦的双环控制，为了后级三相逆变的需要，我们外环使用直流电压环，内环交流电流环。
        在调外环之前，我们先调单电流内环实现整流，此时建议直流侧加上一个负载，并联一个60V的直流电源。
        上电顺序为先开启直流电源，随后开启交流输入，再开始整流
        那么如果设定但电流环的d轴目标值为1.414，q轴为0，则输入功率为36W(题目要求为36V输入)
        需要保证在整流开始前直流电源输送给负载的功率大于输入功率（例如上面说的36W，如果目标值是2.828，则需要保证负载上的功率大于72W）
        只有这样，在开始整流后，功率才不会向直流电源倒灌。

        上面变换的相角都要是锁相得到的角度
        在syscfg中建立两个PI控制器
        Setup中初始化控制器

        可以使用示波器观看电流测量的信号判断PI参数怎么样。
        会附Kp等于1和4的电流瞬时变化图
        */
        theta_ref += CONST_2PI_32 * 0.0025f;
        if (theta_ref >= 2.0f * CONST_2PI_32)
        {
            theta_ref -= 2.0f * CONST_2PI_32;
        }
        float32_t Usd = Ud + Iq*100*CONST_2PI_32*0.002f - DCL_runPI_C3(&PI_Id, 2.828f, Id);
        float32_t Usq = Uq - Id*100*CONST_2PI_32*0.002f - DCL_runPI_C3(&PI_Iq, 0.f, Iq);
        float32_t Usalpha,Usbeta;
        dq_to_AlphaBeta(&Usalpha, &Usbeta, theta, Usd, Usq);
        SPWM(Usalpha / 60.f);
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
    DCL_resetPI(&PI_Id);
    DCL_resetPI(&PI_Iq);
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
