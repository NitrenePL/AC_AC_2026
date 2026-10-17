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
float32_t theta_ref; // 参考相位
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

RMS_Obj Uab_RMS,Ubc_RMS;
float32_t Uab_rms,Ubc_rms;
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
    /*
        在前面建立Uab_rms,Ubc_rms的结构体，并在Setup()中初始化
        并拟合两个电压测量，这个题目三相只需要简单的有效值闭环即可。
        由于是有效值拟合，因此测量极性无所谓。
        Uab连A2,A3
        Ubc连A4.A5
        将有效值显示到屏幕上。用excel表格拟合
        由于是60Hz或者30Hz，有效值计算时便需要注意
        RMS.h中MS_WINDOW_SIZE改为2000，便可以测量6个周期的60Hz，3个周期30Hz
    */
    inst_H =  (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);
    inst_L =  (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3);
    float32_t Uab_inst = (inst_H - inst_L);
    Uab_rms = RMS_Calc(&Uab_RMS, Uab_inst);
    inst_H =  (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4);
    inst_L =  (float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5);
    float32_t Ubc_inst = (inst_H - inst_L);
    Ubc_rms = RMS_Calc(&Ubc_RMS, Ubc_inst);
    PLL_Calc(Uin);
    Ualpha = Uin;
    Ialpha = Iin;
    U_data[i] = Ualpha;
    I_data[i] = Ialpha;
    //获取beta轴
    if (i >= 100)
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
    if (MODE == 1)//锁相成功后开波
    {
        /*
        */
        theta_ref += CONST_2PI_32 * 0.003f;//60Hz
        if (theta_ref >= 2.0f * CONST_2PI_32)
        {
            theta_ref -= 2.0f * CONST_2PI_32;
        }
        float32_t Iin_Set = DCL_runPI_C3(&PI_Udc, 60.f, Udc);
        float32_t Usd = Ud + Iq*100*CONST_2PI_32*0.002f - DCL_runPI_C3(&PI_Id, Iin_Set, Id);
        float32_t Usq = Uq - Id*100*CONST_2PI_32*0.002f - DCL_runPI_C3(&PI_Iq, 0.f, Iq);
        //这里补充说明一下，我们使用的是C3，没有积分限幅，C5才有。可以查阅TI官方的DCL文档
        float32_t Usalpha,Usbeta;
        dq_to_AlphaBeta(&Usalpha, &Usbeta, theta, Usd, Usq);
        SPWM(Usalpha / 60.f);
        float32_t Ua_pu = M * __cos(theta_ref);
        float32_t Ub_pu = M * __cos(theta_ref - CONST_2PI_32 / 3.f);
        float32_t Uc_pu = M * __cos(theta_ref + CONST_2PI_32 / 3.f);
        CB_SVPWM_3Ph_DEF(Ua_pu, Ub_pu, Uc_pu, 0);
        EPWM_Start();
        EPWM_Start_DEF();
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
    DCL_resetPI(&PI_Udc);
    // 锁相环初始化
    PLL_Init();

    Keyboard_Init();       // 初始化键盘GPIO
    KEYBOARD_TIMER_Init(); // 初始化键盘CPUTIMER1中断

    Interrupt_enable(INT_ADCA1); // 使能主控中断

    OLED_Display_Init();

    RMS_Init(&Uab_RMS);
    RMS_Init(&Ubc_RMS);
}

void MyProtect(void)
{
}
