#include "KeyBoard.h"

void KeyAction(uint16_t key)
{
    switch (key)
    {
        case 1:
            LAST_MODE = MODE;
            MODE = 0;
            break;
        case 2:
            LAST_MODE = MODE;
            MODE = 1;
            break;
        case 3:
            LAST_MODE = MODE;
            MODE = 2;
            break;
        case 4:

            break;
        case 5:
            M += 0.1f;
            break;
        case 6:
            M -= 0.1f;
            break;
        case 7:

            break;
        case 8:

            break;
        case 9:

            break;
        case 10:

            break;
        case 11:

            break;
        case 12:

            break;
        case 13:
            break;
        case 14:
            break;
        case 15:
            OLED_Display_PrevPage();
            break;
        case 16:
            OLED_Display_NextPage();
            break;
    }
}

// 初始化定时器1，设置为50ms中断一次
void KEYBOARD_TIMER_Init(void)
{
    // 1. 配置 CPUTimer1
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0); // 不分频

    // 设置周期为 50,000us (50ms)
    // DEVICE_SYSCLK_FREQ 通常为 100,000,000 (100MHz)
    uint32_t periodCount = (uint32_t)((DEVICE_SYSCLK_FREQ / 1000000UL) * 50000UL);
    CPUTimer_setPeriod(CPUTIMER1_BASE, periodCount);

    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);

    // 2. 使能中断
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);

    // 3. 注册中断向量
    // 注意：INT_TIMER1 属于核心中断 INT13，不经过 PIE
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    Interrupt_enable(INT_TIMER1);

    CPUTimer_startTimer(CPUTIMER1_BASE);
}

void Keyboard_Init(void)
{
    // 配置输入：IRQ 和 DA
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(KEYBOARD_IRQ, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(KEYBOARD_IRQ, GPIO_PIN_TYPE_PULLUP); // 根据硬件情况可改为 STD 或 PULLUP

    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(KEYBOARD_DA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(KEYBOARD_DA, GPIO_PIN_TYPE_PULLUP);

    // 配置输出：CK 和 LD
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(KEYBOARD_CK, GPIO_DIR_MODE_OUT);
    GPIO_writePin(KEYBOARD_CK, 0);

    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_setDirectionMode(KEYBOARD_LD, GPIO_DIR_MODE_OUT);
    GPIO_writePin(KEYBOARD_LD, 0);
}

uint16_t Keyboard_ReadData(void)
{
    uint8_t i;
    uint16_t data = 0;

    GPIO_writePin(KEYBOARD_LD, 1);
    DEVICE_DELAY_US(1); // 1ms 延时

    GPIO_writePin(KEYBOARD_LD, 0);

    for (i = 0; i < 16; i++)
    {
        DEVICE_DELAY_US(1);

        if (GPIO_readPin(KEYBOARD_DA))
        {
            data = 16 - i;
        }

        GPIO_writePin(KEYBOARD_CK, 1);
        DEVICE_DELAY_US(1);

        GPIO_writePin(KEYBOARD_CK, 0);
    }
    return data;
}

void KeyBoard_Scan(void)
{
    uint16_t keyNum = 0;
    uint32_t IRQState = 0;
    static uint32_t IRQStateBkp = 0;

    IRQState = GPIO_readPin(KEYBOARD_IRQ);

    // 上升沿触发
    if (IRQStateBkp == 0 && IRQState == 1)
    {
        keyNum = Keyboard_ReadData();
        KeyAction(keyNum);
    }
    IRQStateBkp = IRQState;
}

// Timer 1 中断服务程序
__interrupt void cpuTimer1ISR(void)
{
    KeyBoard_Scan();
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

    // Timer 1 和 Timer 2 是直接连接到 CPU 的核心中断 (INT13/INT14)
    // 不属于 PIE 组，因此不需要执行 Interrupt_clearACKGroup
}
