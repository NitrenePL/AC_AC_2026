#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#define ISR_FREQ 20000.f

#define RAMFUNC __attribute__((section(".TI.ramfunc")))

#define LED_TOGGLE1() GPIO_togglePin(LED1)
#define LED_TOGGLE2() GPIO_togglePin(LED2)
#define LED_WRITE1_HIGH() GPIO_writePin(LED1,1)
#define LED_WRITE2_HIGH() GPIO_writePin(LED2,1)
#define LED_WRITE1_LOW() GPIO_writePin(LED1,0)
#define LED_WRITE2_LOW() GPIO_writePin(LED2,0)
#include "device.h"
#include "driverlib.h"
#include <stdint.h>
#include "c2000ware_libraries.h"



#ifdef __cplusplus
extern "C"
{
#endif
    extern uint16_t PWM_PRD;
    extern uint8_t Open; // 开波标志：1-开，0-关
    extern uint8_t MODE; // 模式变量
    extern uint8_t LAST_MODE;
    extern uint8_t   RECT_ENABLE;
    
    extern float32_t theta_ref;
    extern float32_t Ua_pu;
    extern float32_t Ub_pu;
    extern float32_t Uc_pu;
    extern float32_t M;  //调制比
    extern float32_t Ud, Uq;
    extern float32_t Id, Iq;
    extern float32_t Uin,Iin,Udc;

    extern float32_t Uab_rms,Ubc_rms;
    extern uint8_t MODE_60Hz_30Hz;
    
#ifdef __cplusplus
}
#endif

#endif /* GLOBAL_VARS_H */
