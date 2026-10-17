#ifndef RMS_H_
#define RMS_H_

#include "global.h"
#include "math.h"
#include <stdint.h>

#define RMS_WINDOW_SIZE 2000U

typedef struct
{
    float buffer[RMS_WINDOW_SIZE];
    float sum;
    float out;
    uint16_t index;
    uint16_t flag;
} RMS_Obj;

/**
 * @brief 初始化滑动窗口RMS对象。
 * @param obj RMS对象指针。
 */
static inline void RMS_Init(RMS_Obj *obj)
{
    uint16_t i;

    for (i = 0U; i < RMS_WINDOW_SIZE; i++)
    {
        obj->buffer[i] = 0.0f;
    }

    obj->sum = 0.0f;
    obj->out = 0.0f;
    obj->index = 0U;
    obj->flag = 0U;
}

/**
 * @brief 滑动窗口RMS计算。
 * @param obj RMS对象指针。
 * @param input 当前输入采样值。
 * @return 窗口填满后的RMS输出，填满前返回0。
 */
static inline RAMFUNC float RMS_Calc(RMS_Obj *obj, float input)
{
    float newSq;
    float oldSq;

    newSq = input * input;

    if (obj->flag == 1U)
    {
        oldSq = obj->buffer[obj->index] * obj->buffer[obj->index];
        obj->sum = obj->sum + newSq - oldSq;
    }
    else
    {
        obj->sum += newSq;
    }

    obj->buffer[obj->index] = input;

    if (obj->flag == 1U)
    {
        obj->out = sqrtf(obj->sum / (float)RMS_WINDOW_SIZE);
    }
    else
    {
        obj->out = 0.0f;
    }

    obj->index++;
    if (obj->index >= RMS_WINDOW_SIZE)
    {
        obj->index = 0U;
        obj->flag = 1U;
    }

    return obj->out;
}

#endif /* RMS_H_ */
