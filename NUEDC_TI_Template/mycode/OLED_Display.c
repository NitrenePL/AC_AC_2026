#include "OLED_Display.h"

#include "global.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

#define OLED_DISPLAY_LINE_COUNT 8U
#define OLED_DISPLAY_LINE_LEN   20U
// 新增显示页面时同步增加该数量
#define OLED_DISPLAY_PAGE_COUNT 2U

#pragma SET_DATA_SECTION("logVariables")
static volatile uint8_t oledRefreshFlag = 0U;
static uint8_t oledPage = 0U;
static char oledLine[OLED_DISPLAY_LINE_COUNT][OLED_DISPLAY_LINE_LEN];
static char oledLastLine[OLED_DISPLAY_LINE_COUNT][OLED_DISPLAY_LINE_LEN];
#pragma SET_DATA_SECTION()

static void OLED_Display_FormatFixed2(char *buffer, const char *label, float32_t value)
{
    int32_t integral;
    int32_t fractional;
    float32_t absValue = value;

    if (absValue < 0.0f)
    {
        absValue = -absValue;
    }

    integral = (int32_t)absValue;
    fractional = (int32_t)((absValue - (float32_t)integral) * 100.0f + 0.5f);

    if (fractional >= 100)
    {
        integral++;
        fractional -= 100;
    }

    if (value < 0.0f)
    {
        sprintf(buffer, "%s:-%02ld.%02ld    ", label, integral, fractional);
    }
    else
    {
        sprintf(buffer, "%s: %02ld.%02ld    ", label, integral, fractional);
    }
}

static void OLED_Display_ShowLine(uint8_t page, char *buffer)
{
    OLED_ShowString(0U, page, (uint8_t *)buffer, 8U);
}

static void OLED_Display_UpdateLine(uint8_t page, char *buffer)
{
    if (strcmp(buffer, oledLastLine[page]) != 0)
    {
        OLED_Display_ShowLine(page, buffer);
        strcpy(oledLastLine[page], buffer);
    }
}

static void OLED_Display_Invalidate(void)
{
    memset(oledLastLine, 0xFF, sizeof(oledLastLine));
}

static void OLED_Display_UpdateHeaderFooter(void)
{
    sprintf(oledLine[0], "            NitreneP");
    OLED_Display_UpdateLine(0U, oledLine[0]);

    // 页脚显示应用层页面编号
    sprintf(oledLine[7], "            PAGE:%u/%u", (uint16_t)(oledPage + 1U), OLED_DISPLAY_PAGE_COUNT);
    OLED_Display_UpdateLine(7U, oledLine[7]);
}

// 第 1 页：主要运行变量
static void OLED_Display_UpdatePage0(void)
{
    sprintf(oledLine[1], "MODE:%-10u", MODE);
    OLED_Display_UpdateLine(1U, oledLine[1]);

    OLED_Display_FormatFixed2(oledLine[2], "Udc", Udc);
    OLED_Display_UpdateLine(2U, oledLine[2]);

    OLED_Display_FormatFixed2(oledLine[3], "xx", 0);
    OLED_Display_UpdateLine(3U, oledLine[3]);

    OLED_Display_FormatFixed2(oledLine[4], "xx", 0);
    OLED_Display_UpdateLine(4U, oledLine[4]);

    OLED_Display_FormatFixed2(oledLine[5], "xx", 0);
    OLED_Display_UpdateLine(5U, oledLine[5]);

    OLED_Display_FormatFixed2(oledLine[6], "xx", 0);
    OLED_Display_UpdateLine(6U, oledLine[6]);
}

// 第 2 页：控制器与调试变量，后续调试量可优先添加在这里
static void OLED_Display_UpdatePage1(void)
{
    OLED_Display_FormatFixed2(oledLine[1], "xx", 0);
    OLED_Display_UpdateLine(1U, oledLine[1]);

    OLED_Display_FormatFixed2(oledLine[2], "xx", 0);
    OLED_Display_UpdateLine(2U, oledLine[2]);

    OLED_Display_FormatFixed2(oledLine[3], "xx", 0);
    OLED_Display_UpdateLine(3U, oledLine[3]);

    OLED_Display_FormatFixed2(oledLine[4], "xx", 0);
    OLED_Display_UpdateLine(4U, oledLine[4]);

    sprintf(oledLine[5], "Open:%-10u", Open);
    OLED_Display_UpdateLine(5U, oledLine[5]);

    sprintf(oledLine[6], "Last:%-10u", LAST_MODE);
    OLED_Display_UpdateLine(6U, oledLine[6]);
}

static void OLED_Display_Update(void)
{
    OLED_Display_UpdateHeaderFooter();

    // 新增页面函数后，在此处注册页面切换逻辑
    switch (oledPage)
    {
        case 0U:
            OLED_Display_UpdatePage0();
            break;

        default:
            OLED_Display_UpdatePage1();
            break;
    }
}

void OLED_Display_Init(void)
{
    OLED_Display_Invalidate();
    OLED_Init();
}

void OLED_Display_RequestRefresh(void)
{
    oledRefreshFlag = 1U;
}

void OLED_Display_Task(void)
{
    if (oledRefreshFlag)
    {
        oledRefreshFlag = 0U;
        OLED_Display_Update();
    }
}

void OLED_Display_NextPage(void)
{
    oledPage++;
    if (oledPage >= OLED_DISPLAY_PAGE_COUNT)
    {
        oledPage = 0U;
    }
    OLED_Display_Invalidate();
    OLED_Display_RequestRefresh();
}

void OLED_Display_PrevPage(void)
{
    if (oledPage == 0U)
    {
        oledPage = OLED_DISPLAY_PAGE_COUNT - 1U;
    }
    else
    {
        oledPage--;
    }
    OLED_Display_Invalidate();
    OLED_Display_RequestRefresh();
}
