#include "systick.h"
#include "stm32f1xx.h"

static volatile uint32_t milliseconds;
static uint8_t systick_ready;

uint8_t SysTick_Init(void)
{
    SysTick->CTRL = 0U;
    systick_ready = 0U;
    SystemCoreClockUpdate();
    milliseconds = 0U;

    // CMSIS cấu hình ngắt SysTick theo HCLK thực te
    systick_ready = (uint8_t)(SysTick_Config(SystemCoreClock / 1000U) == 0U);
    return systick_ready;
}

uint8_t SysTick_IsReady(void)
{
    uint32_t required = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
    return (uint8_t)(systick_ready && (SysTick->CTRL & required) == required);
}

uint32_t SysTick_Millis(void)
{
    return milliseconds;
}

void SysTick_Handler(void)
{
    ++milliseconds;
}
