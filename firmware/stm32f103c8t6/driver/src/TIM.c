#include "TIM.h"
#include "RCC.h"
#include "GPIO.h"
#include "AFIO.h"

/**
 * @brief Khoi tao PWM TIM2 Channel 1.
 *
 * @details
 * - TIM2CLK = 72MHz.
 * - PSC = 71.
 * - ARR = 19999.
 * - PWM Mode 1.
 * - Tan so PWM = 50Hz.
 * - Chan PA0.
 *
 * @return Khong co.
 */
void TIM2_PWM_Init(void)
{
    uint32_t mapr;

    RCC_Enable_PortA();
    RCC_Enable_AFIO();
    RCC_Enable_TIM2();

    GPIO_Config(GPIOA, GPIO_PIN_0, GPIO_MODE_AF_PP);

    // Chọn PA0 và đặt SWJ_CFG = 000 tường minh để giữ đầy đủ JTAG/SWD.
    mapr = AFIO->MAPR;
    mapr &= ~((3UL << 8) | (7UL << 24));
    AFIO->MAPR = mapr;

    // Reset TIM2 để loại bỏ chế độ đếm, ngắt hoặc cấu hình PWM cũ.
    RCC->APB1RSTR.BITS.TIM2RST = 1U;
    RCC->APB1RSTR.BITS.TIM2RST = 0U;
    TIM2_CR1 = 0U;
    TIM2_CR2 = 0U;
    TIM2_SMCR = 0U;
    TIM2_DIER = 0U;
    TIM2_CCER = 0U;
    TIM2_CCMR2 = 0U;

    /* Thiet lap bo chia tan so */
    TIM2_PSC = 71U;

    /* Chu ky PWM 20ms */
    TIM2_ARR = 19999U;

    /* Do rong xung ban dau 1500us */
    TIM2_CCR1 = 1500U;

    /* Channel 1 la output */
    TIM2_CCMR1 = 0U;

    /* Xoa cau hinh Output Compare */
    TIM2_CCMR1 &= ~(7UL << 4);

    /* PWM Mode 1 */
    TIM2_CCMR1 |= (6UL << 4);

    /* Bat CCR1 preload */
    TIM2_CCMR1 |= (1UL << 3);

    /* PWM active high */
    TIM2_CCER &= ~(1UL << 1);

    /* Cho phep TIM2 Channel 1 */
    TIM2_CCER |= (1UL << 0);

    /* Bat ARR preload */
    TIM2_CR1 |= (1UL << 7);

    /* Cap nhat thanh ghi */
    TIM2_EGR = (1UL << 0);

    /* Xoa co update */
    TIM2_SR = 0U;

    /* Reset bo dem */
    TIM2_CNT = 0U;

    /* Bat TIM2 */
    TIM2_CR1 |= (1UL << 0);
}

/**
 * @brief Thay doi do rong xung PWM Channel 1.
 *
 * @param pulse Do rong xung PWM tinh bang us.
 * @return Khong co.
 */
void TIM2_PWM_SetPulse(uint16_t pulse)
{
    if (pulse > 19999U)
    {
        pulse = 19999U;
    }

    TIM2_CCR1 = pulse;
}
