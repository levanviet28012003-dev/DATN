#include "I2C.h"
#include "RCC.h"
#include "GPIO.h"
#include "AFIO.h"
#include "cmsis_compiler.h"

#define I2C_TIMEOUT    200000UL
#define I2C_CR1_PE     (1UL << 0)
#define I2C_CR1_START  (1UL << 8)
#define I2C_CR1_STOP   (1UL << 9)
#define I2C_CR1_ACK    (1UL << 10)
#define I2C_CR1_POS    (1UL << 11)
#define I2C_CR1_SWRST  (1UL << 15)
#define I2C_SR1_SB     (1UL << 0)
#define I2C_SR1_ADDR   (1UL << 1)
#define I2C_SR1_BTF    (1UL << 2)
#define I2C_SR1_RXNE   (1UL << 6)
#define I2C_SR1_ERROR  ((1UL << 8) | (1UL << 9) | (1UL << 10) | \
                        (1UL << 11) | (1UL << 14))
#define I2C_SR2_MSL    (1UL << 0)
#define I2C_SR2_BUSY   (1UL << 1)

// Chờ cờ trạng thái, thoát khi có lỗi hoặc hết thời gian.
static uint8_t I2C_WaitFlag(uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    do
    {
        uint32_t status = I2C1_SR1;
        if (status & I2C_SR1_ERROR) return 0U;
        if ((status & flag) == flag) return 1U;
    } while (--timeout);
    return 0U;
}

static uint8_t I2C_WaitBus(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while ((I2C1_SR2 & I2C_SR2_BUSY) || (I2C1_CR1 & I2C_CR1_STOP))
    {
        if (--timeout == 0U) return 0U;
    }
    return 1U;
}

// Chờ STOP hoàn tất trước khi sửa CR1 hoặc tạo giao dịch mới.
static uint8_t I2C_WaitStop(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C1_CR1 & I2C_CR1_STOP)
    {
        if (--timeout == 0U) return 0U;
    }
    return 1U;
}

static void I2C_ClearADDR(void)
{
    (void)I2C1_SR1;
    (void)I2C1_SR2;
}

// Chọn thanh ghi thiết bị bằng START, địa chỉ ghi và byte thanh ghi.
static uint8_t I2C_SelectRegister(uint8_t addr, uint8_t reg)
{
    I2C1_CR1 |= I2C_CR1_START;
    if (!I2C_WaitFlag(I2C_SR1_SB)) return 0U;
    I2C1_DR = (uint32_t)addr << 1;
    if (!I2C_WaitFlag(I2C_SR1_ADDR)) return 0U;
    I2C_ClearADDR();
    I2C1_DR = reg;
    return I2C_WaitFlag(I2C_SR1_BTF);
}

static void I2C_Abort(void)
{
    if ((I2C1_SR2 & I2C_SR2_MSL) &&
        !(I2C1_CR1 & (I2C_CR1_START | I2C_CR1_STOP)))
    {
        I2C1_CR1 |= I2C_CR1_STOP;
    }
    (void)I2C_WaitStop();
    I2C_Init();
}

// Chờ mức chân và giữ một khoảng ngắn với HCLK 72 MHz.
static uint8_t I2C_WaitLine(uint16_t pin, uint8_t state)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (GPIO_Read_Pin(GPIOB, pin) != state)
    {
        if (--timeout == 0U) return 0U;
    }
    for (volatile uint32_t delay = 0U; delay < 72U; ++delay) __NOP();
    return 1U;
}

// ES096 mục 2.8.7: tạo cạnh SDA/SCL để giải phóng BUSY do bộ lọc analog.
static void I2C_UnlockAnalogFilter(void)
{
    uint8_t scl_high, sda_high;
    I2C1_CR1 = 0U;
    GPIO_Write_Pin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 1U);
    GPIO_Config(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_OD);
    scl_high = I2C_WaitLine(GPIO_PIN_6, 1U);
    sda_high = I2C_WaitLine(GPIO_PIN_7, 1U);
    if (!scl_high || !sda_high) goto finish;

    GPIO_Write_Pin(GPIOB, GPIO_PIN_7, 0U);
    if (!I2C_WaitLine(GPIO_PIN_7, 0U)) goto finish;
    GPIO_Write_Pin(GPIOB, GPIO_PIN_6, 0U);
    if (!I2C_WaitLine(GPIO_PIN_6, 0U)) goto finish;
    GPIO_Write_Pin(GPIOB, GPIO_PIN_6, 1U);
    if (!I2C_WaitLine(GPIO_PIN_6, 1U)) goto finish;
    GPIO_Write_Pin(GPIOB, GPIO_PIN_7, 1U);
    if (!I2C_WaitLine(GPIO_PIN_7, 1U)) goto finish;

finish:
    // Luôn nhả chân và khôi phục I2C, kể cả khi đường dây bị giữ thấp.
    GPIO_Write_Pin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 1U);
    GPIO_Config(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_OD);
    I2C1_CR1 = I2C_CR1_SWRST;
    I2C1_CR1 = 0U;
}

static void I2C_ConfigureTiming(void)
{
    I2C1_OAR1 = 1UL << 14;
    I2C1_CR2 = 36U;
    I2C1_CCR = 180U;             // 36 MHz / (2 * 180) = 100 kHz.
    I2C1_TRISE = 37U;
    I2C1_CR1 = I2C_CR1_ACK | I2C_CR1_PE;
}

// I2C1: PB6/SCL, PB7/SDA, PCLK1 36 MHz, tốc độ 100 kHz.
void I2C_Init(void)
{
    RCC_Enable_PortB();
    RCC_Enable_AFIO();
    RCC_Enable_I2C1();

    // Chọn PB6/PB7 và giữ đầy đủ JTAG/SWD.
    AFIO->MAPR = AFIO->MAPR & ~((1UL << 1) | (7UL << 24));
    I2C1_CR1 = 0U;
    GPIO_Write_Pin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 1U);
    GPIO_Config(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_OD);
    RCC->APB1RSTR.BITS.I2C1RST = 1U;
    RCC->APB1RSTR.BITS.I2C1RST = 0U;
    I2C1_CR1 = 0U;
    I2C_ConfigureTiming();

    if ((I2C1_SR2 & (I2C_SR2_BUSY | I2C_SR2_MSL)) == I2C_SR2_BUSY)
    {
        I2C_UnlockAnalogFilter();
        I2C_ConfigureTiming();
    }
}

uint8_t I2C_WriteReg(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (addr > 0x7FU || (len > 0U && data == 0)) return 0U;
    if (!I2C_WaitBus() || !I2C_SelectRegister(addr, reg)) goto error;

    for (uint8_t i = 0U; i < len; ++i)
    {
        I2C1_DR = data[i];
        if (!I2C_WaitFlag(I2C_SR1_BTF)) goto error;
    }
    I2C1_CR1 |= I2C_CR1_STOP;
    if (!I2C_WaitStop()) goto error;
    return 1U;

error:
    I2C_Abort();
    return 0U;
}

uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i = 0U;
    uint32_t primask;
    if (addr > 0x7FU || data == 0 || len == 0U) return 0U;
    if (!I2C_WaitBus()) goto error;

    I2C1_CR1 = (I2C1_CR1 & ~I2C_CR1_POS) | I2C_CR1_ACK |
                (len == 2U ? I2C_CR1_POS : 0U);
    if (!I2C_SelectRegister(addr, reg)) goto error;

    // Repeated START để nhận; giữ ADDR cho trình tự ACK của từng nhánh.
    I2C1_CR1 |= I2C_CR1_START;
    if (!I2C_WaitFlag(I2C_SR1_SB)) goto error;
    I2C1_DR = ((uint32_t)addr << 1) | 1U;
    if (!I2C_WaitFlag(I2C_SR1_ADDR)) goto error;

    if (len == 1U)
    {
        I2C1_CR1 &= ~I2C_CR1_ACK;
        primask = __get_PRIMASK();
        __disable_irq();
        I2C_ClearADDR();
        I2C1_CR1 |= I2C_CR1_STOP;
        __set_PRIMASK(primask);
        if (!I2C_WaitFlag(I2C_SR1_RXNE)) goto error;
        data[0] = (uint8_t)I2C1_DR;
    }
    else if (len == 2U)
    {
        // AN2824: POS trước ADDR; xóa ACK sau khi xóa ADDR.
        primask = __get_PRIMASK();
        __disable_irq();
        I2C_ClearADDR();
        I2C1_CR1 &= ~I2C_CR1_ACK;
        __set_PRIMASK(primask);
        if (!I2C_WaitFlag(I2C_SR1_BTF)) goto error;
        primask = __get_PRIMASK();
        __disable_irq();
        I2C1_CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)I2C1_DR;
        __set_PRIMASK(primask);
        data[1] = (uint8_t)I2C1_DR;
    }
    else
    {
        I2C_ClearADDR();
        while ((uint8_t)(len - i) > 3U)
        {
            if (!I2C_WaitFlag(I2C_SR1_BTF)) goto error;
            data[i++] = (uint8_t)I2C1_DR;
        }
        if (!I2C_WaitFlag(I2C_SR1_BTF)) goto error;
        I2C1_CR1 &= ~I2C_CR1_ACK;

        // ES096: bảo vệ trình tự từ đọc N-2 đến STOP và đọc N-1.
        primask = __get_PRIMASK();
        __disable_irq();
        data[i++] = (uint8_t)I2C1_DR;
        if (!I2C_WaitFlag(I2C_SR1_BTF))
        {
            __set_PRIMASK(primask);
            goto error;
        }
        I2C1_CR1 |= I2C_CR1_STOP;
        data[i++] = (uint8_t)I2C1_DR;
        __set_PRIMASK(primask);
        if (!I2C_WaitFlag(I2C_SR1_RXNE)) goto error;
        data[i++] = (uint8_t)I2C1_DR;
    }

    if (!I2C_WaitStop()) goto error;
    I2C1_CR1 = (I2C1_CR1 & ~I2C_CR1_POS) | I2C_CR1_ACK;
    return 1U;

error:
    I2C_Abort();
    return 0U;
}
