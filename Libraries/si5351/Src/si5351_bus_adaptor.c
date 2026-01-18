#include "si5351_bus_adaptor.h"

static si5351_status_t stm32_hal_read(void *vctx, uint8_t i2c_addr, uint8_t reg, uint8_t *buf, size_t len) {
    si5351_stm32_i2c_ctx_t *ctx = (si5351_stm32_i2c_ctx_t*)vctx;
    if (!ctx || !ctx->hi2c) return SI5351_EINVAL;

    /* Optional readiness polling with configurable tries */
    if (ctx->is_ready_tries) {
        if (HAL_I2C_IsDeviceReady(ctx->hi2c, (uint16_t)(i2c_addr << 1), ctx->is_ready_tries, ctx->is_ready_timeout_ms) != HAL_OK)
            return SI5351_EBUS;
    }
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(ctx->hi2c, (uint16_t)(i2c_addr << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)len, ctx->is_ready_timeout_ms);
    return (r == HAL_OK) ? SI5351_OK : SI5351_EIO;
}

static si5351_status_t stm32_hal_write(void *vctx, uint8_t i2c_addr, uint8_t reg, const uint8_t *buf, size_t len) {
    si5351_stm32_i2c_ctx_t *ctx = (si5351_stm32_i2c_ctx_t*)vctx;
    if (!ctx || !ctx->hi2c) return SI5351_EINVAL;

    if (ctx->is_ready_tries) {
        if (HAL_I2C_IsDeviceReady(ctx->hi2c, (uint16_t)(i2c_addr << 1), ctx->is_ready_tries, ctx->is_ready_timeout_ms) != HAL_OK)
            return SI5351_EBUS;
    }
    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(ctx->hi2c, (uint16_t)(i2c_addr << 1), reg,
                                            I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, (uint16_t)len, ctx->is_ready_timeout_ms);
    return (r == HAL_OK) ? SI5351_OK : SI5351_EIO;
}

static void stm32_hal_delay(void *vctx, uint32_t ms) {
    (void)vctx;
    HAL_Delay(ms);
}

si5351_bus_t si5351_bus_from_stm32_hal(si5351_stm32_i2c_ctx_t *ctx) {
    si5351_bus_t b = {
        .ctx = ctx,
        .read = stm32_hal_read,
        .write = stm32_hal_write,
        .delay_ms = stm32_hal_delay
    };
    return b;
}
