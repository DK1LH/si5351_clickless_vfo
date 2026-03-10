#include "si5351_bus_adaptor.h"

/*
 * @brief Wrapper function for using HAL_I2C_Mem_Read for reading registers via i2c
 */
static si5351_status_t STM32I2cHalReadWrapper(void *vctx, uint8_t i2c_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    si5351_stm32_i2c_ctx_t *ctx = (si5351_stm32_i2c_ctx_t*)vctx;
    if (!ctx || !ctx->hi2c)
    {
        return SI5351_EINVAL;
    }

    // Checking if device is ready if trying is configured
    if (ctx->is_ready_tries > 0)
    {
        if (HAL_I2C_IsDeviceReady(ctx->hi2c, (uint16_t)(i2c_addr << 1), ctx->is_ready_tries, ctx->is_ready_timeout_ms) != HAL_OK)
        {
            return SI5351_EBUS;
        }
    }

    // Reading data and returning status
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(ctx->hi2c, (uint16_t)(i2c_addr << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)len, ctx->is_ready_timeout_ms);
    return (r == HAL_OK) ? SI5351_OK : SI5351_EIO;
}

/*
 * @brief Wrapper function for using HAL_I2C_Mem_Write for writing registers via i2c
 */
static si5351_status_t STM32I2cHalWriteWrapper(void *vctx, uint8_t i2c_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    si5351_stm32_i2c_ctx_t *ctx = (si5351_stm32_i2c_ctx_t*)vctx;
    if (!ctx || !ctx->hi2c)
    {
        return SI5351_EINVAL;
    }

    // Checking if device is ready if trying is configured
    if (ctx->is_ready_tries > 0)
    {
        if (HAL_I2C_IsDeviceReady(ctx->hi2c, (uint16_t)(i2c_addr << 1), ctx->is_ready_tries, ctx->is_ready_timeout_ms) != HAL_OK)
        {
            return SI5351_EBUS;
        }
    }

    // Writing data and returing status
    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(ctx->hi2c, (uint16_t)(i2c_addr << 1), reg,
                                            I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, (uint16_t)len, ctx->is_ready_timeout_ms);
    return (r == HAL_OK) ? SI5351_OK : SI5351_EIO;
}


si5351_bus_t si5351_BusFromCtxSTM32(si5351_stm32_i2c_ctx_t *ctx)
{
    si5351_bus_t b =
    {
        .ctx = ctx,
        .read = STM32I2cHalReadWrapper,
        .write = STM32I2cHalWriteWrapper,
    };
    return b;
}
