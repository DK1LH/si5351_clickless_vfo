#ifndef SI5351_BUS_ADAPTOR_H
#define SI5351_BUS_ADAPTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum {
	SI5351_OK = 0,
	SI5351_EBUS = 1,
	SI5351_EINVAL = 2,
	SI5351_EIO = 3
} si5351_status_t;

typedef struct {
    void *ctx;
    si5351_status_t (*read)(void *ctx, uint8_t i2c_addr, uint8_t reg, uint8_t *buf, size_t len);
    si5351_status_t (*write)(void *ctx, uint8_t i2c_addr, uint8_t reg, const uint8_t *buf, size_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
} si5351_bus_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint32_t is_ready_timeout_ms;
    uint8_t  is_ready_tries;
} si5351_stm32_i2c_ctx_t;

// Function to create a si5351_bus_t from si5351_stm32_i2c_ctx_t
si5351_bus_t si5351_bus_from_stm32_hal(si5351_stm32_i2c_ctx_t *ctx);

#endif