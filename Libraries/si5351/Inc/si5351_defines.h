#ifndef SI5351_DEFINES_H
#define SI5351_DEFINES_H

#include <stdint.h>
#include <stdbool.h>

#include "si5351_bus_adaptor.h"

#define SI5351_PLL_TARGET_FREQ 750000000
#define SI5351_NUM_CLKS 3

#define SI5351_REG_SPREAD_SPECTRUM_PARAMS_1 149
#define SI5351_REG_CLK0_CONTROL 16
#define SI5351_REG_CLK0_PHASE_OFFSET 165
#define SI5351_REG_PLL_RESET 177
#define SI5351_REG_MULTISYNTH_NA_PARAMS_BASE 26
#define SI5351_REG_MULTISYNTH_NA_PARAMS_NB_PARAMS_OFFSET 8
#define SI5351_REG_MULTISYNTH0_PARAMS 42
#define SI5351_REG_MULTISYNTHx_PARAMS_OFFSET 8

typedef enum {
	SI5351_CLK0 = 0,
	SI5351_CLK1 = 1,
	SI5351_CLK2 = 2
} si5351_clk_num_t;

typedef enum {
	SI5351_PLLA = 0,
	SI5351_PLLB = 1
} si5351_pll_num_t;

typedef enum {
	SI5351_OUT_2MA = 0,
	SI5351_OUT_4MA = 1,
	SI5351_OUT_6MA = 2,
	SI5351_OUT_8MA = 3
} si5351_clk_output_current_t;

typedef struct {
    bool powerDown;
    bool integerMode;
    si5351_pll_num_t msPLLSource;
    bool inverted;
    si5351_clk_output_current_t outputCurrent;
} si5351_clk_ctrl_t;

typedef struct {
    uint32_t outDivider;
    uint8_t R;
    uint8_t phaseOffset;
    uint8_t clkCtrlRegVal;
} si5351_clk_t;

typedef struct {
    si5351_bus_t busAdaptor;
    uint16_t i2cAddress;
    uint32_t xtalFrequency;
    uint32_t xtalCorrection;
    si5351_clk_t clks[SI5351_NUM_CLKS];
} si5351_t;

#endif