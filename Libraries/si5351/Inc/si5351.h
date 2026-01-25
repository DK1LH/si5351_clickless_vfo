#ifndef SI5351_H
#define SI5351_H

#include "si5351_bus_adaptor.h"
#include "si5351_defines.h"
#include <stdint.h>

si5351_status_t si5351_Setup(si5351_t *si5351, const si5351_bus_t busAdaptor, const uint8_t i2cAddress, const uint32_t xtalFrequency, const int32_t xtalCorrection);
si5351_status_t si5351_Init(si5351_t* si5351);
si5351_status_t si5351_SetOutputCurrent(si5351_t* si5351, const si5351_clk_num_t clk, const si5351_clk_output_current_t outputCurrent);

si5351_status_t si5351_SetFrequency(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clk, const uint32_t frequency);
si5351_status_t si5351_SetFrequencyIQ(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clkA, const si5351_clk_num_t clkB, const uint32_t frequency);
si5351_status_t si5351_SetFrequencyInv(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clkA, const si5351_clk_num_t clkB, const uint32_t frequency);
#endif