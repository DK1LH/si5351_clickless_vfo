#ifndef SI5351_H
#define SI5351_H

#include "si5351_bus_adaptor.h"
#include "si5351_defines.h"
#include <stdint.h>

/**
 * @brief Setup function for initializing the si5351 struct.
 * 
 * @param si5351 Pointer to the si5351 struct to initialize.
 * @param busAdaptor Bus adaptor struct with read/write function pointers and context for i2c communication.
 * @param i2cAddress I2c address of the si5351 to control.
 * @param xtalFrequency Frequency of the XTAL in Hz.
 * @param xtalCorrection Correction to apply to the XTAL frequency. Can be negative.
 * @return si5351_status_t Status of the operation (SI5351_OK if successful).
 */
si5351_status_t si5351_Setup(si5351_t *si5351, const si5351_bus_t busAdaptor, const uint8_t i2cAddress, const uint32_t xtalFrequency, const int32_t xtalCorrection);

/**
 * @brief Function for initializing the si5351. Must be called after Setup and before any other function.
 * 
 * @param si5351 Pointer to the si5351 struct to initialize.
 * @return si5351_status_t Status of the operation (SI5351_OK if successful).
 */
si5351_status_t si5351_Init(si5351_t* si5351);

/**
 * @brief Function for setting the output current for a specific clock output.
 * 
 * @param si5351 Pointer to the si5351 struct which's output should be modified.
 * @param clk Clock output to modify the output current for.
 * @param outputCurrent Target output current to set for the specified clock output.
 * @return si5351_status_t Status of the operation (SI5351_OK if successful).
 */
si5351_status_t si5351_SetOutputCurrent(si5351_t* si5351, const si5351_clk_num_t clk, const si5351_clk_output_current_t outputCurrent);

/**
 * @brief Function for setting the frequency of a specific clock output using a particular PLL as its source.
 * 
 * @param si5351 Pointer to the si5351 struct which's clock output should be set.
 * @param pll PLL that should be used as the source for the specified clock output.
 * @param clk Clock output to set the frequency for.
 * @param frequency Target frequency to set for the specified clock output in Hz.
 * @return si5351_status_t Status of the operation (SI5351_OK if successful).
 */
si5351_status_t si5351_SetFrequency(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clk, const uint32_t frequency);

/**
 * @brief Function for setting the frequency and phase offset using two specific clock outputs and a particular PLL as its source.
 * 
 * @param si5351 Pointer to the si5351 struct which's clock output should be set.
 * @param pll PLL that should be used as the source for the specified clock output.
 * @param clkA First clock output to set the frequency and phase offset for.
 * @param clkB Second clock output to set the frequency and phase offset for.
 * @param frequency Target frequency to set for the specified clock outputs in Hz.
 * @param phOffset Target phase offset of the second clock output.
 * @return si5351_status_t Status of the operation (SI5351_OK if successful).
 */
si5351_status_t si5351_SetFrequencyPhase(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clkA, const si5351_clk_num_t clkB, const uint32_t frequency, const si5351_ph_offset_t phOffset);
#endif