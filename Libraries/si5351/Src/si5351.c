#include "si5351.h"
#include "si5351_bus_adaptor.h"
#include "si5351_defines.h"


/*
 * @brief Private function for reading registers
 */
si5351_status_t si5351_ReadReg(si5351_t *h, uint8_t reg, uint8_t *val) {
    if (!h || !val) return SI5351_EINVAL;
    return h->busAdaptor.read(h->busAdaptor.ctx, h->i2cAddress, reg, val, 1);
}

/*
 * @brief Private function for writing one register
 */
si5351_status_t si5351_WriteReg(si5351_t *h, uint8_t reg, uint8_t val) {
    if (!h) return SI5351_EINVAL;
    return h->busAdaptor.write(h->busAdaptor.ctx, h->i2cAddress, reg, &val, 1);
}

/*
 * @brief Private function for writing multiple registers
 */
si5351_status_t si5351_WriteRegs(si5351_t *h, uint8_t regStart, uint8_t* vals, uint8_t length)
{
	if (!h) return SI5351_EINVAL;
	return h->busAdaptor.write(h->busAdaptor.ctx, h->i2cAddress, regStart, vals, length);
}

/*
 * @brief Private function for resetting PLLx
 */
si5351_status_t si5351_ResetPll(si5351_t* si5351, si5351_pll_num_t pll)
{
    if(!si5351) return SI5351_EINVAL;
    
    uint8_t regVal = 0;
    switch (pll) {
        case SI5351_PLLA:
            regVal = 0b00100000;
            break;
        case SI5351_PLLB:
            regVal = 0b10000000;
            break;
        default:
            return SI5351_EINVAL;
            break;
    }
    return si5351_WriteReg(si5351, SI5351_REG_PLL_RESET, regVal);
}

/*
 * @brief Private function for writing the clock control register of CLKx
 */
si5351_status_t si5351_WriteClkCtrlRegister(si5351_t* si5351, si5351_clk_num_t clk)
{
    if (!si5351) return SI5351_EINVAL;

	uint8_t regVal = 0b00001100; //CLKx always uses MSx
	regVal |= si5351->clks[clk].powerDown ? (1 << 7) : 0;
	regVal |= si5351->clks[clk].integerMode ? (1 << 6) : 0;
	regVal |= (si5351->clks[clk].msPLLSource << 5);
	regVal |= si5351->clks[clk].inverted ? (1 << 4) : 0;
	regVal |= si5351->clks[clk].outputCurrent;

    return si5351_WriteReg(si5351, SI5351_REG_CLK0_CONTROL + clk, regVal);
}


si5351_status_t si5351_Setup(si5351_t *si5351, si5351_bus_t busAdaptor, uint8_t i2cAddress, uint32_t xtalFrequency, int32_t xtalCorrection)
{
    if(!si5351) return SI5351_EINVAL;

    si5351->busAdaptor = busAdaptor;
    si5351->i2cAddress = i2cAddress;
    si5351->xtalFrequency = xtalFrequency + xtalCorrection;
    return SI5351_OK;
}

si5351_status_t si5351_Init(si5351_t* si5351)
{
    if(!si5351) return SI5351_EINVAL;

    // Turning off spread spectrum mode (undefined after power-up)
    si5351_status_t commResult;
    uint8_t regVal = 0;
    commResult = si5351_ReadReg(si5351, SI5351_REG_SPREAD_SPECTRUM_PARAMS_1, &regVal);
    if(commResult != SI5351_OK) return commResult;
    regVal &= ~(1 << 7);
    commResult = si5351_WriteReg(si5351, SI5351_REG_SPREAD_SPECTRUM_PARAMS_1, regVal);
    if(commResult != SI5351_OK) return commResult;

    for(uint8_t i = 0; i <= 2; i++)
    {
        si5351->clks[i].powerDown = false;
        si5351->clks[i].integerMode = true;
        si5351->clks[i].msPLLSource = SI5351_PLLA;
        si5351->clks[i].inverted = false;
        si5351->clks[i].outputCurrent = SI5351_OUT_8MA;
        commResult = si5351_WriteClkCtrlRegister(si5351, i);
        if(commResult != SI5351_OK) return commResult;
    }

    return commResult;
}

si5351_status_t si5351_SetOutputCurrent(si5351_t* si5351, si5351_clk_num_t clk, si5351_clk_output_current_t outputCurrent)
{
    if(!si5351) return SI5351_EINVAL;

    si5351->clks[clk].outputCurrent = outputCurrent;
    return si5351_WriteClkCtrlRegister(si5351, clk);
}


si5351_status_t si5351_SetFrequency(si5351_t* si5351, si5351_pll_num_t pll, si5351_clk_num_t clk, uint32_t frequency)
{
    if (!si5351) return SI5351_EINVAL;

    // Finding an even out divider that 
    uint32_t outDivider = SI5351_PLL_MAX_FREQ / frequency;
    if(outDivider % 2) outDivider--;

    // Calculating the actual frequency of PLLx
    uint32_t frequencyPLLx = outDivider * frequency;
    
    // Calculating (a/b/c) for the (fractional) divider of PLLx's FMD
    uint8_t a = frequencyPLLx / si5351->xtalFrequency;
    uint32_t b = (frequencyPLLx % si5351->xtalFrequency) >> 5;
    uint32_t c = si5351->xtalFrequency >> 5;
    uint32_t f = (128 * b) / c;

    // Building the register values for PLLx's MultiSynth N (FMD)
    uint32_t MSNxP1 = 128 * a + f - 512;
    uint32_t MSNxP2 = 128 * b - f * c;
    uint32_t MSNxP3 = c;

    uint8_t PLLxMSNxParams[] =
    {
    (MSNxP3 & 0xFF00) >> 8,                                     // Bits [15:8] of MSNx_P3 in register 26
	MSNxP3 & 0xFF,
    (MSNxP1 & 0x030000L) >> 16,
    (MSNxP1 & 0xFF00) >> 8,                                     // Bits [15:8]  of MSNx_P1 in register 29
	MSNxP1 & 0xFF,                                              // Bits [7:0]  of MSNx_P1 in register 30
	((MSNxP3 & 0x0F0000L) >> 12) | ((MSNxP2 & 0x0F0000) >> 16), // Parts of MSNx_P3 and MSNx_P1
	(MSNxP2 & 0xFF00) >> 8,                                     // Bits [15:8]  of MSNx_P2 in register 32
	MSNxP2 & 0xFF                                               // Bits [7:0]  of MSNx_P2 in register 33
    };

    // Checking whether updating the CLK's MultiSynth (OMD) is necessary (PLL reset needed)
    si5351_status_t commResult = SI5351_OK;
    if(outDivider != si5351->clks[clk].outDivider)
    {
        // Updating the outDivider
        si5351->clks[clk].outDivider = outDivider;

        uint32_t MSxP1 = 128 * outDivider - 512;
        uint8_t CLKxMSxParams[] =
	    {
        0, 								// Bits [15:8] of MS0_P3 (always 0) in register 42
		1,            					// Bits [7:0]  of MS0_P3 (always 1) in register 43
		((MSxP1 & 0x030000L) >> 16),    // Bits [17:16] of MSx_P1 in bits [1:0] and R in [7:4] | [3:2]
		(MSxP1 & 0xFF00) >> 8, 			// Bits [15:8]  of MSx_P1 in register 45
		MSxP1 & 0xFF,           		// Bits [7:0]  of MSx_P1 in register 46
		0,             					// Bits [19:16] of MS0_P2 and MS0_P3 are always 0
		0,                        		// Bits [15:8]  of MS0_P2 are always 0
		0                        		// Bits [7:0]   of MS0_P2 are always 0
	    };

        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH_NA_PARAMS_BASE + (pll * SI5351_REG_MULTISYNTH_NA_PARAMS_NB_PARAMS_OFFSET), PLLxMSNxParams, 8);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH0_PARAMS + (SI5351_REG_MULTISYNTHx_PARAMS_OFFSET * clk), CLKxMSxParams, 8);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_ResetPll(si5351, pll);
    }
    else
    {
        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH_NA_PARAMS_BASE + (pll * SI5351_REG_MULTISYNTH_NA_PARAMS_NB_PARAMS_OFFSET), PLLxMSNxParams, 8);
    }

    return commResult;
}


si5351_status_t si5351_SetFrequencyPhase(si5351_t* si5351, si5351_pll_num_t pll, si5351_clk_num_t clkA, si5351_t clkB, uint32_t frequency, uint8_t phaseOffset)
{
    return SI5351_OK;
}