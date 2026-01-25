#include "si5351.h"
#include "si5351_bus_adaptor.h"
#include "si5351_defines.h"
#include <stdint.h>


/*
 * @brief Private function for reading registers
 */
static inline si5351_status_t si5351_ReadReg(const si5351_t *h, const uint8_t reg, uint8_t *val) {
    if (!h || !val) return SI5351_EINVAL;
    return h->busAdaptor.read(h->busAdaptor.ctx, h->i2cAddress, reg, val, 1);
}

/*
 * @brief Private function for writing one register
 */
static inline si5351_status_t si5351_WriteReg(const si5351_t *h, const uint8_t reg, const uint8_t val) {
    if (!h) return SI5351_EINVAL;
    return h->busAdaptor.write(h->busAdaptor.ctx, h->i2cAddress, reg, &val, 1);
}

/*
 * @brief Private function for writing multiple registers
 */
static inline si5351_status_t si5351_WriteRegs(const si5351_t *h, const uint8_t regStart, const uint8_t* vals, const uint8_t length)
{
	if (!h) return SI5351_EINVAL;
	return h->busAdaptor.write(h->busAdaptor.ctx, h->i2cAddress, regStart, vals, length);
}

/*
 * @brief Private function for resetting PLLx
 */
static inline si5351_status_t si5351_ResetPll(const si5351_t* si5351, const si5351_pll_num_t pll)
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
 * @brief Private function for creating a ClkCtrl register value from the struct representation
 */
static inline uint8_t si5351_PackClkCtrlRegVal(const si5351_clk_ctrl_t clkCtrl)
{
    uint8_t regVal = 0b00001100; //CLKx always uses MSx
	regVal |= clkCtrl.powerDown ? (1u << 7) : 0u;
	regVal |= clkCtrl.integerMode ? (1u << 6) : 0u;
	regVal |= (uint8_t)((clkCtrl.msPLLSource & 0x01u) << 5);
	regVal |= clkCtrl.inverted ? (1u << 4) : 0u;
	regVal |= (uint8_t)(clkCtrl.outputCurrent & 0x03u);

    return regVal;
}

/*
 * @brief Private function for creating the struct representation from a ClkCtrl register value
 */
static inline si5351_clk_ctrl_t si5351_UnpackClkCtrlRegVal(const uint8_t regVal)
{
    si5351_clk_ctrl_t clkCtrl =
    {
        .powerDown = ((regVal >> 7) & 1) ? true : false,
        .integerMode = ((regVal >> 6) & 1) ? true : false,
        .msPLLSource = (si5351_pll_num_t)((regVal >> 5) & 0x01u),
        .inverted = ((regVal >> 4) & 1) ? true : false,
        .outputCurrent = (si5351_clk_output_current_t)(regVal & 0x03u)
    };

    return clkCtrl;
}

/*
 * @brief Private function for ensuring that a ClkCtrl register has a desired value
 */
static inline si5351_status_t si5351_EnsureClkCtrl(si5351_t* si5351, const si5351_clk_num_t clk, const si5351_clk_ctrl_t clkCtrl)
{
    if(!si5351 || (clk >= SI5351_NUM_CLKS)) return SI5351_EINVAL;

    uint8_t clkCtrlNewRegval = si5351_PackClkCtrlRegVal(clkCtrl);
    if(si5351->clks[clk].clkCtrlRegVal == clkCtrlNewRegval)
    {
        return SI5351_OK;
    }

    si5351->clks[clk].clkCtrlRegVal = clkCtrlNewRegval;
    return si5351_WriteReg(si5351, SI5351_REG_CLK0_CONTROL + clk, clkCtrlNewRegval);
}

/*
 * @brief Private function for generating a PLL's MSN's register values
 */
static inline void si5351_GeneratePLLxMSNxParams(const uint32_t frequencyPLLx, const uint32_t xtalFrequency, uint8_t outPLLxMSNxParams[8])
{
    // Calculating (a/b/c) for the (fractional) divider of PLLx's FMD
    uint8_t a = frequencyPLLx / xtalFrequency;
    uint32_t b = (frequencyPLLx % xtalFrequency) >> 5;
    uint32_t c = xtalFrequency >> 5;
    uint32_t f = (128 * b) / c;

    // Building the register values for PLLx's MultiSynth N (FMD)
    uint32_t MSNxP1 = 128 * a + f - 512;
    uint32_t MSNxP2 = 128 * b - f * c;
    uint32_t MSNxP3 = c;

    outPLLxMSNxParams[0] = (uint8_t)((MSNxP3 & 0xFF00) >> 8);                                       // Bits [15:8] of MSNx_P3 in register 26
	outPLLxMSNxParams[1] = (uint8_t)(MSNxP3 & 0xFF);
    outPLLxMSNxParams[2] = (uint8_t)((MSNxP1 & 0x030000L) >> 16);
    outPLLxMSNxParams[3] = (uint8_t)((MSNxP1 & 0xFF00) >> 8);                                       // Bits [15:8]  of MSNx_P1 in register 29
	outPLLxMSNxParams[4] = (uint8_t)(MSNxP1 & 0xFF);                                                // Bits [7:0]  of MSNx_P1 in register 30
	outPLLxMSNxParams[5] = (uint8_t)(((MSNxP3 & 0x0F0000L) >> 12) | ((MSNxP2 & 0x0F0000) >> 16));   // Parts of MSNx_P3 and MSNx_P1
	outPLLxMSNxParams[6] = (uint8_t)((MSNxP2 & 0xFF00) >> 8);                                       // Bits [15:8]  of MSNx_P2 in register 32
	outPLLxMSNxParams[7] = (uint8_t)(MSNxP2 & 0xFF);                                                // Bits [7:0]  of MSNx_P2 in register 33
}

/*
 * @brief Private function for generating a CLK's MS's register values
 */
static inline void si5351_GenerateCLKxMSxParams(const uint32_t outDivider, uint8_t outCLKxMSxParams[8])
{
    uint32_t MSxP1 = 128 * outDivider - 512;
    outCLKxMSxParams[0] = 0u; 								    // Bits [15:8] of MS0_P3 (always 0) in register 42
	outCLKxMSxParams[1] = 1u;            					    // Bits [7:0]  of MS0_P3 (always 1) in register 43
	outCLKxMSxParams[2] = (uint8_t)((MSxP1 & 0x030000L) >> 16); // Bits [17:16] of MSx_P1 in bits [1:0] and R in [7:4] | [3:2]
	outCLKxMSxParams[3] = (uint8_t)((MSxP1 & 0xFF00) >> 8);     // Bits [15:8]  of MSx_P1 in register 45
	outCLKxMSxParams[4] = (uint8_t)(MSxP1 & 0xFF);              // Bits [7:0]  of MSx_P1 in register 46
	outCLKxMSxParams[5] = 0u;             					    // Bits [19:16] of MS0_P2 and MS0_P3 are always 0
	outCLKxMSxParams[6] = 0u;                        		    // Bits [15:8]  of MS0_P2 are always 0
	outCLKxMSxParams[7] = 0u;                        		    // Bits [7:0]   of MS0_P2 are always 0
}


si5351_status_t si5351_Setup(si5351_t *si5351, const si5351_bus_t busAdaptor, const uint8_t i2cAddress, const uint32_t xtalFrequency, const int32_t xtalCorrection)
{
    if(!si5351) return SI5351_EINVAL;

    si5351->busAdaptor = busAdaptor;
    si5351->i2cAddress = i2cAddress;
    si5351->xtalFrequency = xtalFrequency + xtalCorrection;
    
    si5351_status_t commResult;
    for(uint8_t clk = 0; clk < SI5351_NUM_CLKS; ++clk)
    {
        si5351->clks[clk].outDivider = 0;
        commResult = si5351_ReadReg(si5351, SI5351_REG_CLK0_CONTROL + clk, &si5351->clks[clk].clkCtrlRegVal);
        if(commResult != SI5351_OK) return commResult;
    }

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
    regVal &= ~(1u << 7);
    commResult = si5351_WriteReg(si5351, SI5351_REG_SPREAD_SPECTRUM_PARAMS_1, regVal);
    if(commResult != SI5351_OK) return commResult;

    // Setting up the clock register (needed because CLKx should use MSx for output)
    const si5351_clk_ctrl_t clkCtrl = {
        .powerDown = false,
        .integerMode = true,
        .msPLLSource = SI5351_PLLA,
        .inverted = false,
        .outputCurrent = SI5351_OUT_2MA
    };
    for(uint8_t clk = 0; clk < SI5351_NUM_CLKS; ++clk)
    {   
        commResult = si5351_EnsureClkCtrl(si5351, clk, clkCtrl);
        if(commResult != SI5351_OK) return commResult;
    }

    return commResult;
}

si5351_status_t si5351_SetOutputCurrent(si5351_t* si5351, const si5351_clk_num_t clk, const si5351_clk_output_current_t outputCurrent)
{
    if(!si5351 || (clk >= SI5351_NUM_CLKS)) return SI5351_EINVAL;
    si5351_clk_ctrl_t clkCtrl = si5351_UnpackClkCtrlRegVal(si5351->clks[clk].clkCtrlRegVal);
    clkCtrl.outputCurrent = outputCurrent;
    return si5351_EnsureClkCtrl(si5351, clk, clkCtrl);
}

si5351_status_t si5351_SetFrequency(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clk, const uint32_t frequency)
{
    if (!si5351 || (clk >= SI5351_NUM_CLKS) || frequency == 0) return SI5351_EINVAL;
    si5351_status_t commResult;

    // Ensuring that clk is powered-up and in integer mode
    si5351_clk_ctrl_t clkCtrl = si5351_UnpackClkCtrlRegVal(si5351->clks[clk].clkCtrlRegVal);
    clkCtrl.powerDown = false;
    clkCtrl.integerMode = true;
    clkCtrl.msPLLSource = pll;
    clkCtrl.inverted = false;
    commResult = si5351_EnsureClkCtrl(si5351, clk, clkCtrl);
    if(commResult != SI5351_OK) return commResult;

    // Finding an even out divider 
    uint32_t outDivider = SI5351_PLL_MAX_FREQ / frequency;
    outDivider &= ~1u;

    // Calculating the actual frequency of PLLx
    uint32_t frequencyPLLx = outDivider * frequency;

    // Generating the register map for MSNx of PLLx
    uint8_t PLLxMSNxParams[8];
    si5351_GeneratePLLxMSNxParams(frequencyPLLx, si5351->xtalFrequency, PLLxMSNxParams);

    // Checking whether updating the CLK's MultiSynth (OMD) is necessary (PLL reset needed)
    if(outDivider != si5351->clks[clk].outDivider)
    {
        // Updating the outDivider of CLKx
        si5351->clks[clk].outDivider = outDivider;

        // Generating the register map for MSx of CLKx
        uint8_t CLKxMSxParams[8];
        si5351_GenerateCLKxMSxParams(outDivider, CLKxMSxParams);

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

si5351_status_t si5351_SetFrequencyIQ(si5351_t* si5351, const si5351_pll_num_t pll, const si5351_clk_num_t clkA, const si5351_clk_num_t clkB, const uint32_t frequency)
{
    if (!si5351 || (clkA >= SI5351_NUM_CLKS) || (clkB >= SI5351_NUM_CLKS) || frequency == 0) return SI5351_EINVAL;
    si5351_status_t commResult;

    // Ensuring that clkA is powered-up and in fractional mode
    si5351_clk_ctrl_t clkCtrl = si5351_UnpackClkCtrlRegVal(si5351->clks[clkA].clkCtrlRegVal);
    clkCtrl.powerDown = false;
    clkCtrl.integerMode = false;
    clkCtrl.msPLLSource = pll;
    clkCtrl.inverted = false;
    commResult = si5351_EnsureClkCtrl(si5351, clkA, clkCtrl);
    if(commResult != SI5351_OK) return commResult;

    // Ensuring that clkB is powered-up and in fractional mode
    clkCtrl = si5351_UnpackClkCtrlRegVal(si5351->clks[clkB].clkCtrlRegVal);
    clkCtrl.powerDown = false;
    clkCtrl.integerMode = false;
    clkCtrl.msPLLSource = pll;
    clkCtrl.inverted = false;
    commResult = si5351_EnsureClkCtrl(si5351, clkB, clkCtrl);
    if(commResult != SI5351_OK) return commResult;


    return commResult;
}


/*
si5351_status_t si5351_SetFrequencyIQ(si5351_t* si5351, si5351_pll_num_t pll, si5351_clk_num_t clkA, si5351_clk_num_t clkB, uint32_t frequency)
{
    if (!si5351 || frequency == 0) return SI5351_EINVAL;

    si5351_status_t commResult = SI5351_OK;
    if(si5351->clks[clkA].integerMode || si5351->clks[clkA].msPLLSource != pll || si5351->clks[clkB].msPLLSource != pll || si5351->clks[clkB].integerMode)
    {
        si5351->clks[clkA].integerMode = false;
        si5351->clks[clkA].msPLLSource = pll;
        si5351->clks[clkA].inverted = false;
        si5351->clks[clkB].integerMode = false;
        si5351->clks[clkB].msPLLSource = pll;
        si5351->clks[clkB].inverted = false;
        commResult = si5351_WriteClkCtrlRegister(si5351, clkA);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_WriteClkCtrlRegister(si5351, clkB);
        if(commResult != SI5351_OK) return commResult;
    }

    
    // Finding an even out divider 
    uint32_t outDivider = SI5351_PLL_MAX_FREQ / frequency;
    outDivider &= ~1u;

    if(outDivider > 126)
    {
        outDivider = 126;
    }
    

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

    uint8_t PLLxMSNxParams[8] =
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
    if(outDivider != si5351->clks[clkA].outDivider)
    {
        // Updating the outDivider
        si5351->clks[clkA].outDivider = outDivider;

        uint32_t MSxP1 = 128 * outDivider - 512;
        uint8_t CLKxMSxParams[8] =
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
        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH0_PARAMS + (SI5351_REG_MULTISYNTHx_PARAMS_OFFSET * clkA), CLKxMSxParams, 8);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH0_PARAMS + (SI5351_REG_MULTISYNTHx_PARAMS_OFFSET * clkB), CLKxMSxParams, 8);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_WriteReg(si5351, SI5351_REG_CLK0_PHASE_OFFSET + clkB, outDivider);
        if(commResult != SI5351_OK) return commResult;
        commResult = si5351_ResetPll(si5351, pll);
    }
    else
    {
        commResult = si5351_WriteRegs(si5351, SI5351_REG_MULTISYNTH_NA_PARAMS_BASE + (pll * SI5351_REG_MULTISYNTH_NA_PARAMS_NB_PARAMS_OFFSET), PLLxMSNxParams, 8);
    }

    return commResult;
}

*/