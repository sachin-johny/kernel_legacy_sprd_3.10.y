#ifndef __POWER_MANAGER_H__
#define __POWER_MANAGER_H__

#define ARM_CTL 0

extern void PWRMNG_ForcePowerOnVPll(void);

extern void PWRMNG_SetVpllPdSrc (int pdsrc);

extern void PWRMNG_SetVpll(unsigned int vpll_clk);

extern unsigned int CHIP_GetVPllClk (void);

extern void PWRMNG_SetUsbClkSrc(void);

extern void clk_12M_divider_set(unsigned int vpll_clk);
#endif
