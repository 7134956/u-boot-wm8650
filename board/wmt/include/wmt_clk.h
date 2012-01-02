/*++ 
Copyright (c) 2010 WonderMedia Technologies, Inc.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details. You
should have received a copy of the GNU General Public License along with this
program. If not, see http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.
10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

#ifndef _WMT_CLK_H_
#define _WMT_CLK_H_

/*
 *   WMT_CLK register struct
 *
 */

enum dev_id {
 DEV_UART0 = 1,
 DEV_UART1,
 DEV_SDMMC1 = 4,
 DEV_I2C0,
 DEV_I2S,
 DEV_RTC,
 DEV_KEYPAD,
 DEV_I2C1,
 DEV_PWM,
 DEV_GPIO,
 DEV_SPI0,
 DEV_CAMERA = 16,
 DEV_CIR,
 DEV_GOVRHD,
 DEV_VID,
 DEV_VDU,
 DEV_SCC,
 DEV_DSP = 25,
 DEV_MBOX = 28,
 DEV_GE,
 DEV_SCL444U,
 DEV_GOVW,
 DEV_DDRMC,  /* PMC upper register offset 0xd8130254 */
 DEV_NA0,
 DEV_NA12,
 DEV_DVO = 36,			
 DEV_DMA,			
 DEV_ROT,			
 DEV_UHDC,
 DEV_PERM,
 DEV_DSPCFG,
 DEV_AHBB = 45,
 DEV_NAND = 48,
 DEV_SDMMC0 = 50,
 DEV_ETHMAC = 52,
 DEV_SF = 55,
 DEV_ETHPHY = 58,
 DEV_VPP = 63,
 DEV_ARM, /* number >= 64 has no clk_en to enable clk */
 DEV_AHB,
 DEV_APB,
 DEV_PCM,			
 DEV_JDEC,
 DEV_MSVD,
 DEV_AMP,			
 DEV_HDMI,
 DEV_DISP,
 DEV_VPU,			
};

enum clk_cmd {
CLK_DISABLE = 0,
CLK_ENABLE,
SET_DIV,
SET_PLL,
SET_PLLDIV
};
#endif /* __WMT_CLK_H__*/
