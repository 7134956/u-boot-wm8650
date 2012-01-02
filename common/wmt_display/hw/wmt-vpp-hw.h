/*++ 
 * linux/drivers/video/wmt/hw/wmt-vpp-hw.h
 * WonderMedia video post processor (VPP) driver
 *
 * Copyright c 2010  WonderMedia  Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 4F, 533, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/

#ifndef WMT_VPP_HW_H
#define WMT_VPP_HW_H

/*-------------------- DEPENDENCY -------------------------------------*/
#include "wmt-vpp-reg.h"
#include "wmt-vpu-reg.h"
#include "wmt-govm-reg.h"
#include "wmt-govw-reg.h"
#include "wmt-govrh-reg.h"

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/*
* Product ID / Project ID
* 84xx series: 8420/3300, 8430/3357, 8435/3437
* 85xx series: 8500/3400, 8510/3426, 8520/3429
*/
//84xx series, (1-100) with VDU & DSP
#define VIA_PID_8420	10	// 3300
#define VIA_PID_8430	12	// 3357
#define WMT_PID_8435	14	// 3437
#define WMT_PID_8440	16	// 3451
#define WMT_PID_8425	18	// 3429
#define WMT_PID_8710	20	// 3445

//85xx series, (101-200)
#define VIA_PID_8500	110	// 3400
#define WMT_PID_8505	111
#define WMT_PID_8510	112	// 3426*

// 
#define WMT_PID_8650	210	// 3465

//current pid
#define WMT_CUR_PID		WMT_PID_8650

// #define WMT_SUB_PID		WMT_PID_8505
#ifndef WMT_SUB_PID
	#define WMT_SUB_PID		0
#endif

// VPP interrupt map to irq
#define VPP_IRQ_SCL_FINISH	IRQ_VPP_IRQ0
#define VPP_IRQ_SCL			IRQ_VPP_IRQ1
#define VPP_IRQ_SCL444_TG	IRQ_VPP_IRQ2
#define VPP_IRQ_VPPM		IRQ_VPP_IRQ3
#define VPP_IRQ_GOVW_TG		IRQ_VPP_IRQ4
#define VPP_IRQ_GOVW		IRQ_VPP_IRQ5
#define VPP_IRQ_GOVM		IRQ_VPP_IRQ6
#define VPP_IRQ_GE			IRQ_VPP_IRQ7
#define VPP_IRQ_GOVRH_TG	IRQ_VPP_IRQ8
#define VPP_IRQ_DVO			IRQ_VPP_IRQ9
#define VPP_IRQ_VID			IRQ_VPP_IRQ10
#define VPP_IRQ_GOVR		IRQ_VPP_IRQ11
#define VPP_IRQ_GOVRS_TG	IRQ_VPP_IRQ12
#define VPP_IRQ_VPU			IRQ_VPP_IRQ13
#define VPP_IRQ_VPU_TG		IRQ_VPP_IRQ14
#define VPP_IRQ_HDCP		IRQ_VPP_IRQ15
#define VPP_IRQ_HDMI_HPDH	IRQ_VPP_IRQ16
#define VPP_IRQ_HDMI_HPDL	IRQ_VPP_IRQ17
#define VPP_IRQ_18			IRQ_VPP_IRQ18

// platform hw define
#define VPP_DIV_VPP			VPP_DIV_NA12
#define VPP_VOINT_NO		0
#define VPP_BLT_PWM_NUM		0

// vout enable
#ifdef CONFIG_WMT_VGA
//#define WMT_FTBLK_VOUT_VGA
#endif
#define WMT_FTBLK_VOUT_DVI
#define WMT_FTBLK_VOUT_DVO2HDMI
#define WMT_FTBLK_VOUT_LCD
#ifdef CONFIG_WMT_INTTV
//#define WMT_FTBLK_VOUT_SDA
#endif
//#define WMT_FTBLK_VOUT_SDD
//#define WMT_FTBLK_VOUT_HDMI
//#define WMT_FTBLK_VOUT_LVDS




#endif //WMT_VPP_HW_H

