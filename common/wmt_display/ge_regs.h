/*
 * ge_regs.h
 *
 * GE register API for DirectFB GFX driver.
 *
 * Supported architectures:
 * ARCH_VT8430, ARCH_WM8510, ARCH_WM8510_ECO2, ARCH_WM8435.
 *
 * Warnings! GE register API is for DirectFB GFX driver only.
 * The API will change without warnings in the future.
 * Please don't use it unless you know exactly what you are doing.
 * Please use GTK+/DirectFB for 2D acceleration application instead.
 *
 * Copyright 2008-2009 WonderMedia Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY WONDERMEDIA CORPORATION ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL WONDERMEDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GE_REGS_H
#define GE_REGS_H 1

#include "ge_regs_8430.h"
#include "ge_regs_8435.h"
#include "ge_regs_8510.h"

#define SCC_CHIP_ID	(*(unsigned int *)0xd8120000)

#define GE_MMIO_START	0xd8050000
#define GE_MMIO_OFFSET	0x400

#ifndef BIT0
#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000
#endif

#ifndef REG_SET32
#define REG_SET32(addr, val)	(*(volatile unsigned int *)(addr)) = val
#define REG_GET32(addr)		(*(volatile unsigned int *)(addr))
#define REG_VAL32(addr)		(*(volatile unsigned int *)(addr))
#define REG_SET16(addr, val)	(*(volatile unsigned short *)(addr)) = val
#define REG_GET16(addr)		(*(volatile unsigned short *)(addr))
#define REG_VAL16(addr)		(*(volatile unsigned short *)(addr))
#define REG_SET8(addr, val)	(*(volatile unsigned char *)(addr)) = val
#define REG_GET8(addr)		(*(volatile unsigned char *)(addr))
#define REG_VAL8(addr)		(*(volatile unsigned char *)(addr))
#endif

/* GL.h */

/* pixel formats */
#define GEPF_FMT(cd, hm) (((cd) << 4) + (hm & 0xf))
#define GEPF_RGB32  GEPF_FMT(3, 0)
#define GEPF_RGB24  GEPF_FMT(2, 0) /* Reserved */
#define GEPF_RGB16  GEPF_FMT(1, 0)
#define GEPF_RGB555 GEPF_FMT(1, 1)
#define GEPF_RGB454 GEPF_FMT(1, 2)
#define GEPF_LUT8   GEPF_FMT(0, 0)

/* GE commands for VT8430 (VT3357) */
#define GECMD_BLIT   0x1
#define GECMD_TEXT   0x2
#define GECMD_ROTATE 0x8
#define GECMD_MIRROR 0x9

/* GE commands for WM8510 (VT3426) */
#define GECMD_BLIT_DMA		0x3
#define GECMD_BEZIER		0x4
#define GECMD_LINE		0x7
#define GECMD_DMA		0xa

/* AMX CSC table */
#define AMX_CSC_DEFAULT		0
#define AMX_CSC_SDTV_16_235	1
#define AMX_CSC_SDTV_0_255	2
#define AMX_CSC_HDTV_16_235	3
#define AMX_CSC_HDTV_0_255	4
#define AMX_CSC_JFIF_0_255	5
#define AMX_CSC_SMPTE_170M	6
#define AMX_CSC_SMPTE_240M	7

typedef struct {
	void *mmio;
	unsigned int fd;
} ge_info_t;

typedef struct {
	unsigned int addr;
	unsigned int xres;
	unsigned int yres;
	unsigned int xres_virtual;
	unsigned int yres_virtual;
	unsigned int x;
	unsigned int y;
	unsigned int pixelformat;
} ge_surface_t;

extern int ge_sys_init(ge_info_t **geinfo, void *priv);
extern int ge_sys_exit(ge_info_t *geinfo, void *priv);
extern int ge_get_chip_id(ge_info_t *geinfo, unsigned int *chip_id);
extern int ge_lock(ge_info_t *geinfo);
extern int ge_unlock(ge_info_t *geinfo);
extern int ge_trylock(ge_info_t *geinfo);
extern void ge_wait_sync(ge_info_t *geinfo);
extern void WAIT_PXD_INT(void);

extern int ge_set_pixelformat(ge_info_t *geinfo, unsigned int pixelformat);
extern int ge_set_destination(ge_info_t *geinfo, ge_surface_t *dst);
extern int ge_set_source(ge_info_t *geinfo, ge_surface_t *src);
extern int ge_set_mask(ge_info_t *geinfo, ge_surface_t *src);
extern int ge_set_command(ge_info_t *geinfo, unsigned int cmd,
	unsigned int rop);
extern unsigned int ge_get_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt);
extern void ge_set_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt);
extern void ge_set_sck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt);
extern void ge_set_dck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt);
extern int ge_blit(ge_info_t *geinfo);
extern int ge_fillrect(ge_info_t *geinfo);
extern int ge_rotate(ge_info_t *geinfo, unsigned int arc);
extern int ge_mirror(ge_info_t *geinfo, int mode);

extern int amx_show_surface(ge_info_t *geinfo, int id, ge_surface_t *s,
	int x, int y);
extern int amx_get_surface(ge_info_t *geinfo, int id, ge_surface_t *s);
extern int amx_hide_surface(ge_info_t *geinfo, int id);
extern int amx_set_colorkey(ge_info_t *geinfo, int id, int enable,
	unsigned int r, unsigned int g, unsigned int b, unsigned int pixfmt);
extern int amx_set_alpha(ge_info_t *geinfo, int id, unsigned int alpha);
extern int amx_sync(ge_info_t *geinfo);

/* New features */
extern int amx_set_alpha_wm8510_eco2(ge_info_t *geinfo, unsigned int alpha);
extern int amx_set_alpha_wm8435(ge_info_t *geinfo, unsigned int alpha);
extern int amx_set_csc(ge_info_t *geinfo, int table_id);
extern int ge_fillrect2(ge_surface_t *d, unsigned int color);
#endif /* GE_REGS_H */

