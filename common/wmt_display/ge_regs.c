/*
 * ge_regs.c
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

#include <malloc.h>
#include "ge_regs.h"

/* GE core functions */
#ifndef INLINE
#define INLINE
#endif

#define printk printf

INLINE int ge_sys_init(ge_info_t **geinfo, void *priv)
{
	*geinfo = malloc(sizeof(ge_info_t));

	if (*geinfo)
		(*geinfo)->mmio = (void *)(GE_MMIO_START + GE_MMIO_OFFSET);
	else
		return -1;

	return 0;
}

INLINE int ge_sys_exit(ge_info_t *geinfo, void *priv)
{
	free(geinfo);

	return 0;
}

INLINE int ge_get_chip_id(ge_info_t *geinfo, unsigned int *chip_id)
{
	*chip_id = SCC_CHIP_ID;

	return 0;
}

/* PUT SYSTEM INDEPENDANT FUNCTIONS HERE */

INLINE int ge_set_pixelformat(ge_info_t *geinfo, unsigned int pixelformat)
{
	volatile struct ge_regs_8430 *regs;
	int ret = 0;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	switch (pixelformat) {
	case GEPF_LUT8:
		regs->color_depth = 0;
		break;
	case GEPF_RGB16:
		regs->color_depth = 1;
		regs->hm_sel = 0;
		break;
	case GEPF_RGB555:
		regs->color_depth = 1;
		regs->hm_sel = 1;
		break;
	case GEPF_RGB454:
		regs->color_depth = 1;
		regs->hm_sel = 2;
		break;
	case GEPF_RGB32:
		regs->color_depth = 3;
		break;
	default:
		ret = -1;
	}

	return ret;
}

INLINE int ge_set_destination(ge_info_t *geinfo, ge_surface_t *dst)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	regs->des_baddr = dst->addr;
	regs->des_disp_w = dst->xres_virtual - 1;
	regs->des_disp_h = dst->yres_virtual - 1;

	regs->des_x_start = dst->x;
	regs->des_y_start = dst->y;
	regs->des_width = dst->xres - 1;
	regs->des_height = dst->yres - 1;

	return ge_set_pixelformat(geinfo, dst->pixelformat);
}

INLINE int ge_set_source(ge_info_t *geinfo, ge_surface_t *src)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	regs->src_baddr = src->addr;
	regs->src_disp_w = src->xres_virtual - 1;
	regs->src_disp_h = src->yres_virtual - 1;

	regs->src_x_start = src->x;
	regs->src_y_start = src->y;
	regs->src_width = src->xres - 1;
	regs->src_height = src->yres - 1;

	return 0;
}

INLINE int ge_set_mask(ge_info_t *geinfo, ge_surface_t *src)
{
	volatile struct ge_regs_8435 *regs;

	regs = (struct ge_regs_8435 *)geinfo->mmio;

	regs->mask_baddr = src->addr;
	regs->mask_disp_w = src->xres_virtual - 1;
	regs->mask_disp_h = src->yres_virtual - 1;

	regs->mask_x_start = src->x;
	regs->mask_y_start = src->y;
	regs->mask_width = src->xres - 1;
	regs->mask_height = src->yres - 1;

	return 0;
}

INLINE int ge_set_command(ge_info_t *geinfo, unsigned int cmd, unsigned int rop)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	regs->ge_command = cmd;
	regs->rop_code = rop;	/* cc = s->d, 0f = p->d */
	regs->ge_eng_en = 1;
	regs->ge_fire = 1;

	return 0;
}

INLINE unsigned int ge_get_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	unsigned int color;

	switch (pixfmt) {
	case GEPF_LUT8:
		color = a;
		break;
	case GEPF_RGB16:
		color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
		break;
	case GEPF_RGB555:
		color = ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
		break;
	case GEPF_RGB454:
		color = ((r >> 4) << 9) | ((g >> 3) << 4) | (b >> 4);
		break;
	case GEPF_RGB32:
		color = (a << 24) | (r << 16) | (g << 8) | (b) ;
		break;
	default:
		color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
		break;
	}

	return color;
}

INLINE void ge_set_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->pat0_color = color;
}

INLINE void ge_set_sck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->src_ck = color;
	regs->ck_sel |= BIT2;
}

INLINE void ge_set_dck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->des_ck = color;
	regs->ck_sel |= BIT3 | BIT1;
}

INLINE int ge_blit(ge_info_t *geinfo)
{
	return ge_set_command(geinfo, GECMD_BLIT, 0xcc);
}

INLINE int ge_fillrect(ge_info_t *geinfo)
{
	return ge_set_command(geinfo, GECMD_BLIT, 0xf0);
}

INLINE int ge_rotate(ge_info_t *geinfo, unsigned int arc)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int chip_id;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	/*
	 * Rotation function requires LUT disabled
	 *
	 * VT8430: G2_LUT and G3_LUT have to be disabled.
	 * WM8510: G2_LUT has to be disabled.
	 * VT8435: nothing to do.
	 */

	ge_get_chip_id(geinfo, &chip_id);
	chip_id >>= 16;

	switch (chip_id) {
	case 0x3357:	/* VT8430 */
		regs->g2_lut_en = 0;
		regs->g3_lut_en = 0;
		break;
	case 0x3426:	/* WM8510 */
		regs->g2_lut_en = 0;
		break;
	case 0x3437:	/* WM8435 */
	default:
		/* Nothing to do here... */
		break;
	}

	switch (arc % 360) {
	case 0:
		regs->rotate_mode = 0;
		break;
	case 90:
		regs->rotate_mode = 1;
		break;
	case 180:
		regs->rotate_mode = 2;
		break;
	case 270:
		regs->rotate_mode = 3;
		break;
	default:
		break;
	}
	ge_set_command(geinfo, GECMD_ROTATE, 0);

	//ge_wait_sync(geinfo); /* Required. */ //fan

	return 0;
}

INLINE int ge_mirror(ge_info_t *geinfo, int mode)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->mirror_mode = mode & BIT0;

	ge_set_command(geinfo, GECMD_MIRROR, 0);

	return 0;
}

/* AMX barebone functions */

INLINE int amx_show_surface(ge_info_t *geinfo,
	int id, ge_surface_t *s, int x, int y)
{
	volatile struct ge_regs_8430 *regs;
	volatile unsigned int *gov_regs;
	unsigned int cd; /* color depth */
	unsigned int hm; /* hi color mode */
	unsigned int gov_dspcr;

	cd = (s->pixelformat) >> 4;
	hm = (s->pixelformat) & 0xf;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	gov_regs = (unsigned int *)geinfo->mmio;
	gov_regs -= 64;
	gov_dspcr = gov_regs[7];

	if (gov_dspcr) {
		/* if gov was initialized ... */
		regs->disp_x_end = gov_dspcr & 0x7ff;
		regs->disp_y_end = gov_dspcr >> 16;
	} else {
		/* if gov is not initialized yet ... */
		regs->disp_x_end = s->xres_virtual - 1;
		regs->disp_y_end = s->yres_virtual - 1;
	}

	if (id == 0) {
		/* G1 */
		regs->g1_cd = cd;
		regs->g1_amx_hm = hm;

		/* If FG being used, use BG. */
		if (regs->g1_fb_sel == 0) {
			regs->g1_bg_addr = s->addr;
			regs->g1_fb_sel = 1;
		} else {
			regs->g1_fg_addr = s->addr;
			regs->g1_fb_sel = 0;
		}

		regs->g1_fbw = s->xres_virtual;
		regs->g1_hcrop = s->x;
		regs->g1_vcrop = s->y;
		regs->g1_x_start =  x;
		regs->g1_x_end = x + s->xres - 1;
		regs->g1_y_start = y;
		regs->g1_y_end = y + s->yres - 1;
		regs->g1_amx_en = 1;
	} else {
		/* G2 */
		regs->g2_cd = cd;
		regs->g2_amx_hm = hm;

		/* If FG being used, use BG. */
		if (regs->g2_fb_sel == 0) {
			regs->g2_bg_addr = s->addr;
			regs->g2_fb_sel = 1;
		} else {
			regs->g2_fg_addr = s->addr;
			regs->g2_fb_sel = 0;
		}

		regs->g2_fbw = s->xres_virtual;
		regs->g2_hcrop = s->x;
		regs->g2_vcrop = s->y;
		regs->g2_x_start =  x;
		regs->g2_x_end = x + s->xres - 1;
		regs->g2_y_start = y;
		regs->g2_y_end = y + s->yres - 1;
		regs->g2_amx_en = 1;
	}

	return 0;
}

INLINE int amx_get_surface(ge_info_t *geinfo, int id, ge_surface_t *s)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int cd; /* color depth */
	unsigned int hm; /* hi color mode */

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	if (id == 0) {
		/* G1 */
		cd = regs->g1_cd;
		hm = regs->g1_amx_hm;

		s->addr = regs->g1_fb_sel ?
			regs->g1_bg_addr : regs->g1_fg_addr;
		s->x = regs->g1_hcrop;
		s->y = regs->g1_vcrop;
		s->xres = regs->g1_x_end - regs->g1_x_start + 1;
		s->yres = regs->g1_y_end - regs->g1_y_start + 1;
		s->xres_virtual = regs->g1_fbw;
		s->yres_virtual = s->yres;
	} else {
		/* G2 */
		cd = regs->g2_cd;
		hm = regs->g2_amx_hm;

		s->addr = regs->g2_fb_sel ?
			regs->g2_bg_addr : regs->g2_fg_addr;
		s->x = regs->g2_hcrop;
		s->y = regs->g2_vcrop;
		s->xres = regs->g2_x_end - regs->g2_x_start + 1;
		s->yres = regs->g2_y_end - regs->g2_y_start + 1;
		s->xres_virtual = regs->g2_fbw;
		s->yres_virtual = s->yres;
	}

	s->pixelformat = GEPF_FMT(cd, hm);

	return 0;
}

INLINE int amx_hide_surface(ge_info_t *geinfo, int id)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	switch (id) {
	case 0:
		regs->g1_amx_en = 0;
		break;
	case 1:
		regs->g2_amx_en = 0;
		break;
	default:
		break;
	}
	return 0;
}

INLINE int amx_set_colorkey(ge_info_t *geinfo, int id, int enable,
	unsigned int r, unsigned int g, unsigned int b, unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;

	switch (pixfmt) {
	case GEPF_RGB16:
		r &= ~0x7;
		g &= ~0x3;
		b &= ~0x7;
		break;
	case GEPF_RGB555:
		r &= ~0x7;
		g &= ~0x7;
		b &= ~0x7;
		break;
	case GEPF_RGB454:
		r &= ~0xf;
		g &= ~0x7;
		b &= ~0xf;
		break;
	}

	switch (id) {
	case 0:
		regs->g1_ck_en = enable;
		regs->g1_c_key = enable ?
			((r << 16) | (g << 8) | (b)) : 0;
		break;
	case 1:
		regs->g2_ck_en = enable;
		regs->g2_c_key = enable ?
			((r << 16) | (g << 8) | (b)) : 0;
		break;
	default:
		break;
	}

	return 0;
}

INLINE int amx_set_alpha_vt8430(ge_info_t *geinfo, int id, unsigned int alpha)
{
	volatile struct ge_regs_8430 *regs;

	/*
	 * GE = G1 * ALPHA_1 + G2 * (1 - ALPHA_1)
	 * VOUT = GE * (1 - ALPHA_2) + VPU * ALPHA2
	 */

	/*
	 *	G2_HIT	G1_HIT	G2_CK	G1_CK	ALPHA_1	ALPHA_2	VT8430
	 *	0	0	0	0	X (FF)	FF	Yes
	 *	0	1	0	0	FF	00	Yes
	 *	1	0	0	0	00	00	Yes
	 *	1	1	0	0	00	00	Yes
	 *
	 *	0	1	0	1	X (00)	FF	Yes
	 *	1	0	1	0	00	FF	No
	 *	1	1	0	1	00	00	Yes
	 *	1	1	1	0	FF	00	Yes
	 *	1	1	1	1	X	FF	No
	 */

	/*
	 *	GE AMX setting for VT8430
	 *
	 *	{MSB,...,LSB}
	 *
	 *	GE_FIX_APA = {00,00,ff,ff}
	 *	GE_FIX2_APA = {00,00,00,ff}
	 *	GE_AMX_CTL = {N,N,N,N,0,0,0,0}
	 *	GE_AMX2_CTL = {N,N,X,1,1,0,0,0}
	 *	GE_CK_APA = {N,N,N,N,N,0,1,0}
	 */

	alpha &= 0xff;

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_fix2_apa = 0xff;
	regs->ge_amx_ctl = 0;
	regs->ge_amx2_ctl = BIT4 | BIT3;
	regs->ge_ck_apa = BIT1;

	return 0;
}

INLINE int amx_set_alpha_wm8510_eco2(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8510 *regs;

	alpha &= 0xff;

	regs = (struct ge_regs_8510 *)geinfo->mmio;

	regs->ge_amx_ctl = 0;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT1;

	regs->ge_amx2_ctl = 0xf;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck_apa |= BIT8 | BIT7 | BIT5 | BIT3;

	return 0;
}

INLINE int amx_set_alpha_wm8435(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8435 *regs;

	alpha &= 0xff;

	regs = (struct ge_regs_8435 *)geinfo->mmio;

	regs->ge_amx_ctl = 0;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT9;

	regs->ge_amx2_ctl = 0x76;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck2_apa = BIT11 | BIT7 | BIT3;

	return 0;
}

INLINE int amx_set_alpha_wm8440(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8435 *regs;

	alpha &= 0xff;

	regs = (struct ge_regs_8435 *)geinfo->mmio;

	regs->ge_amx_ctl = 0;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT10;

	regs->ge_amx2_ctl = 0x76;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck2_apa = BIT11 | BIT7 | BIT3;

	return 0;
}

INLINE int amx_set_alpha_wm8650(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8435 *regs;

	alpha &= 0xff;

	regs = (struct ge_regs_8435 *)geinfo->mmio;

	regs->ge_amx_ctl = BIT4;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT10;

	regs->ge_amx2_ctl = 0x76;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck2_apa = BIT11 | BIT7 | BIT3;

	return 0;
}

INLINE int amx_set_alpha(ge_info_t *geinfo, int id, unsigned int alpha)
{
	unsigned int chip_id;
	unsigned int chip_version;

	ge_get_chip_id(geinfo, &chip_id);
	chip_version = chip_id & 0xffff;

	chip_id >>= 16;

	alpha &= 0xff;

	if (id)
		alpha = 0xff - alpha;

	switch (chip_id) {
	case 0x3357:	/* VT8430 */
		amx_set_alpha_vt8430(geinfo, 0, alpha);
		break;
	case 0x3426:	/* WM8510 */
		if (chip_version > 0x0102) /* WM8510 A1 */
			amx_set_alpha_wm8510_eco2(geinfo, alpha);
		else
			amx_set_alpha_vt8430(geinfo, 0, alpha);
		break;
	case 0x3429:	/* WM8425 */
	case 0x3437:	/* WM8435 */
		amx_set_alpha_wm8435(geinfo, alpha);
		break;
	case 0x3451:	/* WM8440 */
		amx_set_alpha_wm8440(geinfo, alpha);
		break;
	default:
	case 0x3465:	/* WM8650 */
		amx_set_alpha_wm8650(geinfo, alpha);
		break;
	}

	return 0;
}

INLINE int amx_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	regs = (struct ge_regs_8430 *)geinfo->mmio;
	regs->ge_reg_sel = 0; /* select level1 registers */
	regs->ge_reg_upd = 1; /* update level2 registers */

	return 0;
}

INLINE int amx_set_csc(ge_info_t *geinfo, int table_id)
{
	volatile struct ge_regs_8510 *regs;

	regs = (struct ge_regs_8510 *)geinfo->mmio;

	switch (table_id) {
	case AMX_CSC_DEFAULT:
		break;
	case AMX_CSC_SDTV_16_235:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f50;
		regs->c5_coef = 0x1ea5;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e4a;
		regs->c9_coef = 0x1fab;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SDTV_0_255:
		regs->c1_coef = 0x107;
		regs->c2_coef = 0x204;
		regs->c3_coef = 0x64;
		regs->c4_coef = 0x1f68;
		regs->c5_coef = 0x1ed6;
		regs->c6_coef = 0x1c2;
		regs->c7_coef = 0x1c2;
		regs->c8_coef = 0x1e78;
		regs->c9_coef = 0x1fb7;
		regs->coef_i = 0x20;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_HDTV_16_235:
		regs->c1_coef = 0xda;
		regs->c2_coef = 0x2dc;
		regs->c3_coef = 0x4a;
		regs->c4_coef = 0x1f88;
		regs->c5_coef = 0x1e6d;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e25;
		regs->c9_coef = 0x1fd0;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_HDTV_0_255:
		regs->c1_coef = 0xbb;
		regs->c2_coef = 0x275;
		regs->c3_coef = 0x3f;
		regs->c4_coef = 0x1f99;
		regs->c5_coef = 0x1ea6;
		regs->c6_coef = 0x1c2;
		regs->c7_coef = 0x1c2;
		regs->c8_coef = 0x1e67;
		regs->c9_coef = 0x1fd7;
		regs->coef_i = 0x20;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_JFIF_0_255:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f53;
		regs->c5_coef = 0x1ead;
		regs->c6_coef = 0x200;
		regs->c7_coef = 0x200;
		regs->c8_coef = 0x1e53;
		regs->c9_coef = 0x1fad;
		regs->coef_i = 0x0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SMPTE_170M:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f50;
		regs->c5_coef = 0x1ea5;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e4a;
		regs->c9_coef = 0x1fab;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SMPTE_240M:
		regs->c1_coef = 0xd9;
		regs->c2_coef = 0x2ce;
		regs->c3_coef = 0x59;
		regs->c4_coef = 0x2f89;
		regs->c5_coef = 0x1e77;
		regs->c6_coef = 0x200;
		regs->c7_coef = 0x200;
		regs->c8_coef = 0x1e38;
		regs->c9_coef = 0x1fc8;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	}

	return 0;
}

