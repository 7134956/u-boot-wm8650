/*
 * ge_regs_8440.h
 *
 * GE register API for DirectFB GFX driver.
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

struct ge_regs_8440 {
	unsigned int ge_command;	/* 0x0000 GE Command */
	unsigned int color_depth;	/* 0x0004 GE Color Depth */
	unsigned int hm_sel;		/* 0x0008 GE High Color Mode Select */
	unsigned int pat_tran_en;	/* 0x000c GE Pattern Transp. Enable */

	unsigned int font_tran_en;	/* 0x0010 GE Font Transparency Enable */
	unsigned int rop_code;		/* 0x0014 GE ROP Code */
	unsigned int ge_fire;		/* 0x0018 GE Fire Code */
	unsigned int rop_bg_code;	/* 0x001c */

	unsigned int src_baddr;		/* 0x0020 GE Source Base Address */
	unsigned int src_disp_w;	/* 0x0024 GE Source Display width */
	unsigned int src_disp_h;	/* 0x0028 GE Source DIsplay height */
	unsigned int src_x_start;	/* 0x002c GE Source X start point */

	unsigned int src_y_start;	/* 0x0030 GE Source Y start point */
	unsigned int src_width;		/* 0x0034 GE Source Width */
	unsigned int src_height;	/* 0x0038 GE Source Height */
	unsigned int des_baddr;		/* 0x003c GE Dest Base Address */

	unsigned int des_disp_w;	/* 0x0040 GE Dest Display width */
	unsigned int des_disp_h;	/* 0x0044 GE Dest DIsplay height */
	unsigned int des_x_start;	/* 0x0048 GE Dest X start point */
	unsigned int des_y_start;	/* 0x004c GE Dest Y start point */

	unsigned int des_width;		/* 0x0050 GE Dest Width */
	unsigned int des_height;	/* 0x0054 GE Dest Height */
	unsigned int font0_buf;		/* 0x0058 GE FONT Color 0 Buffer */
	unsigned int font1_buf;		/* 0x005c GE FONT Color 1 Buffer */

	unsigned int font2_buf;		/* 0x0060 GE FONT Color 2 Buffer*/
	unsigned int font3_buf;		/* 0x0064 GE FONT Color 3 Buffer*/
	unsigned int pat0_buf;		/* 0x0068 GE Pattern 0 Buffer */
	unsigned int pat1_buf;		/* 0x006c GE Pattern 1 Buffer */

	unsigned int pat2_buf;		/* 0x0070 GE Pattern 2 Buffer */
	unsigned int pat3_buf;		/* 0x0074 GE Pattern 3 Buffer */
	unsigned int pat4_buf;		/* 0x0078 GE Pattern 4 Buffer */
	unsigned int pat5_buf;		/* 0x007c GE Pattern 5 Buffer */

	unsigned int pat6_buf;		/* 0x0080 GE Pattern 6 Buffer */
	unsigned int pat7_buf;		/* 0x0084 GE Pattern 7 Buffer */
	unsigned int pat0_color;	/* 0x0088 GE Pattern 0 Color */
	unsigned int pat1_color;	/* 0x008c GE Pattern 1 Color */

	unsigned int pat2_color;	/* 0x0090 GE Pattern 2 Color */
	unsigned int pat3_color;	/* 0x0094 GE Pattern 3 Color */
	unsigned int pat4_color;	/* 0x0098 GE Pattern 4 Color */
	unsigned int pat5_color;	/* 0x009c GE Pattern 5 Color */

	unsigned int pat6_color;	/* 0x00a0 GE Pattern 6 Color */
	unsigned int pat7_color;	/* 0x00a4 GE Pattern 7 Color */
	unsigned int pat8_color;	/* 0x00a8 GE Pattern 8 Color */
	unsigned int pat9_color;	/* 0x00ac GE Pattern 9 Color */

	unsigned int pat10_color;	/* 0x00b0 GE Pattern 10 Color */
	unsigned int pat11_color;	/* 0x00b4 GE Pattern 11 Color */
	unsigned int pat12_color;	/* 0x00b8 GE Pattern 12 Color */
	unsigned int pat13_color;	/* 0x00bc GE Pattern 13 Color */

	unsigned int pat14_color;	/* 0x00c0 GE Pattern 14 Color */
	unsigned int pat15_color;	/* 0x00c4 GE Pattern 15 Color */
	unsigned int ck_sel;		/* 0x00c8 Color Key Select */
	unsigned int src_ck;		/* 0x00cc GE Source Colorkey */

	unsigned int des_ck;		/* 0x00d0 GE Destination Colorkey */
	unsigned int alpha_sel;		/* 0x00d4 GE Alpha Select */
	unsigned int bitblt_alpha;	/* 0x00d8 */
	unsigned int reserved;		/* 0x00dc Reserved */

	unsigned int rotate_mode;	/* 0x00e0 GE Rotate Mode */
	unsigned int mirror_mode;	/* 0x00e4 GE Mirror Mode */
	unsigned int ge_delay;		/* 0x00e8 GE Start Cycle Delay */
	unsigned int ge_eng_en;		/* 0x00ec GE Engine Enable */

	unsigned int ge_int_en;		/* 0x00f0 GE Intr. Enable signal */
	unsigned int ge_int_flag;	/* 0x00f4 GE Interrupt Flag */
	unsigned int ge_status;		/* 0x00f8 GE AMX Status */
	unsigned int ge_swid;		/* 0x00fc GE Software Identify */

	unsigned int ln_x_start;	/* 0x0100 */
	unsigned int ln_x_end;		/* 0x0104 */
	unsigned int ln_y_start;	/* 0x0108 */
	unsigned int reserved2;		/* 0x010c Reserved */

	unsigned int ln_y_end;		/* 0x0110 */
	unsigned int ln_tck;		/* 0x0114 */
	unsigned int reserved3;		/* 0x0118 Reserved */
	unsigned int c1_coef;		/* 0x011c */

	unsigned int ln_stl_tb;		/* 0x0120 */
	unsigned int ln_stl_rtn;	/* 0x0124 */
	unsigned int ln_stl_data;	/* 0x0128 */
	unsigned int ln_stl_apa;	/* 0x012c */

	unsigned int bc_p1x;		/* 0x0130 */
	unsigned int bc_p1y;		/* 0x0134 */
	unsigned int bc_p2x;		/* 0x0138 */
	unsigned int bc_p2y;		/* 0x013c */

	unsigned int bc_p3x;		/* 0x0140 */
	unsigned int bc_p3y;		/* 0x0144 */
	unsigned int bc_color;		/* 0x0148 */
	unsigned int bc_alpha;		/* 0x014c */

	unsigned int bc_delta_t;	/* 0x0150 */
	unsigned int bc_l_stl;		/* 0x0154 */
	unsigned int bc_l_stl_rtn;	/* 0x0158 */
	unsigned int c2_coef;		/* 0x015c */

	unsigned int c3_coef;		/* 0x0160 */
	unsigned int c4_coef;		/* 0x0164 */
	unsigned int c5_coef;		/* 0x0168 */
	unsigned int c6_coef;		/* 0x016c */

	unsigned int c7_coef;		/* 0x0170 */
	unsigned int c8_coef;		/* 0x0174 */
	unsigned int reserved4[2];	/* 0x0178 - 0x017c Reserved */

	unsigned int vq_en;		/* 0x0180 VQ Enable */
	unsigned int vq_size;		/* 0x0184 VQ Buffer Size */
	unsigned int vq_udptr;		/* 0x0188 VQ Pointer Update */
	unsigned int vq_baseaddr;	/* 0x018c VQ Buffer Base Address */

	unsigned int vq_wrsize;		/* 0x0190 VQ Free Buffer Space */
	unsigned int vq_staddrw;	/* 0x0194 DRAM Access Address */
	unsigned int vq_thr;		/* 0x0198 VQ Lower Intr. Threshold */
	unsigned int reserved5;		/* 0x019c Reserved */

	unsigned int rop4_en;		/* 0x01a0 */
	unsigned int alpha_plane_en;	/* 0x01a4 */
	unsigned int mask_baddr;	/* 0x01a8 */
	unsigned int mask_disp_w;	/* 0x01ac */

	unsigned int mask_disp_h;	/* 0x01b0 */
	unsigned int mask_x_start;	/* 0x01b4 */
	unsigned int mask_y_start;	/* 0x01b8 */
	unsigned int mask_width;	/* 0x01bc */

	unsigned int mask_height;	/* 0x01c0 */
	unsigned int dw_mask_baddr;	/* 0x01c4 */
	unsigned int alpha_plane_wbe;	/* 0x01c8 */
	unsigned int reserved6;		/* 0x01cc */

	unsigned int adap_blend_en;	/* 0x01d0 (New!) */
	unsigned int src_alpha_sel;	/* 0x01d4 (New!) */
	unsigned int src_blend_apa;	/* 0x01d8 (New!) */
	unsigned int des_alpha_sel;	/* 0x01dc (New!) */

	unsigned int des_blend_apa;	/* 0x01e0 (New!) */
	unsigned int adap_clamp_en;	/* 0x01e4 (New!) */
	unsigned int reserved7[2];	/* 0x01e8 - 0x01ec Reserved */

	unsigned int c9_coef;		/* 0x01f0 */
	unsigned int coef_i;		/* 0x01f4 */
	unsigned int coef_j;		/* 0x01f8 */
	unsigned int coef_k;		/* 0x01fc */

	unsigned int g1_cd;		/* 0x0200 G1 Color Depth */
	unsigned int g2_cd;		/* 0x0204 G2 Color Depth */
	unsigned int reserved8[2];	/* 0x0208 - 0x020c Reserved */

	unsigned int g1_fg_addr;	/* 0x0210 G1 FG Start Address */
	unsigned int g1_bg_addr;	/* 0x0214 G1 BG Start Address */
	unsigned int g1_fb_sel;		/* 0x0218 G1 Framebuffer Select */
	unsigned int g2_fg_addr;	/* 0x021c G2 FG Start Address */

	unsigned int g2_bg_addr;	/* 0x0220 G2 BG Start Address */
	unsigned int g2_fb_sel;		/* 0x0224 G2 Framebuffer Select */
	unsigned int reserved9[2];	/* 0x0228 - 0x022c Reserved */

	unsigned int g1_x_start;	/* 0x0230 G1 X-COOR Start Point */
	unsigned int g1_x_end;		/* 0x0234 G1 X-COOR End Point */
	unsigned int g1_y_start;	/* 0x0238 G1 Y-COOR Start Point */
	unsigned int g1_y_end;		/* 0x023c G1 Y-COOR End Point */

	unsigned int g2_x_start;	/* 0x0240 G2 X-COOR Start Point */
	unsigned int g2_x_end;		/* 0x0244 G2 X-COOR End Point */
	unsigned int g2_y_start;	/* 0x0248 G2 Y-COOR Start Point */
	unsigned int g2_y_end;		/* 0x024c G2 Y-COOR End Point */

	unsigned int disp_x_end;	/* 0x0250 GE Display X End Point */
	unsigned int disp_y_end;	/* 0x0254 GE Display Y End Point */
	unsigned int ge_amx_cb;		/* 0x0258 GE AMX color bar (New!) */
	unsigned int reserved10;	/* 0x025c Reserved */

	unsigned int reserved11[12];	/* 0x0260 - 0x028c Reserved */

	unsigned int reserved12[2];	/* 0x0290 - 0x0294 Reserved */
	unsigned int g1_ck_en;		/* 0x0298 G1 Color Key Enable */
	unsigned int g2_ck_en;		/* 0x029c G2 Color Key Enable */

	unsigned int g1_c_key;		/* 0x02a0 G1 Color Key */
	unsigned int g2_c_key;		/* 0x02a4 G2 Color Key */
	unsigned int g1_amx_en;		/* 0x02a8 G1 Alpha Mixing Enable */
	unsigned int g2_amx_en;		/* 0x02ac G2 Alpha Mixing Enable */

	unsigned int ge_ck2_apa;	/* 0x02b0 */
	unsigned int ge_amx_ctl;	/* 0x02b4 GE Alpha Mixing Control */
	unsigned int ge_ck_apa;		/* 0x02b8 GE Color Key alpha */
	unsigned int ge_fix_apa;	/* 0x02bc GE Fix Alpha */

	unsigned int g1_amx_hm;		/* 0x02c0 G1 AMX Hi Color Mode */
	unsigned int g2_amx_hm;		/* 0x02c4 G2 AMX Hi Color Mode */
	unsigned int ge_nh_data;	/* 0x02c8 G1/G2 No Hit Data Output */
	unsigned int ge_vsync_sts;	/* 0x02cc GE Vsync Status (New!) */

	unsigned int ge_reg_upd;	/* 0x02d0 GE Register Updata */
	unsigned int ge_reg_sel;	/* 0x02d4 GE Register Read Select */
	unsigned int ge_amx2_ctl;	/* 0x02d8 GE AMX Output Control */
	unsigned int ge_fix2_apa;	/* 0x02dc GE Fix Output Alpha */

	unsigned int g1_h_scale;	/* 0x02e0 G1 H Scaling Enable */
	unsigned int g2_h_scale;	/* 0x02e4 G2 H Scaling Enable */
	unsigned int g1_fbw;		/* 0x02e8 G1 Frame Buffer Width */
	unsigned int g1_vcrop;		/* 0x02ec G1 Vertical Cropping */

	unsigned int g1_hcrop;		/* 0x02f0 G1 Horizontal Cropping */
	unsigned int g2_fbw;		/* 0x02f4 G2 Frame Buffer Width */
	unsigned int g2_vcrop;		/* 0x02f8 G2 Vertical Cropping */
	unsigned int g2_hcrop;		/* 0x02fc G2 Horizontal Cropping */
};
