/*++ 
 * linux/drivers/video/wmt/vpp.c
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

#define VPP_C
// #define DEBUG
#include <common.h>
#include <malloc.h>
#include "vpp.h"
#include "../../board/wmt/include/wmt_clk.h"
#include "wmt_display.h"
// #include "../pmc/wmt_clk.h"

vpp_mod_base_t *vpp_mod_base_list[VPP_MOD_MAX];
extern int auto_pll_divisor(enum dev_id dev, enum clk_cmd cmd, int unit, int freq);

unsigned int vpp_get_chipid(void)
{
	// byte 3,2: chip id, byte 1:ver id, byte 0:sub id
	// ex: 0x34290101 (0x3429 A0), 0x34290102 (0x3429 A1)
	return REG32_VAL(0xd8120000);
}

void vpp_set_dbg_gpio(int no,int value)
{
	unsigned int mask;

	return;
	mask = 0x1 << no;
	REG32_VAL(0xd8110040) |= mask;
	REG32_VAL(0xd8110080) |= mask;
	if( value == 0xFF ){
		if( REG32_VAL(0xd81100C0) & mask ){
			REG32_VAL(0xd81100C0) &= ~mask;			
		}
		else {
			REG32_VAL(0xd81100C0) |= mask;
		}
	}
	else {
		if( value ){
			REG32_VAL(0xd81100C0) |= mask;
		}
		else {
			REG32_VAL(0xd81100C0) &= ~mask;
		}
	}
}

int vpp_check_dbg_level(vpp_dbg_level_t level)
{
	if( level == VPP_DBGLVL_ALL )
		return 1;

	switch( g_vpp.dbg_msg_level ){
		case VPP_DBGLVL_DISABLE:
			break;
		case VPP_DBGLVL_ALL:
			return 1;
		default:
			if( g_vpp.dbg_msg_level == level ){
				return 1;
			}
			break;
	}
	return 0;
}

void vpp_mod_unregister(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;

	if( mod >= VPP_MOD_MAX ){
		return;
	}

	if( !(mod_p = vpp_mod_base_list[mod]) )
		return;
	
	if( mod_p->fb_p ) kfree(mod_p->fb_p);
	kfree(mod_p);
	vpp_mod_base_list[mod] = 0;
}

vpp_mod_base_t *vpp_mod_register(vpp_mod_t mod,int size,unsigned int flags)
{
	vpp_mod_base_t *mod_p;

	if( mod >= VPP_MOD_MAX ){
		return 0;
	}

	if( vpp_mod_base_list[mod] ){
		vpp_mod_unregister(mod);
	}

	mod_p = (void *) kmalloc(size,GFP_KERNEL);
	if( !mod_p ) return 0;

	vpp_mod_base_list[mod] = mod_p;
	memset(mod_p,0,size);
	mod_p->mod = mod;

	if( flags & VPP_MOD_FLAG_FRAMEBUF ){
		if( !(mod_p->fb_p = (void *) kmalloc(sizeof(vpp_fb_base_t),GFP_KERNEL)) ){
			goto error;
		}
		memset(mod_p->fb_p,0,sizeof(vpp_fb_base_t));
	}
//	printf("vpp mod register %d,0x%x,0x%x\n",mod,(int)mod_p,(int)mod_p->fb_p);
	return mod_p;
	
error:
	vpp_mod_unregister(mod);
	printf("vpp mod register NG %d\n",mod);	
	return 0;
}

vpp_mod_base_t *vpp_mod_get_base(vpp_mod_t mod)
{
	if( mod >= VPP_MOD_MAX )
		return 0;

	return vpp_mod_base_list[mod];
}

vpp_fb_base_t *vpp_mod_get_fb_base(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;
	mod_p = vpp_mod_get_base(mod);
	if( mod_p )
		return mod_p->fb_p;

	return 0;
}

vdo_framebuf_t *vpp_mod_get_framebuf(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;

	mod_p = vpp_mod_get_base(mod);
	if( mod_p && mod_p->fb_p )
		return &mod_p->fb_p->fb;

	return 0;
}

vpp_display_format_t vpp_get_fb_field(vdo_framebuf_t *fb)
{
	if( fb->flag & VDO_FLAG_INTERLACE )
		return VPP_DISP_FMT_FIELD;
	
	return VPP_DISP_FMT_FRAME;
}

unsigned int vpp_get_base_clock(vpp_clk_pll_t pll,vpp_clk_div_t div)
{
	unsigned int clock = 0;

	switch(pll){
		case VPP_PLL_B:
			clock = auto_pll_divisor(DEV_DVO,CLK_ENABLE,0,0);
			break;
		case VPP_PLL_C:
			clock = auto_pll_divisor(DEV_VPP,CLK_ENABLE,0,0);
			break;
		default:
			break;
	}
	printf("[VPP] get base clock %s : %d\n",(pll==VPP_PLL_B)? "PLLB":"PLLC",clock);
	return clock;
}

unsigned int vpp_calculate_diff(unsigned int val1,unsigned int val2)
{
	return (val1 >= val2)?(val1-val2):(val2-val1);
}

void vpp_calculate_clock(vpp_base_clock_t *clk,unsigned int div_max,unsigned int base_mask)
{
	unsigned int pixclk;
	unsigned int sum1,sum2;
	int diff,diff_min;
	int base,mul,div;
	int base_bk,mul_bk,div_bk;

	diff_min = pixclk = clk->pixel_clock;
	base_bk = mul_bk = div_bk = 1;
	if( base_mask & 0x1 ){	/* 25MHz base */
		for(div=1;div<=div_max;div++){
			base = 6250000;
			mul = pixclk * div / base;
			sum1 = base * mul / div;
			sum2 = base * (mul + 1) / div;
			sum1 = vpp_calculate_diff(sum1,pixclk);
			sum2 = vpp_calculate_diff(sum2,pixclk);
			mul = (sum1 < sum2)? mul:(mul+1);
			sum1 = base * mul / div;
			mul /= 2;
			base *= 2;
			if( mul > 62 ){
				base *= 2;
				mul /= 2;
			}
			sum1 = base * mul;
			if( (sum1 < 300000000) || (sum1 > 750000000) ) continue;
			if( (mul % 2) || (mul < 8) || (mul > 62) ) continue;
			sum1 = sum1 / div;
			diff = vpp_calculate_diff(sum1,pixclk);
			if( diff < diff_min ){
				diff_min = diff;
				base_bk = base;
				mul_bk = mul;
				div_bk = div;
			}
		}
	}

	if( base_mask & 0x2 ){	/* 27MHz base */
		for(div=1;div<=div_max;div++){
			base = 6750000;
			mul = pixclk * div / base;
			sum1 = base * mul / div;
			sum2 = base * (mul + 1) / div;
			sum1 = vpp_calculate_diff(sum1,pixclk);
			sum2 = vpp_calculate_diff(sum2,pixclk);
			mul = (sum1 < sum2)? mul:(mul+1);

			sum1 = base * mul / div;
			mul /= 2;
			base *= 2;
			if( mul > 62 ){
				base *= 2;
				mul /= 2;
			}
			sum1 = base * mul;
			if( (sum1 < 300000000) || (sum1 > 750000000) ) continue;
			if( (mul % 2) || (mul < 8) || (mul > 62) ) continue;
			sum1 = sum1 / div;
			diff = vpp_calculate_diff(sum1,pixclk);
			if( diff < diff_min ){
				diff_min = diff;
				base_bk = base;
				mul_bk = mul;
				div_bk = div;
			}
		}
	}
	
//	DBGMSG("pixclk %d, base %d, mul %d, div %d,diff %d\n",pixclk,base_bk,mul_bk,div_bk,diff_min);
	clk->pixel_clock = base_bk * mul_bk / div_bk;
	clk->divisor = div_bk;
	clk->rd_cyc = 0;

	switch( base_bk ){
		case 12500000: 
			clk->PLL = 0x20; 
			break;
		default:
		case 25000000: 
			clk->PLL = 0x120; 
			break;
		case 13500000:
			clk->PLL = 0x00;
			break;
		case 27000000:
			clk->PLL = 0x100;
			break;
	}
	clk->PLL = clk->PLL | (mul_bk / 2);
	
}

unsigned int vpp_check_value(unsigned int val,unsigned int min,unsigned int max)
{
	if( min >= max ) 
		return min;
	val = (val < min)? min:val;
	val = (val > max)? max:val;
	return val;
}

void vpp_show_timing(vpp_timing_t *tmr,vpp_clock_t *clk)
{
	if( tmr ){
		printf("pixel clock %d,option 0x%x\n",tmr->pixel_clock,tmr->option);
		printf("H sync %d,bp %d,pixel %d,fp %d\n",tmr->hsync,tmr->hbp,tmr->hpixel,tmr->hfp);
		printf("V sync %d,bp %d,pixel %d,fp %d\n",tmr->vsync,tmr->vbp,tmr->vpixel,tmr->vfp);
	}
	
	if( clk ){
		printf("H beg %d,end %d,total %d\n",clk->begin_pixel_of_active,clk->end_pixel_of_active,clk->total_pixel_of_line);
		printf("V beg %d,end %d,total %d\n",clk->begin_line_of_active,clk->end_line_of_active,clk->total_line_of_frame);
		printf("Hsync %d, Vsync %d\n",clk->hsync,clk->vsync);
		printf("VBIE %d,PVBI %d\n",clk->line_number_between_VBIS_VBIE,clk->line_number_between_PVBI_VBIS);
	}
}

void vpp_trans_timing(vpp_mod_t mod,vpp_timing_t *tmr,vpp_clock_t *hw_tmr,int to_hw)
{
	vpp_fb_base_t *mod_fb_p;
	unsigned int pixel_clock;
	int temp;

	mod_fb_p = vpp_mod_get_fb_base(mod);
	if( to_hw ){
		hw_tmr->begin_pixel_of_active = tmr->hsync + tmr->hbp;
		hw_tmr->end_pixel_of_active = hw_tmr->begin_pixel_of_active + tmr->hpixel;
		hw_tmr->total_pixel_of_line = hw_tmr->end_pixel_of_active + tmr->hfp;	
		hw_tmr->begin_line_of_active = tmr->vsync + tmr->vbp + 1;
		hw_tmr->end_line_of_active = hw_tmr->begin_line_of_active + tmr->vpixel;
		hw_tmr->total_line_of_frame = hw_tmr->end_line_of_active + tmr->vfp -1;	
		hw_tmr->line_number_between_VBIS_VBIE = tmr->vsync + 1; // hw_tmr->begin_line_of_active - 3;
		temp = hw_tmr->total_line_of_frame - hw_tmr->end_line_of_active;
		hw_tmr->line_number_between_PVBI_VBIS = (temp>2)? (temp-1):1;
		hw_tmr->hsync = tmr->hsync;
		hw_tmr->vsync = tmr->vsync;
		
		// pixel_clock = hw_tmr->total_pixel_of_line * hw_tmr->total_line_of_frame * mod_fb_p->framerate;
		pixel_clock = tmr->pixel_clock;
		if( mod == VPP_MOD_GOVRH ){
			hw_tmr->read_cycle = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / pixel_clock;		
		}
		else {
			hw_tmr->read_cycle = vpp_get_base_clock(VPP_PLL_B,VPP_DIV_VPP) / pixel_clock;		
		}
	}
	else {
		pixel_clock = hw_tmr->total_pixel_of_line * hw_tmr->total_line_of_frame * mod_fb_p->framerate;
		tmr->pixel_clock = pixel_clock;
		tmr->option = 0;
		
		tmr->hsync = hw_tmr->hsync;
		tmr->hbp = hw_tmr->begin_pixel_of_active - hw_tmr->hsync;
		tmr->hpixel = hw_tmr->end_pixel_of_active - hw_tmr->begin_pixel_of_active;
		tmr->hfp = hw_tmr->total_pixel_of_line - hw_tmr->end_pixel_of_active;

		tmr->vsync = hw_tmr->vsync;
		tmr->vbp = hw_tmr->begin_line_of_active - hw_tmr->vsync -1;
		tmr->vpixel = hw_tmr->end_line_of_active - hw_tmr->begin_line_of_active;
		tmr->vfp = hw_tmr->total_line_of_frame - hw_tmr->end_line_of_active +1;
	}
}

void vpp_calculate_timing(vpp_mod_t mod,unsigned int fps,vpp_clock_t *tmr)
{
	unsigned int base_clock;
	unsigned int rcyc_max,rcyc_min;
	unsigned int h_min,h_max;
	unsigned int v_min,v_max;
	unsigned int diff_min,diff_h,diff_v,diff_rcyc;	
	unsigned int hbp_min,hfp_min,hporch_min;
	unsigned int vbp_min,vfp_min,vporch_min;	
	int i,j,k,temp;
	unsigned int hpixel,vpixel;
    int diff_clk;

	hpixel = tmr->end_pixel_of_active - tmr->begin_pixel_of_active;
	vpixel = tmr->end_line_of_active - tmr->begin_line_of_active;

	if( mod == VPP_MOD_GOVRH ){
		base_clock = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / fps;
	}
	else {
		base_clock = vpp_get_base_clock(VPP_PLL_B,VPP_DIV_VPP) / fps;
	}
	hbp_min = tmr->begin_pixel_of_active;
	hfp_min = tmr->total_pixel_of_line - tmr->end_pixel_of_active;
	hporch_min = hbp_min + hfp_min;
	vbp_min = tmr->begin_line_of_active;
	vfp_min = tmr->total_line_of_frame - tmr->end_line_of_active;
	vporch_min = vbp_min + vfp_min;

	rcyc_min = vpp_check_value((base_clock / (4096 * 4096)),WMT_VPU_RCYC_MIN+1,256);
	rcyc_max = vpp_check_value((base_clock / (hpixel * vpixel)) + 1,WMT_VPU_RCYC_MIN+1,256);

	if( g_vpp.govw_hfp && g_vpp.govw_hbp ){
		h_min = g_vpp.govw_hfp + g_vpp.govw_hbp + hpixel;
		h_max = g_vpp.govw_hfp + g_vpp.govw_hbp + hpixel;

		vbp_min = 4;
		vporch_min = 6;
		rcyc_min = rcyc_max = 3;
	}
	else {
		h_min = vpp_check_value((base_clock / (rcyc_max * 4096)),hpixel+hporch_min,4096);
		h_max = vpp_check_value((base_clock / (rcyc_min * vpixel)) + 1,hpixel+hporch_min,4096);
	}

	if( g_vpp.govw_vfp && g_vpp.govw_vbp ){
		v_min = g_vpp.govw_vfp + g_vpp.govw_vbp + vpixel;
		v_max = g_vpp.govw_vfp + g_vpp.govw_vbp + vpixel;
	}
	else {
		v_min = vpp_check_value((base_clock / (rcyc_max * 4096)),vpixel+vporch_min,4096);
		v_max = vpp_check_value((base_clock / (rcyc_min * hpixel)) + 1,vpixel+vporch_min,4096);
	}

//	printf("[VPP] clk %d,rcyc(%d,%d),h(%d,%d),v(%d,%d)\n",base_clock,rcyc_min,rcyc_max,h_min,h_max,v_min,v_max);

	diff_min=0xFFFFFFFF;
	diff_rcyc = diff_h = diff_v = 0;
	for(i=rcyc_max;i>=rcyc_min;i--){
		for(j=v_max;j>=v_min;j--){
			temp = (base_clock * 100) / (i*j);
			k = temp / 100;
			k += ((temp % 100) >= 50)? 1:0;
			k = vpp_check_value(k,h_min,h_max);
			temp = i * j * k;
			diff_clk = vpp_calculate_diff(base_clock,temp);
			if(diff_min > diff_clk){
				diff_min = diff_clk;
				diff_h = k;
				diff_v = j;
				diff_rcyc = i;
//				printf("[VPP] rcyc %d h %d v %d\n",diff_rcyc,diff_h,diff_v);
			}
			if( diff_clk == 0 ){
				i = rcyc_min;
				break;
			}
		}
	}

	tmr->read_cycle = diff_rcyc - 1;
	tmr->total_pixel_of_line = diff_h;
	if( g_vpp.govw_hfp && g_vpp.govw_hbp ){
		tmr->begin_pixel_of_active = g_vpp.govw_hbp;
		tmr->end_pixel_of_active = tmr->begin_pixel_of_active + hpixel;
	}
	else {
#if 1
		temp = diff_h - hpixel;
		tmr->begin_pixel_of_active = ( temp/10 );
		tmr->begin_pixel_of_active = vpp_check_value(tmr->begin_pixel_of_active,20,504);
#else
		temp = diff_h - hpixel - hbp_min;
		tmr->begin_pixel_of_active = ( hbp_min + temp/2 );
		tmr->begin_pixel_of_active = vpp_check_value(tmr->begin_pixel_of_active,hbp_min,504);		
#endif		
		tmr->end_pixel_of_active = tmr->begin_pixel_of_active + hpixel;
	}

	tmr->total_line_of_frame = diff_v;
	if( g_vpp.govw_vfp && g_vpp.govw_vbp ){
		tmr->begin_line_of_active = g_vpp.govw_vbp;
		tmr->end_line_of_active = tmr->begin_line_of_active + vpixel;
	}
	else {
		temp = diff_v - vpixel - vbp_min;
		tmr->begin_line_of_active = ( vbp_min + temp/2 );
		tmr->begin_line_of_active = vpp_check_value(tmr->begin_line_of_active,vbp_min,504);
		tmr->end_line_of_active = tmr->begin_line_of_active + vpixel;
	}

	tmr->line_number_between_VBIS_VBIE = tmr->begin_line_of_active - 3;

#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
	g_vpp.govw_tg_rcyc = tmr->read_cycle;
	g_vpp.govw_tg_rtn_cnt = 0;
	g_vpp.govw_tg_rtn_max = fps;
#endif	
}

vpp_csc_t vpp_check_csc_mode(vpp_csc_t mode,vdo_color_fmt src_fmt,vdo_color_fmt dst_fmt,unsigned int flags)
{
	if( mode >= VPP_CSC_MAX ) 
		return VPP_CSC_BYPASS;

	mode = (mode > VPP_CSC_RGB2YUV_MIN)? (mode - VPP_CSC_RGB2YUV_MIN):mode;
	if( src_fmt >= VDO_COL_FMT_ARGB ){
		mode = VPP_CSC_RGB2YUV_MIN + mode;
	}
	else {
		src_fmt = VDO_COL_FMT_YUV444;
	}
	dst_fmt = (dst_fmt >= VDO_COL_FMT_ARGB)? VDO_COL_FMT_ARGB:VDO_COL_FMT_YUV444;
	if( flags == 0 ){
		mode = ( src_fmt != dst_fmt )? mode:VPP_CSC_BYPASS;
	}
	return mode;
}

void vpp_set_vout_resolution(int resx,int resy,int fps)
{
	p_govw->fb_p->fb.img_w = resx;
	p_govw->fb_p->fb.img_h = resy;
	p_govw->fb_p->set_framebuf(&p_govw->fb_p->fb);

#ifdef WMT_FTBLK_GOVRH
	p_govrh->fb_p->fb.img_w = resx;
	p_govrh->fb_p->fb.img_h = resy;
	p_govrh->fb_p->framerate = fps;
	p_govrh->fb_p->set_framebuf(&p_govrh->fb_p->fb);
#endif	
}

int vpp_get_gcd(int A, int B)
{
	while(A != B){
		if( A > B ){
			A = A - B;
		}
		else {
			B = B - A;
		}
	}
	return A;
}

void vpp_check_scale_ratio(int *src,int *dst,int max,int min)
{
	int val1,val2;

	if( *dst >= *src ){	// scale up
		if( (*dst) > ((*src)*max) ){
			printf("*W* scale up over spec (max %d)\n",max);
			*dst = (*src) * max;
		}
	}
	else {	// scale down
		int p,q,diff;
		int diff_min,p_min,q_min;

		val1 = val2 = (*dst) * 1000000 / (*src);
		diff_min = val1;
		p_min = 1;
		q_min = min;
		for(q=2;q<=min;q++){
			for(p=1;p<q;p++){
				val2 = p * 1000000 / q;
				if( val1 < val2 ){
					break;
				}
				diff = vpp_calculate_diff(val1,val2);
				if( diff < diff_min ){
					diff_min = diff;
					p_min = p;
					q_min = q;
				}
			}
			if( val1 == val2 )
				break;
		}
		val1 = (*src) / q_min;
		val2 = (*dst) / p_min;
		val1 = (val1 < val2)? val1:val2;
		*src = val1 * q_min;
		*dst = val1 * p_min;
	}
}	

void vpp_calculate_scale_ratio(int *src,int *dst,int max,int min,int align_s,int align_d)
{
	int i;

	if( *dst >= *src ){	// scale up and bypass
		if( (*dst) > ((*src)*max) ){
			printf("*W* scale up over spec (max %d)\n",max);
			*dst = (*src) * max;
		}

		*src -= (*src % align_s);
		*dst -= (*dst % align_d);
	}
	else {	// scale down
		int val1,val2;

		for(i=0;;i++){
			val1 = *src + i;
			val2 = *dst - (*dst % align_d);
			vpp_check_scale_ratio(&val1,&val2,max,min);
			if( val1 < *src ) continue;		// don't drop
			if( val1 % align_s ) continue;	// src img_w align
			if( val2 % align_d ) continue;	// dst img_w align
			if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
				int temp_s,temp_d;
				int diff,diff2;
				
				temp_s = (val1 > *src)? (*src * 100):(val1 * 100);
				diff = vpp_calculate_diff(*src*100,temp_s);

				temp_d = temp_s * val2 / val1;
				diff2 = vpp_calculate_diff(*dst*100,temp_d);
				printf("[VPP] %d:%d->%d,%d->%d,%d->%d,s diff %d,d diff %d\n",i,*src,*dst,val1,val2,temp_s,temp_d,diff,diff2);
			}
			break;
		};
		*src = val1;
		*dst = val2;
	}
}

unsigned int vpp_get_video_mode_fps(vpp_timing_t *timing)
{
	unsigned int temp;
	unsigned int resx,resy;
	unsigned int diff1,diff2;

	temp = VPP_GET_OPT_FPS(timing->option);
	if( temp )
		return temp;

	resx = timing->hsync + timing->hbp + timing->hpixel + timing->hfp;
	resy = timing->vsync + timing->vbp + timing->vpixel + timing->vfp;
	temp = timing->pixel_clock - (timing->pixel_clock % 1000);
	temp = temp / resx;
	temp = temp / resy;
	diff1 = vpp_calculate_diff(timing->pixel_clock,(resx*resy*temp));
	diff2 = vpp_calculate_diff(timing->pixel_clock,(resx*resy*(temp+1)));
	temp = (diff1 < diff2)? temp:(temp+1);
	return temp;
}

vpp_timing_t *vpp_get_video_mode(unsigned int resx,unsigned int resy,unsigned int fps_pixclk)
{
	int is_fps;
	int i,j;
	vpp_timing_t *ptr;
	unsigned int line_pixel;
	int index;

	if (g_display_vaild&TMRFROMENV)
		return &g_display_tmr;
	is_fps = ( fps_pixclk >= 1000000 )? 0:1;
	for(i=0,index=0;;i++){
		ptr = (vpp_timing_t *) &vpp_video_mode_table[i];
		if( ptr->pixel_clock == 0 ){
			break;
		}
		line_pixel = (ptr->option & VPP_OPT_INTERLACE)? (ptr->vpixel*2):ptr->vpixel;
		if ((ptr->hpixel == resx) && (line_pixel == resy)) {
			for(j=i,index=i;;j++){
				ptr = (vpp_timing_t *) &vpp_video_mode_table[j];
				if( ptr->pixel_clock == 0 ){
					break;
				}
				if( ptr->hpixel != resx ){
					break;
				}
				if( is_fps ){
					if( fps_pixclk == vpp_get_video_mode_fps(ptr) ){
						index = j;
						break;
					}
				}
				else {
					if( (fps_pixclk/1000) == (ptr->pixel_clock/1000) ){
						index = j;
					}
					if( (fps_pixclk) == (ptr->pixel_clock) ){
						index = j;
						goto get_mode_end;
					}
				}
			}
			break;
		}
		if( ptr->hpixel > resx ){
			break;
		}
		index = i;
		if( ptr->option & VPP_OPT_INTERLACE ){
			i++;
		}
	}
get_mode_end:	
	ptr = (vpp_timing_t *) &vpp_video_mode_table[index];
	// printk("[VPP] get video mode %dx%d@%d,index %d\n",resx,resy,fps_pixclk,index);	
	return ptr;	
}

void vpp_set_video_mode(unsigned int resx,unsigned int resy,unsigned int pixel_clock)
{
	vpp_timing_t *timing;

	timing = vpp_get_video_mode(resx,resy,pixel_clock);
	govrh_set_timing(timing);
}


unsigned int vpp_calculate_y_crop(unsigned int hcrop,unsigned int vcrop,unsigned int fbw,vdo_color_fmt colfmt)
{
	unsigned int offset;

	offset = vcrop * fbw + hcrop;
	if( colfmt == VDO_COL_FMT_ARGB ){
		offset *= 4;
	}
	return offset;
}

unsigned int vpp_calculate_c_crop(unsigned int hcrop,unsigned int vcrop,unsigned int fbw,vdo_color_fmt colfmt)
{
	unsigned int offset;
	unsigned int stribe;
	unsigned int c_line,c_pixel;

	if( colfmt == VDO_COL_FMT_ARGB )
		return 0;

	switch( colfmt ){
		case VDO_COL_FMT_YUV420:
			c_pixel = 2;
			c_line = 2;
			break;
		case VDO_COL_FMT_YUV422H:
			c_pixel = 2;
			c_line = 1;
			break;
		default:
		case VDO_COL_FMT_YUV444:
			c_pixel = 1;
			c_line = 1;
			break;
	}
	stribe = fbw * 2 / c_pixel;
	offset = (vcrop / c_line) * stribe + (hcrop / c_pixel) * 2;
	return offset;
}

void vpp_mod_init(void)
{
	//printf("[VPP] vpp_mod_init\n");
	vppm_mod_init();

#ifdef WMT_FTBLK_GOVRH	
	govrh_mod_init();
#endif


#ifdef WMT_FTBLK_LCDC
	lcdc_mod_init();
#endif

#ifdef WMT_FTBLK_GOVW
	govw_mod_init();
#endif
/*
#ifdef WMT_FTBLK_SCL
	scl_mod_init();
#endif
*/
#ifdef WMT_FTBLK_GOVM
	govm_mod_init();
#endif
/*
#ifdef WMT_FTBLK_VPU
	vpu_mod_init();
#endif
*/

#ifdef WMT_FTBLK_DISP	
	disp_mod_init();
#endif

}

#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
/*!*************************************************************************
* vpp_govw_dynamic_tg_set_rcyc()
* 
* Private Function by Sam Shen, 2009/11/06
*/
/*!
* \brief	set govw tg
*		
* \retval  None
*/ 
void vpp_govw_dynamic_tg_set_rcyc(int rcyc)
{
	rcyc = (rcyc > 0xFF)? 0xFF:rcyc;
	vppif_reg32_write(GOVW_TG_RDCYC,rcyc);
}

/*!*************************************************************************
* vpp_govw_dynamic_tg()
* 
* Private Function by Sam Shen, 2009/10/14
*/
/*!
* \brief	check govw tg error and recover status
*		
* \retval  None
*/ 
void vpp_govw_dynamic_tg(int err)
{
	int rcyc;
	int diff;	

	if( g_vpp.govw_tg_dynamic == 0 )
		return;

	if( err ){
		g_vpp.govw_tg_rtn_cnt = 0;
		rcyc = vppif_reg32_read(GOVW_TG_RDCYC);
		rcyc = (rcyc >= 0xFF)? 255:(rcyc+1);
//		vppif_reg32_write(GOVW_TG_ENABLE,0x0);
		vpp_govw_dynamic_tg_set_rcyc(rcyc);
//		vppif_reg32_write(GOVW_TG_ENABLE,0x1);
		if( vpp_check_dbg_level(VPP_DBGLVL_TG) ){
			printf("[VPP] adjust GOVW rcyc %d\n",rcyc);
		}
	}
	else {
		g_vpp.govw_tg_rtn_cnt++;
		if( g_vpp.govw_tg_rtn_cnt > g_vpp.govw_tg_rtn_max){
			g_vpp.govw_tg_rtn_cnt = 0;
			rcyc = vppif_reg32_read(GOVW_TG_RDCYC);
			if (rcyc > g_vpp.govw_tg_rcyc){
				diff = rcyc - g_vpp.govw_tg_rcyc + 1;
				rcyc -= (diff/2);
//				vppif_reg32_write(GOVW_TG_ENABLE,0x0);
				vpp_govw_dynamic_tg_set_rcyc(rcyc);
//				vppif_reg32_write(GOVW_TG_ENABLE,0x1);							
				if( vpp_check_dbg_level(VPP_DBGLVL_TG) ){
					printf("[VPP] return GOVW rcyc %d\n",rcyc);
				}
			}
		}
	}
} /* End of vpp_govw_dynamic_tg */

/*!*************************************************************************
* vpp_set_vppm_int_enable()
* 
* Private Function by Sam Shen, 2010/10/07
*/
/*!
* \brief	set vppm interrupt enable
*		
* \retval  None
*/ 
void vpp_set_vppm_int_enable(vpp_int_t int_bit,int enable)
{
	if( int_bit & VPP_INT_GOVRH_VBIS ){
		int int_enable;

		int_enable = enable;
		if( vppif_reg32_read(GOVRH_CUR_ENABLE) )	// govrh hw cursor
			int_enable = 1;

		if( g_vpp.direct_path )	// direct path
			int_enable = 1;

		if( g_vpp.vga_enable )	// vga plug detect
			int_enable = 1;

		vppm_set_int_enable(int_enable,VPP_INT_GOVRH_VBIS);
		int_bit &= ~VPP_INT_GOVRH_VBIS;
	}
	
	if( int_bit ){
		vppm_set_int_enable(enable,int_bit);
	}
} /* End of vpp_set_vppm_int_enable */

#endif
