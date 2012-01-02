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
4F, 531, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

#include <config.h>
#include <common.h>
#include <command.h>
#include <version.h>
#include <stdarg.h>
#include <linux/types.h>
#include <devices.h>
#include <linux/stddef.h>
#include <asm/byteorder.h>

#include "vpp.h"
#include "vout.h"
#include "ge_regs.h"
#include "minivgui.h"

#define WMT_DISPLAY

#include "wmt_display.h"
#include "vout.h"

struct wmt_display_param_t g_display_param;
struct wmt_pwm_setting_t g_pwm_setting;
vpp_timing_t g_display_tmr;
int g_display_vaild;
unsigned int g_fb_phy;
unsigned int g_img_phy;
int g_logo_x;
int g_logo_y;

extern vout_mode_t dvo_vout_mode;
extern vout_mode_t int_vout_mode;
int vpp_dac_sense_enable;

extern vout_t *vo_parm_table[VOUT_MODE_MAX];
extern vout_t vo_boot_parm;

struct fb_var_screeninfo vfb_var = {
	.xres           = 800,
	.yres           = 480,
	.xres_virtual   = 800,
	.yres_virtual   = 480,
	.bits_per_pixel = 16,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
	.transp         = {0, 0, 0},
	//.activate       = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE,
	.height         = -1,
	.width          = -1,
	.pixclock       = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
	.left_margin    = 40,
	.right_margin   = 24,
	.upper_margin   = 32,
	.lower_margin   = 11,
	.hsync_len      = 96,
	.vsync_len      = 2,
	//.vmode          = FB_VMODE_NONINTERLACED
};

char *video_str[] = {
	"SDA", "SDD" , "LCD", "DVI" , "HDMI",
	"DVO2HDMI", "LVDS", "VGA"
};


int lcd_poweron_ctl(char *name)
{
	char *p;
    char * endp;
    int i = 0;
    ulong   addr;
    ulong   val, org;
    char    op;


	p = getenv(name);
    if (!p) {
        printf("OEM LCD power-on control is not set\n");
        return 0;
    }

    while(1)
    {
        addr = simple_strtoul(p, &endp, 16);
        if( *endp == '\0')
            break;

        op = *endp;
        if ((op == ',') || ((op == ';'))) {
        	val = 0;
        	org = 1;
        	for (i = 0; i < 8; i++) {
        		val += ((addr&0x0f)*org);
        		addr >>= 4;
				org *= 10;
        	}
        	//printf("delay %d usec .....\n", val);
        	udelay(val);
        	goto nextcheck;
        }
        if( endp[1] == '~') {
            val = simple_strtoul(endp+2, &endp, 16);
            val = ~val;
        } else
            val = simple_strtoul(endp+1, &endp, 16);
		
		if (addr&0x03) {
        	printf(" address not alginment to 32bit , address = 0x%x\n",addr);
        	goto nextcheck;
    	}		
        //printf("  reg op: 0x%X %c 0x%X\n", addr, op, val);
		org = REG32_VAL(addr);
        switch(op)
        {
            case '|': org |= val; break;
            case '=': org = val ; break;
            case '&': org &= val; break;
            default:
                printf("Error, Unknown operator %c\n", op);
                break;
        }
        REG32_VAL(addr) = org;
nextcheck:
        if(*endp == '\0')
            break;
        p = endp + 1;
    }


    return 0;
}

vout_mode_t vout_priority[] = { VOUT_BOOT, VOUT_LCD, VOUT_LVDS, VOUT_HDMI, VOUT_DVO2HDMI, VOUT_DVI, VOUT_SD_DIGITAL, VOUT_VGA, VOUT_SD_ANALOG, VOUT_MODE_MAX };
extern vout_t *vout_array[VOUT_MODE_MAX];

int vout_init(struct fb_var_screeninfo *var)
{
	vout_t *vo = 0;
	int sense_enable;
	int match = 0, i;
	unsigned int user_resx,user_resy,user_freq;
	
	printf("vo_init_wm8440\n");

	sense_enable = 0;
	user_resx = 0;
	/* default parameters */
#ifdef CONFIG_WMT_EDID
	memset(vout_edid,0,256);
#endif
	g_vpp.hdmi_audio_interface = 1;		// 0-I2S, 1-SPDIF
	g_vpp.hdmi_cp_enable = 1;
	g_vpp.govw_vfp = 5;
	g_vpp.govw_vbp = 5;

	/* check video out device */
	dvo_vout_mode = vout_extdev_probe();
	int_vout_mode = vout_intdev_probe();
    vout_register(VOUT_BOOT,&vo_boot_parm);

	vpp_dac_sense_enable = sense_enable;
	for (i = 0; i < 100; i++){
		if (vout_priority[i] == VOUT_MODE_MAX)
			break;
		if( vout_get_info(vout_priority[i]) ){
			if( vout_chkplug(vout_priority[i]) ){
				vo = vout_get_info(vout_priority[i]);
				if( user_resx == 0 ){
					user_resx = vo->resx;
					user_resy = vo->resy;
					user_freq = VPP_HD_DISP_FPS;
				}
				if( match ){
					var->xres = user_resx;
					var->yres = user_resy;
					var->pixclock = user_freq;
				}	else { 	// use default
					var->xres = vo->resx;
					var->yres = vo->resy;
					var->pixclock = vo->pixclk;
				}
				break;
			}
		}
	}

	if ( g_vpp.govrh_preinit ){
			vout_set_mode(VOUT_BOOT,1);
	} else {
		if ((g_display_param.op1 == LCD_WMT_OEM)&&(g_display_param.vout == VPP_VOUT_LCD))
			lcd_poweron_ctl(ENV_LCD_POWERON);
		if( (dvo_vout_mode != VOUT_MODE_MAX) && (dvo_vout_mode !=vout_priority[i]) ) vout_set_mode(dvo_vout_mode,1);
		if( (int_vout_mode != VOUT_MODE_MAX) && (int_vout_mode !=vout_priority[i]) ) vout_set_mode(int_vout_mode,1);
		if( vout_priority[i] < VOUT_BOOT ) vout_set_mode(vout_priority[i],1);
	}

	printf("vo_init_wm8440 (%s %dx%d,%d)\n",(vo)? vo->name:"NO",var->xres,var->yres,var->pixclock);
	return 0;
}


static int wmt_lcd_check(void)
{
	// op1 : lcd id , op2 : bpp
	if (g_display_param.op1 >= LCD_PANEL_MAX) {
		printf("error : LCD ID is %d , only support 0 ~ %d\n",g_display_param.op1,LCD_PANEL_MAX-1);
		goto lcdfailed;
	}

	switch( g_display_param.op2 ) {
		case 15:
		case 16:
		case 18:
		case 24:
			break;		
		default:
			printf("Warning  : unknow bpp %d , set to default 24bpp\n",g_display_param.op2);
			g_display_param.op2 = 24;
			break;
	}

	// if lcd id == oem ,get tmr from env
	if (g_display_param.op1 == LCD_WMT_OEM) {
		if ((g_display_vaild&TMRFROMENV) == 0) {
			if (parse_display_tmr(ENV_DISPLAY_TMR))
				goto lcdfailed;
		}
		vfb_var.xres = g_display_tmr.hpixel;
		vfb_var.yres = g_display_tmr.vpixel;
		vfb_var.xres_virtual = g_display_tmr.hpixel;
		vfb_var.yres_virtual = g_display_tmr.vpixel;
		vfb_var.pixclock = g_display_tmr.pixel_clock;
	} else {
		if ((g_display_param.resx == 0) || (g_display_param.resy == 0) || (g_display_param.fps_pixclk == 0))
			goto lcdfailed;
	}
	// get try pwm setting from env , run default if not set
	parse_pwm_params(ENV_DISPLAY_PWM, NULL);

	return 0;
lcdfailed:
	g_display_vaild = 0;
	return -1;	
}

static int wmt_init_check(void)
{
	char *tmp;
	unsigned int memtotal = 0;
	unsigned int memmax = 512; //should get from MC4
	unsigned int tmp1 = 0;

	// we must check "memtotal" because we get framebuffer from it
	if ((tmp = getenv (ENV_MEMTOTAL)) != NULL) {
		memtotal = (unsigned int)simple_strtoul (tmp, NULL, 10);
	} else {
		printf("error : we need %s to calculate framebuffer address\n", ENV_MEMTOTAL);
		return -1;
	}

	if (memtotal > 256) {         /* 512M */
		memtotal = 512;
	} else if (memtotal > 128) {  /* 256M */
		memtotal = 256;
	} else if (memtotal > 64) {   /* 128M */
		memtotal = 128;
	} else if (memtotal > 32) {   /* 64M */
		memtotal = 64;
	} else if (memtotal > 16) {   /* 32M */
		memtotal = 32;
	} else {
		memtotal = 0;
	}

	if ((memtotal == 0) || (memtotal > memmax)) {
		printf("error : memtotal = %dMByte , can not calculate framebuffer address\n", memtotal);
		return -1;
	}
	tmp1 = *(unsigned int *)0xd8120000;
	tmp1 >>= 16;
	//fan , only support 3429 for now
	switch(tmp1) {
	case 0x3429:
	case 0x3465:
		g_fb_phy = memtotal-8;
		break;
	default: 
		printf("error : unknow chip %x\n", tmp1);
		return -1;
	}	
	g_fb_phy <<= 20;

	//find "wmt.display.param" to get vout informat
	if (parse_display_params(ENV_DISPLAY_PARM))
		goto checkfailed;

	// get tmr form env if resx = 0 || resy == 0 || fps == 0
	if ((g_display_param.resx == 0) || (g_display_param.resy == 0) || (g_display_param.fps_pixclk == 0)) {
		if (parse_display_tmr(ENV_DISPLAY_TMR))
			goto checkfailed;
		vfb_var.xres = g_display_tmr.hpixel;
		vfb_var.yres = g_display_tmr.vpixel;
		vfb_var.xres_virtual = g_display_tmr.hpixel;
		vfb_var.yres_virtual = g_display_tmr.vpixel;
		vfb_var.pixclock = g_display_tmr.pixel_clock;
	} else {
		vfb_var.xres = g_display_param.resx;
		vfb_var.yres = g_display_param.resy;
		vfb_var.xres_virtual = g_display_param.resx;
		vfb_var.yres_virtual = g_display_param.resy;
		if (g_display_param.fps_pixclk < 1000)
			vfb_var.pixclock = vfb_var.xres*vfb_var.yres*g_display_param.fps_pixclk;
		else
			vfb_var.pixclock = g_display_param.fps_pixclk;
	}

	switch(g_display_param.vout) {
	case VPP_VOUT_SDA:
	case VPP_VOUT_SDD:
		printf("not support vout format : %s\n",video_str[g_display_param.vout]);
		goto checkfailed;
	case VPP_VOUT_LCD:
		if (wmt_lcd_check())
			goto checkfailed;
		break;
	case VPP_VOUT_DVI:
		break;
	case VPP_VOUT_HDMI:
	case VPP_VOUT_DVO2HDMI:
	case VPP_VOUT_LVDS:
	case VPP_VOUT_VGA:
		printf("not support vout format : %s\n",video_str[g_display_param.vout]);
		goto checkfailed;
	default:
		printf("error : Unknow vout format ....\n");
		goto checkfailed;
	}

	if ((tmp = getenv (ENV_IMGADDR)) != NULL) {
		g_img_phy = (unsigned int)simple_strtoul (tmp, NULL, 16);
	} else {
		printf("%s is not found , command (display show) is invaild\n", ENV_IMGADDR);
		g_img_phy = 0xFFFF;
	}

	return 0;

checkfailed :
	g_display_vaild = 0;
	return -1;	
}

int vpp_alloc_framebuffer(unsigned int resx,unsigned int resy)
{
	vdo_framebuf_t *fb;
	unsigned int y_size, fb_size;
/*
	if( g_vpp.mb[0] ){
		printf("vpp_alloc_framebuffer : alreay alloc \n");
		return 0;
	}
*/
	if( resx % 64 ){
		resx += (64 - (resx % 64));
	}

	y_size = resx * resy;
	fb_size = y_size*(vfb_var.bits_per_pixel>>3);

	g_vpp.mb[0] = g_fb_phy+(fb_size*2);
	g_vpp.mb[1] = g_vpp.mb[0];

	fb = &p_govw->fb_p->fb;
	fb->y_addr = g_vpp.mb[0];
	fb->c_addr = g_vpp.mb[0] + y_size;

	fb->y_size = y_size;
	fb->c_size = y_size * 3;
	fb->fb_w = resx;
	fb->fb_h = resy;

	fb = &g_vpp.govr->fb_p->fb;	
	fb->y_addr = g_vpp.mb[1];
	fb->c_addr = g_vpp.mb[1] + y_size;

	fb->y_size = y_size;
	fb->c_size = y_size * 3;
	fb->fb_w = resx;
	fb->fb_h = resy;

	return 0;
}

int vpp_config(struct fb_var_screeninfo *info)
{
	vout_info_t vo_info;
	//vdo_framebuf_t *fb;

	g_vpp.resx = info->xres;
	g_vpp.resy = info->yres;
	vo_info.resx = info->xres;
	vo_info.resy = info->yres;
	vo_info.timing.pixel_clock = info->pixclock;
	vo_info.bpp = info->bits_per_pixel;
	vo_info.fps = info->pixclock / (info->xres * info->yres);
	if( info->pixclock != (info->xres * info->yres * vo_info.fps ) ){
		vo_info.fps = VPP_VOUT_FRAMERATE_DEFAULT;
	}
	if( vo_info.fps == 0 ){
		vo_info.fps = VPP_VOUT_FRAMERATE_DEFAULT;
	}
	printf("vpp_config(%dx%d@%d),pixclock %d\n",vo_info.resx,vo_info.resy,vo_info.fps,info->pixclock);
	vout_config((g_vpp.govrh_preinit)? VOUT_BOOT:VOUT_MODE_ALL,&vo_info);
	if( (vo_info.resx != info->xres) || (vo_info.resy != info->yres) ){
		printf("vout mode update (%dx%d)\n",vo_info.resx,vo_info.resy);
		info->xres = vo_info.resx;
		info->yres = vo_info.resy;
	}
	vpp_alloc_framebuffer(vo_info.resx,vo_info.resy);

	g_vpp.govr->fb_p->fb.img_w = info->xres;
	g_vpp.govr->fb_p->fb.img_h = info->yres;
	g_vpp.govr->fb_p->framerate = vo_info.fps;

	if ( g_vpp.govrh_preinit ){
		g_vpp.govr->fb_p->fb.fb_w = info->xres;
		g_vpp.govr->fb_p->fb.fb_h = info->yres;
		p_govw->fb_p->fb.fb_w = info->xres;
		p_govw->fb_p->fb.fb_h = info->yres;
	}
	else {
		g_vpp.govr->fb_p->set_framebuf(&g_vpp.govr->fb_p->fb);
	}

#ifdef WMT_FTBLK_GOVRH	
	p_govrh->vga_dac_sense_cnt = vo_info.fps * VPP_DAC_SENSE_SECOND;
#endif	
	p_govw->fb_p->fb.img_w = info->xres;
	p_govw->fb_p->fb.img_h = info->yres;
	p_govw->fb_p->set_framebuf(&p_govw->fb_p->fb);

	if( g_vpp.direct_path ){
		govw_set_tg_enable(VPP_FLAG_DISABLE);
		g_vpp.direct_path_colfmt = g_vpp.govr->fb_p->fb.col_fmt;
	}

#ifdef WMT_FTBLK_GOVRH	
	govrh_set_tg_enable(VPP_FLAG_ENABLE);
#endif

	if( g_vpp.vo_enable ){
		//vpp_wait_vsync();
		//vpp_wait_vsync();
		vout_set_blank(VOUT_MODE_ALL,0);
		g_vpp.vo_enable = 0;
	}

	return 0;
} /* End of vpp_config */

static ge_info_t *geinfo;

void ge_wait_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;
	regs->ge_int_en = 0; /* disable interrupt */

	while (regs->ge_status & BIT2);
	regs->ge_int_flag |= ~0;
}

int ge_fillrect2(ge_surface_t *d, unsigned int color)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int offset;
	unsigned int r, g, b, a;
	ge_surface_t s;

	if ((d->xres == 0) || (d->yres == 0)) {
		printf("error : rect width = %d , height = %d\n", d->xres, d->yres);
		return -1;
	}

	regs = geinfo->mmio;

	memcpy(&s, d, sizeof(ge_surface_t));

	offset = (d->y * vfb_var.xres + d->x);
	offset *= vfb_var.bits_per_pixel >> 3;


	s.addr = g_fb_phy+offset;
	s.xres = vfb_var.xres;
	s.yres = vfb_var.yres;
	s.xres_virtual = vfb_var.xres_virtual;
	s.yres_virtual = vfb_var.yres;
	s.x = 0;
	s.y = 0;

	switch (vfb_var.bits_per_pixel) {
	case 8:
		d->pixelformat = GEPF_LUT8;
		break;
	case 16:
		d->pixelformat = GEPF_RGB16;
		break;
	case 32:
		d->pixelformat = GEPF_RGB32;
		break;
	default:
		d->pixelformat = GEPF_RGB16;
		break;
	}
	r = (color>>16)&0xFF;
	g = (color>>8)&0xFF;
	b = (color&0xFF);
	a = (color>>24)&0xFF;

	ge_wait_sync(geinfo); //fan need to check
	ge_set_color(geinfo, r, g, b, a, d->pixelformat);
	ge_set_pixelformat(geinfo, d->pixelformat);
	ge_set_destination(geinfo, &s);
	ge_set_command(geinfo, GECMD_BLIT, 0xf0);
	ge_wait_sync(geinfo);

	return 0;
}

int ge_init(struct fb_var_screeninfo *info)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int offset = 0;
	ge_surface_t s;

	ge_sys_init(&geinfo, 0);

	regs = geinfo->mmio;

	/* 0x00000 (fastest) - 0x30003 (slowest) */
	regs->ge_delay = 0x10001;


	REG_SET32(0xd8050308, 0x00000001); /* Turn on GE by GOV */ //fan

	amx_set_csc(geinfo, AMX_CSC_JFIF_0_255);
	amx_set_alpha(geinfo, 0, 0xff);

	s.addr = g_fb_phy;
	s.yres = info->yres;
	s.yres_virtual = info->yres;
	s.x = 0;
	s.y = 0;

	// GE width & height has only 11bits
	if ((info->xres<<1) >= 2048) {
		s.xres = info->xres;
		s.xres_virtual = info->xres_virtual;
		offset = (info->xres * info->yres);
		offset *= info->bits_per_pixel >> 3;
	} else {
		s.xres = info->xres*2;
		s.xres_virtual = info->xres_virtual*2;
	}

	switch (info->bits_per_pixel) {
	case 8:
		s.pixelformat = GEPF_LUT8;
		break;
	case 16:
		if ((info->red.length == 5) &&
			(info->green.length == 6) &&
			(info->blue.length == 5)) {
			s.pixelformat = GEPF_RGB16;
		} else if ((info->red.length == 5) &&
			(info->green.length == 5) &&
			(info->blue.length == 5)) {
			s.pixelformat = GEPF_RGB555;
		} else {
			s.pixelformat = GEPF_RGB454;
		}
		break;
	case 32:
		s.pixelformat = GEPF_RGB32;
		break;
	default:
		s.pixelformat = GEPF_RGB16;
		break;
	}


	ge_wait_sync(geinfo);
	ge_set_color(geinfo, 0, 0, 0, 0, s.pixelformat);
	ge_set_destination(geinfo, &s);
	ge_set_pixelformat(geinfo, s.pixelformat);
	ge_set_command(geinfo, GECMD_BLIT, 0xf0);
	ge_wait_sync(geinfo);

	if (offset) {
		// clear secondary framebuffer
		s.addr = g_fb_phy+offset;
		ge_wait_sync(geinfo);
		ge_set_color(geinfo, 0, 0, 0, 0, s.pixelformat);
		ge_set_destination(geinfo, &s);
		ge_set_pixelformat(geinfo, s.pixelformat);
		ge_set_command(geinfo, GECMD_BLIT, 0xf0);
		ge_wait_sync(geinfo);
		s.addr = g_fb_phy;
	} else {
		s.xres = info->xres;
		s.xres_virtual = info->xres_virtual;
	}
	amx_show_surface(geinfo, 0, &s, 0, 0); /* id:0, x:0, y:0 */
	amx_sync(geinfo);

	return 0;
}


int wmt_graphic_init(void)
{
	vpp_mod_base_t *mod_p;
	int i;
	unsigned int mod_mask;
	lcd_parm_t *p_oem_lcd;

	if (wmt_init_check())
		return -1;

//fan init
	vpp_mod_init();
	
	if (g_display_param.vout == VPP_VOUT_DVI)
		vt1632_module_init();
	if (g_display_param.vout == VPP_VOUT_LCD) {
		lcd_module_init();
		if (g_display_param.op1 == LCD_WMT_OEM) {
			p_oem_lcd = lcd_get_parm(LCD_WMT_OEM,24);
			memcpy(&p_oem_lcd->timing, &g_display_tmr, sizeof(vpp_timing_t));
			//fan , may need to check OEM ( fps , bits_per_pixel  ,	capability ) ??
			/*
			if (g_display_param.op2)
				p_oem_lcd->bits_per_pixel = g_display_param.op2;
			*/
		}
	}
//
	//g_vpp.disp_fb_max = VPP_DISP_FB_NUM;
	g_vpp.disp_fb_cnt = 0;
	g_vpp.govw_tg_dynamic = 1;
	g_vpp.video_quality_mode = 1;
	g_vpp.scale_keep_ratio = 1;
	g_vpp.govrh_field = VPP_FIELD_TOP;
	g_vpp.disp_fb_keep = 0;
	g_vpp.fbsync_enable = 1;

	g_vpp.govr = (vpp_mod_base_t*) p_govrh;

	/* check govrh preinit for uboot logo function */
	g_vpp.govrh_preinit = 0;
	g_vpp.direct_path = 0;

	printf("vpp_init(boot logo %d)\n",g_vpp.govrh_preinit);

	mod_mask = BIT(VPP_MOD_GOVRS) | BIT(VPP_MOD_GOVRH) | BIT(VPP_MOD_GOVW) | BIT(VPP_MOD_VPPM) | 
				BIT(VPP_MOD_GOVM);

	for(i=0;i<VPP_MOD_MAX;i++){
		if( !(mod_mask & (0x01 << i)) ) {
			continue;
		}
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->init ) {
			mod_p->init(mod_p);
		}
	}

	// init video out device
	vout_init(&vfb_var);
	govm_set_disp_coordinate(vfb_var.xres,vfb_var.yres);
	//vpp_set_video_quality(g_vpp.video_quality_mode);

	vpp_config(&vfb_var);

	ge_init(&vfb_var);

	g_display_vaild |= DISPLAY_ENABLE;

	return 0;

}
#undef WMT_DISPLAY

