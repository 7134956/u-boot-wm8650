/*++ 
 * linux/drivers/video/wmt/vout-wm8440.c
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

#include <common.h>
#include <malloc.h>
#include "vpp.h"
#include "vout.h"
#include "wmt_display.h"

#define CONFIG_LCD_VGA_DUAL_OUTPUT

//static int vo_plug_flag;
vout_mode_t vo_plug_mode;
int (*vo_plug_func)(void);
#ifdef CONFIG_WMT_EDID
char vout_edid[256];
#endif
extern int vpp_dac_sense_enable;
vout_mode_t dvo_vout_mode;
vout_mode_t int_vout_mode;
//vpp_timing_t vo_oem_tmr;

#if 0//fan
// GPIO 7 & 9
swi2c_reg_t vo_gpio_scl = {
	.bit_mask = BIT7,
	.gpio_en = (__GPIO_BASE + 0x40),
	.out_en = (__GPIO_BASE + 0x80),
	.data_in = (__GPIO_BASE + 0x00),
	.data_out = (__GPIO_BASE + 0xC0),
};

swi2c_reg_t vo_gpio_sda = {
	.bit_mask = BIT9,
	.gpio_en = (__GPIO_BASE + 0x40),
	.out_en = (__GPIO_BASE + 0x80),
	.data_in = (__GPIO_BASE + 0x00),
	.data_out = (__GPIO_BASE + 0xC0),
};

// I2C0 SCL & SDA
swi2c_reg_t vo_i2c0_scl = {
	.bit_mask = BIT8,
	.gpio_en = (__GPIO_BASE + 0x54),
	.out_en = (__GPIO_BASE + 0x94),
	.data_in = (__GPIO_BASE + 0x14),
	.data_out = (__GPIO_BASE + 0xD4),
};

swi2c_reg_t vo_i2c0_sda = {
	.bit_mask = BIT9,
	.gpio_en = (__GPIO_BASE + 0x54),
	.out_en = (__GPIO_BASE + 0x94),
	.data_in = (__GPIO_BASE + 0x14),
	.data_out = (__GPIO_BASE + 0xD4),
};

swi2c_handle_t vo_swi2c_vga = { 
	.scl_reg = &vo_i2c0_scl,
	.sda_reg = &vo_i2c0_sda,
};

swi2c_handle_t vo_swi2c_dvi = { 
	.scl_reg = &vo_gpio_scl,
	.sda_reg = &vo_gpio_sda,
};

extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
extern void hdmi_config_audio(vout_audio_t *info);

#endif

/*--------------------------------------- API ---------------------------------------*/

#if 0
static void vo_do_plug
(
	void *ptr		/*!<; // work input data */
)
{
	vout_t *vo;
	int plugin;

	if( vo_plug_func == 0 )
		return;

 	govrh_set_dvo_enable(1);
	plugin = vo_plug_func();
	govrh_set_dvo_enable(plugin);
	vo = vout_get_info(vo_plug_mode);
	vo->status = (vo->status & ~VPP_VOUT_STS_PLUGIN) | ((plugin)? VPP_VOUT_STS_PLUGIN:0);
	vo_plug_flag = 0;
	printf("vo_do_plug %d\n",plugin);
	return;
}
#endif
static void vo_plug_enable(int enable,void *func,vout_mode_t mode)
{
	printf("vo_plug_enable(%d)\n",enable);
	vo_plug_mode = mode;

	vo_plug_func = 0;
 	govrh_set_dvo_enable(enable);
}


void vo_set_govr_timing(unsigned int resx,unsigned int resy,unsigned int *pixclk)
{
	if( resx && resy && pixclk ){
		vpp_timing_t *timing;

		timing = vpp_get_video_mode(resx,resy,*pixclk);
		govrh_set_timing(timing);
		*pixclk = timing->pixel_clock;
	}
	else {
		if (g_display_vaild&TMRFROMENV) {
			govrh_set_timing(&g_display_tmr);
			printf("[VO] set OEM timing\n");
		} else
			printf("*E* no OEM timing\n");

	}
}

/*--------------------------------------- VGA ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_VGA
static int vo_vga_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_vga_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_BOOT:
		case VOUT_SD_DIGITAL:
		case VOUT_SD_ANALOG:
#ifndef CONFIG_LCD_VGA_DUAL_OUTPUT
		case VOUT_LCD:
#endif
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_vga_visible(int arg)
{
	printf("vo_vga_visible(%d)\n",arg);
	if( arg ){
		govrh_set_vga_enable(VPP_FLAG_ENABLE);
	}
	else {
		govrh_set_vga_enable(VPP_FLAG_DISABLE);
	}
	return 0;
}

static int vo_vga_config(int arg)
{
	int conf;
	vout_t *vo;
	vout_info_t *vo_info;

	vo_info = (vout_info_t *) arg;
	conf = 1;
	if( vppif_reg32_read(GOVRH_DVO_ENABLE) ){
		vo = vout_get_info(VOUT_DVI);
		if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
			conf = 0;
		}

		vo = vout_get_info(VOUT_DVO2HDMI);
		if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
			conf = 0;
		}

		vo = vout_get_info(VOUT_LCD);
		if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
			unsigned int line_pixel;
			vpp_timing_t *timing;

			timing = vpp_get_video_mode(vo_info->resx,vo_info->resy,vo_info->timing.pixel_clock);			
			line_pixel = timing->hsync + timing->hbp + timing->hpixel + timing->hfp;
			govrh_set_VGA_sync(timing->hsync, timing->vsync * line_pixel);
			govrh_set_VGA_sync_polar(1,1);
			conf = 0;
		}
	}

	vo = vout_get_info(VOUT_SD_DIGITAL);
	if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
		conf = 0;
	}

	vo = vout_get_info(VOUT_SD_ANALOG);
	if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
		conf = 0;
	}

	vo = vout_get_info(VOUT_HDMI);
	if( vo && (vo->status & VPP_VOUT_STS_ACTIVE ) ){
		conf = 0;
	}

	printf("vo_vga_config %d\n",conf);

	if( conf ){	
		printf("vo_vga_config %dx%d,%d\n",vo_info->resx,vo_info->resy,vo_info->timing.pixel_clock);
		if( (vo_info->resx == 1280) && (vo_info->resy == 720) ){
			if( (vo_info->timing.pixel_clock/1000) == 74250 ){
				vo_info->timing.pixel_clock = 74500000;
			}
		}
		vo_set_govr_timing(vo_info->resx,vo_info->resy,&vo_info->timing.pixel_clock);
	}
	return 0;
}

static int vo_vga_init(int arg)
{
	printf("vo_vga_init(%d)\n",arg);

	govrh_set_vga_enable(VPP_FLAG_ENABLE);
#ifdef CONFIG_WMT_INT_DEV_PLUG_DISABLE
	vpp_dac_sense_enable = 0;
	govrh_DAC_set_pwrdn(VPP_FLAG_DISABLE);
#else
	//vpp_set_vppm_int_enable(VPP_INT_GOVRH_VBIS,VPP_FLAG_ENABLE);
#endif
	govrh_DAC_set_sense_value(0,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	govrh_DAC_set_sense_value(1,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	govrh_DAC_set_sense_value(2,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	g_vpp.vga_enable = 1;
	return 0;
}

static int vo_vga_uninit(int arg)
{
	printf("vo_vga_uninit(%d)\n",arg);
	g_vpp.vga_enable = 0;
	//vpp_set_vppm_int_enable(VPP_INT_GOVRH_VBIS,VPP_FLAG_DISABLE);
	govrh_set_vga_enable(VPP_FLAG_DISABLE);
	govrh_DAC_set_pwrdn(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_vga_suspend(int arg)
{
	printf("vo_vga_suspend(%d)\n",arg);
	govrh_set_vga_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_vga_resume(int arg)
{
	printf("vo_vga_resume(%d)\n",arg);
	govrh_set_vga_enable(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_vga_chkplug(int arg)
{
	int plugin,sense;
	vout_t *vo;

	vo = vout_get_info(VOUT_VGA);
	govrh_DAC_set_sense_value(0,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	govrh_DAC_set_sense_value(1,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	govrh_DAC_set_sense_value(2,p_govrh->vag_dac_sense_val,p_govrh->vag_dac_sense_val);
	govrh_DAC_set_pwrdn(0);
	sense = govrh_DAC_monitor_sense();
	plugin = ((sense & 0x7) == 0x7)? 1:0;
	govrh_DAC_set_pwrdn((plugin)? 0:1);
    vo->status = (vo->status & ~VPP_VOUT_STS_PLUGIN) | ((plugin)? VPP_VOUT_STS_PLUGIN:0);
	VPPMSG("vo_vga_chkplug %d,0x%x\n",plugin,sense);
	return plugin;
}

int vo_vga_plug_detect(void)
{
	static unsigned int sense_cnt = 0;
	static unsigned int pre_plugin;	
	unsigned int plugin;

	if( g_vpp.vga_enable == 0 ){
		return -1;
	}

	sense_cnt++;
	if( (sense_cnt % p_govrh->vga_dac_sense_cnt) != 0 ){
		return -1;
	}

	if( vppif_reg32_read(GOVRH_DAC_PWRDN) == 1 ){
		vppif_reg32_write(GOVRH_DAC_PWRDN, 0);	/* DAC power on */
		sense_cnt--;
		return -1;
	}

	plugin = (govrh_DAC_monitor_sense() & BIT2)? 1:0;
	govrh_DAC_set_pwrdn((plugin)? 0:1);	
	if( plugin != pre_plugin ){
		govrh_set_vga_enable(plugin);
		pre_plugin = plugin;
		DPRINT("[VOUT] VGA PLUG %s\n",(plugin)?"IN":"OUT");
	}
	return plugin;
}

static int vo_vga_get_edid(int arg)
{
#ifdef CONFIG_WMT_EDID
	swi2c_I2C_ReadRegs(&vo_swi2c_vga,0xA0,0x0,0,128,&vout_edid[0]);
	if( vout_edid[0x7E] ){
		swi2c_I2C_ReadRegs(&vo_swi2c_vga,0xA0,0x0,128,128,&vout_edid[128]);
	}
#endif
	return 0;
}

vout_ops_t vo_vga_ops = 
{
	.init = vo_vga_init,
	.uninit = vo_vga_uninit,
	.compatible = vo_vga_compatible,
	.visible = vo_vga_visible,
	.config = vo_vga_config,
	.suspend = vo_vga_suspend,
	.resume = vo_vga_resume,
	.chkplug = vo_vga_chkplug,
	.get_edid = vo_vga_get_edid,
};

vout_t vo_vga_parm = {
	.ops = &vo_vga_ops,
	.name = "VGA",
	.option[0] = 0,
	.option[1] = 0,
	.option[2] = 0,
	.resx = VPP_HD_DISP_RESX,
	.resy = VPP_HD_DISP_RESY,
	.pixclk = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
};
#endif /* WMT_FTBLK_VOUT_VGA */

/*--------------------------------------- DVI ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_DVI
static int vo_dvi_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_dvi_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_BOOT:
		case VOUT_SD_DIGITAL:
//		case VOUT_SD_ANALOG:
		case VOUT_DVO2HDMI:
		case VOUT_LCD:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_dvi_visible(int arg)
{
	printf("vo_dvi_visible(%d)\n",arg);
	govrh_set_dvo_enable(arg);
	return 0;
}

static int vo_dvi_config(int arg)
{
	vout_info_t *vo_info;
	vout_t *vo;	

	printf("vo_dvi_config\n");

	vo_info = (vout_info_t *) arg;
	vo = vout_get_info(VOUT_SD_ANALOG);
	if( !(vo && (vo->status & VPP_VOUT_STS_ACTIVE )) ){
		vo_set_govr_timing(vo_info->resx,vo_info->resy,&vo_info->timing.pixel_clock);
	}
	vo = vout_get_info(VOUT_DVI);	
	if( vo->dev_ops ){
		vo->dev_ops->config(vo_info);
	}
	return 0;
}

static int vo_dvi_init(int arg)
{
	vout_t *vo;

	printf("vo_dvi_init(%d)\n",arg);

	vo = vout_get_info(VOUT_DVI);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
		vo->dev_ops->set_power_down(VPP_FLAG_DISABLE);
		vo_plug_enable(VPP_FLAG_ENABLE,vo->dev_ops->check_plugin,VOUT_DVI);
	}
	govrh_set_dvo_color_format(vo->option[0]);
	govrh_set_dvo_outdatw(vo->option[1]);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_dvi_uninit(int arg)
{
	vout_t *vo;
	
	printf("vo_dvi_uninit(%d)\n",arg);

	vo_plug_enable(VPP_FLAG_DISABLE,0,VOUT_MODE_MAX);
	vo = vout_get_info(VOUT_DVI);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(VPP_FLAG_ENABLE);
	}
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvi_suspend(int arg)
{
	vout_t *vo;

	printf("vo_dvi_suspend(%d)\n",arg);
	vo_plug_enable(VPP_FLAG_DISABLE,vo_plug_func,VOUT_MODE_MAX);
	vo = vout_get_info(VOUT_DVI);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(VPP_FLAG_ENABLE);
	}
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvi_resume(int arg)
{
	vout_t *vo;

	printf("vo_dvi_resume(%d)\n",arg);
	vo = vout_get_info(VOUT_DVI);
	if( vo->dev_ops ){
		vo->dev_ops->init();
	}
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	vo_plug_enable(VPP_FLAG_ENABLE,vo_plug_func,VOUT_DVI);
	return 0;
}

static int vo_dvi_chkplug(int arg)
{
	vout_t *vo;
	int plugin = 0;

	vo = vout_get_info(VOUT_DVI);
	if( vo->dev_ops ){
		plugin = vo->dev_ops->check_plugin();
	}
	printf("vo_dvi_chkplug %d\n",plugin);
	return plugin;
}

static int vo_dvi_get_edid(int arg)
{
#ifdef CONFIG_WMT_EDID
	swi2c_I2C_ReadRegs(&vo_swi2c_dvi,0xA0,0x0,0,128,&vout_edid[0]);
	if( vout_edid[0x7E] ){
		swi2c_I2C_ReadRegs(&vo_swi2c_dvi,0xA0,0x0,128,128,&vout_edid[128]);
	}
#endif
	return 0;
}

vout_ops_t vo_dvi_ops = 
{
	.init = vo_dvi_init,
	.uninit = vo_dvi_uninit,
	.compatible = vo_dvi_compatible,
	.visible = vo_dvi_visible,
	.config = vo_dvi_config,
	.suspend = vo_dvi_suspend,
	.resume = vo_dvi_resume,
	.chkplug = vo_dvi_chkplug,
	.get_edid = vo_dvi_get_edid,
};

vout_t vo_dvi_parm = {
	.ops = &vo_dvi_ops,
	.name = "DVI",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_24,
	.option[2] = 0,
	.resx = VPP_HD_DISP_RESX,
	.resy = VPP_HD_DISP_RESY,
	.pixclk = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
};
#endif /* WMT_FTBLK_VOUT_DVI */

/*--------------------------------------- DVO2HDMI ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_DVO2HDMI
static int vo_dvo2hdmi_init(int arg)
{
	vout_t *vo;
	vdo_color_fmt colfmt;

	printf("vo_dvo2hdmi_init(%d)\n",arg);

	vo = vout_get_info(VOUT_DVO2HDMI);
#if 0
	// EDID not support, default RGB mode
	if( edid_info.option & EDID_OPT_VALID ){
		switch( vo->option[0] ){
			case VDO_COL_FMT_YUV422H:
			case VDO_COL_FMT_YUV422V:
				if((edid_info.option & EDID_OPT_YUV422)==0)
					vo->option[0] = VDO_COL_FMT_ARGB;
				break;
			case VDO_COL_FMT_YUV444:
				if((edid_info.option & EDID_OPT_YUV444)==0)
					vo->option[0] = VDO_COL_FMT_ARGB;
				break;
			default:
				break;
		}
	}
#endif
	vo->option[1] = ( g_vpp.hdmi_audio_interface )? (vo->option[1]|BIT1):(vo->option[1]&~BIT1);	// 0-i2s,1-spdif
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
		vo_plug_enable(VPP_FLAG_ENABLE,vo->dev_ops->check_plugin,VOUT_DVO2HDMI);		
	}
	colfmt = (vo->option[0]==VDO_COL_FMT_YUV422V)? VDO_COL_FMT_YUV422H:vo->option[0];
	govrh_set_dvo_color_format(colfmt);
	govrh_set_dvo_outdatw(vo->option[1] & BIT0);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	return 0;
}

static int vo_dvo2hdmi_uninit(int arg)
{
	vout_t *vo;

	printf("vo_dvo2hdmi_uninit(%d)\n",arg);
	vo = vout_get_info(VOUT_DVO2HDMI);
	vo_plug_enable(VPP_FLAG_DISABLE,0,VOUT_MODE_MAX);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(1);
	}	
	return 0;
}

static int vo_dvo2hdmi_suspend(int arg)
{
	vout_t *vo;
	
	printf("vo_dvo2hdmi_suspend(%d)\n",arg);
	vo = vout_get_info(VOUT_DVO2HDMI);	
	vo_plug_enable(VPP_FLAG_DISABLE,vo_plug_func,VOUT_MODE_MAX);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(1);
	}
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvo2hdmi_resume(int arg)
{
	vout_t *vo;
	
	printf("vo_dvo2hdmi_resume(%d)\n",arg);
	vo = vout_get_info(VOUT_DVO2HDMI);
	if( vo->dev_ops ){
		vo->dev_ops->init();
	}
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	vo_plug_enable(VPP_FLAG_ENABLE,vo_plug_func,VOUT_DVO2HDMI);
	return 0;
}

static int vo_dvo2hdmi_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_dvo2hdmi_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_BOOT:
//		case VOUT_VGA:
		case VOUT_DVI:
		case VOUT_SD_DIGITAL:
//		case VOUT_SD_ANALOG:
		case VOUT_LCD:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_dvo2hdmi_visible(int arg)
{
	printf("vo_dvo2hdmi_visible(%d)\n",arg);

	govrh_set_dvo_enable(arg);
	return 0;
}

static int vo_dvo2hdmi_config(int arg)
{
	vout_info_t *vo_info;
	//unsigned int reg;
	unsigned int sense;
	vout_t *vo;

	printf("vo_dvo2hdmi_config\n");

	vo = vout_get_info(VOUT_DVO2HDMI);
	vo_info = (vout_info_t *) arg;
	// 1280x720@60, HDMI pixel clock 74250060 not 74500000
	if( vppif_reg32_read(GOVRH_DVO_ENABLE) ){	
		if( (vo_info->resx == 1280) && (vo_info->resy == 720) ){
			if( vo_info->timing.pixel_clock == 74500000 ){
				vo_info->timing.pixel_clock = 74250060;
			}
		}
	}
	vo_set_govr_timing(vo_info->resx,vo_info->resy,&vo_info->timing.pixel_clock);

	sense = vpp_dac_sense_enable;
	vpp_dac_sense_enable = 0;
#ifdef WMT_FTBLK_GOVRH_DAC		
	reg = vppif_reg32_read(GOVRH_DAC_PWRDN);
	vppif_reg32_write(GOVRH_DAC_PWRDN,0xFF);
#endif	
	govrh_set_tg_enable(VPP_FLAG_ENABLE);
	if( vo->dev_ops ){
		vo->dev_ops->config(vo_info);
	}
	govrh_set_tg_enable(VPP_FLAG_DISABLE);
#ifdef WMT_FTBLK_GOVRH_DAC		
	vppif_reg32_write(GOVRH_DAC_PWRDN,reg);
#endif	
	vpp_dac_sense_enable = sense;
	return 0;
}

static int vo_dvo2hdmi_chkplug(int arg)
{
	vout_t *vo;
	int plugin = 0;
//fan
	//mdelay(100); // wait plugin status

	vo = vout_get_info(VOUT_DVO2HDMI);
	if( vo->dev_ops ){
		plugin = vo->dev_ops->check_plugin();
	}
	printf("vo_dvo2hdmi_chkplug %d\n",plugin);
	return plugin;
}

static int vo_dvo2hdmi_get_edid(int arg)
{
	vout_t *vo;

	printf("vo_dvo2hdmi_get_edid\n");
	vo = vout_get_info(VOUT_DVO2HDMI);
#ifdef CONFIG_WMT_EDID
	if( vo->dev_ops ){
		vo->dev_ops->get_edid(vout_edid);
	}
#endif
	return 0;
}

vout_ops_t vo_dvo2hdmi_ops = 
{
	.init = vo_dvo2hdmi_init,
	.uninit = vo_dvo2hdmi_uninit,
	.compatible = vo_dvo2hdmi_compatible,
	.visible = vo_dvo2hdmi_visible,
	.config = vo_dvo2hdmi_config,
	.suspend = vo_dvo2hdmi_suspend,
	.resume = vo_dvo2hdmi_resume,
	.chkplug = vo_dvo2hdmi_chkplug,
	.get_edid = vo_dvo2hdmi_get_edid,
};

vout_t vo_dvo2hdmi_parm = {
	.ops = &vo_dvo2hdmi_ops,
	.name = "DVO2HDMI",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_12,
	.option[2] = 0,
	.resx = 1280,
	.resy = 720,
	.pixclk = 74250060,
};
#endif /* WMT_FTBLK_VOUT_DVO2HDMI */

/*--------------------------------------- LCD ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_LCD
static int vo_lcd_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_lcd_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_BOOT:
		case VOUT_DVO2HDMI:
		case VOUT_DVI:
		case VOUT_SD_DIGITAL:
		case VOUT_SD_ANALOG:
#ifndef CONFIG_LCD_VGA_DUAL_OUTPUT
		case VOUT_VGA:
#endif
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_lcd_visible(int arg)
{
	vout_t *vo;

	printf("vo_lcd_visible(%d)\n",arg);
	vo = vout_get_info(VOUT_LCD);	
	govrh_set_dvo_enable(arg);
	if( vo->dev_ops ){
		//--> added by SZ on 20101120
		//purpose: don't set visable after execute vo_lcd_init()
		if(arg && g_vpp.vo_enable == 1)
		{
		    return 0;
		}
		else
		{
		//<-- end
 		    vo->dev_ops->set_power_down(!arg);
 		}
	}
	return 0;
}

static int vo_lcd_config(int arg)
{
	vout_t *vo;
	vout_info_t *vo_info;

	printf("vo_lcd_config\n");

	vo_info = (vout_info_t *) arg;
	vo = vout_get_info(VOUT_LCD);
	if( vo->dev_ops ){
		vo->dev_ops->config(vo_info);
	}

	if (g_display_vaild&TMRFROMENV)
		govrh_set_timing(&g_display_tmr);
	else
		govrh_set_timing(&vo_info->timing);
	return 0;
}

static int vo_lcd_init(int arg)
{
	vout_t *vo;
	unsigned int capability;

	printf("vo_lcd_init(%d)\n",arg);

	vo = vout_get_info(VOUT_LCD);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
	}
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	govrh_set_dvo_color_format(VDO_COL_FMT_ARGB);
	govrh_set_dvo_outdatw(VPP_DATAWIDHT_24);
	capability = vo->option[2];
	govrh_set_dvo_clock_delay((capability & LCD_CAP_CLK_HI)? 0:1, 0);
	govrh_set_dvo_sync_polar((capability & LCD_CAP_HSYNC_HI)? 0:1,(capability & LCD_CAP_VSYNC_HI)? 0:1);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	switch( vo->option[1] ){
		case 15:
			govrh_IGS_set_mode(0,1,1);
			break;
		case 16:
			govrh_IGS_set_mode(0,3,1);
			break;
		case 18:
			govrh_IGS_set_mode(0,2,1);
			break;
		case 24:
		default:
			govrh_IGS_set_mode(0,0,0);
			break;
	}
	g_vpp.vo_enable = 1;
	return 0;
}

static int vo_lcd_uninit(int arg)
{
	vout_t *vo;
	
	printf("vo_lcd_uninit(%d)\n",arg);
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	vo = vout_get_info(VOUT_LCD);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(0);
	}
	return 0;
}

static int vo_lcd_suspend(int arg)
{
	vout_t *vo;

	printf("vo_lcd_suspend(%d)\n",arg);

	vo = vout_get_info(VOUT_LCD);	
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(1);
	}
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_lcd_resume(int arg)
{
	vout_t *vo;
	
	printf("vo_lcd_resume(%d)\n",arg);

	REG32_VAL(0xd8130250) |= BIT10;
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	//fan
	//mdelay(150);
	vo = vout_get_info(VOUT_LCD);	
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(0);
	}
	return 0;
}

static int vo_lcd_chkplug(int arg)
{
	vout_t *vo;
	
	vo = vout_get_info(VOUT_LCD);	
	if( vo->dev_ops ){
		return vo->dev_ops->check_plugin();
	}
	return 0;
}

vout_ops_t vo_lcd_ops = 
{
	.init = vo_lcd_init,
	.uninit = vo_lcd_uninit,
	.compatible = vo_lcd_compatible,
	.visible = vo_lcd_visible,
	.config = vo_lcd_config,
	.suspend = vo_lcd_suspend,
	.resume = vo_lcd_resume,
	.chkplug = vo_lcd_chkplug,
};

vout_t vo_lcd_parm = {
	.ops = &vo_lcd_ops,
	.name = "LCD",
	.option[0] = LCD_PANEL_MAX,		/* [LCD] option1 : panel id */
	.option[1] = 24,				/* [LCD] option2 : bit per pixel */
	.option[2] = 0,					/* [LCD] option3 : capability */
};

void vo_set_lcd_id(int id)
{
	vo_lcd_parm.option[0] = id;
}

int vo_get_lcd_id(void)
{
	return vo_lcd_parm.option[0];
}
#endif /* WMT_FTBLK_VOUT_LCD */

/*--------------------------------------- SD_ANALOG ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_SDA
static int vo_sda_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_sda_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_VGA:
//		case VOUT_DVI:
//		case VOUT_DVO2HDMI:
		case VOUT_LCD:
		case VOUT_SD_DIGITAL:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_sda_visible(int arg)
{
	printf("vo_sda_visible(%d)\n",arg);
	if( arg ){
		disp_set_enable(VPP_FLAG_ENABLE);
	}
	else {
		disp_set_enable(VPP_FLAG_DISABLE);
	}
	return 0;
}

vpp_tvsys_t vo_res_to_tvsys(unsigned int resx,unsigned int resy)
{
	if( resx == 1280 )
		return VPP_TVSYS_720P;

	if( resx == 1920 )
		return VPP_TVSYS_1080I;
	
	if( resx != 720 ){
		return VPP_TVSYS_MAX;
	}

	if( resy == 576 ){
		return VPP_TVSYS_PAL;
	}
	return VPP_TVSYS_NTSC;
}

static int vo_sda_config(int arg)
{
	vout_info_t *vo_info;
	unsigned int pixel_clock;
	
	printf("vo_sda_config\n");

	vo_info = (vout_info_t *) arg;
	switch(p_disp->tvsys){
		default:
		case VPP_TVSYS_NTSC:
			vo_info->resx = 720;
			vo_info->resy = 480;
			pixel_clock = 27027060;
			break;
		case VPP_TVSYS_PAL:
			vo_info->resx = 720;
			vo_info->resy = 576;
			pixel_clock = 27000000;
			break;	
		case VPP_TVSYS_720P:
			vo_info->resx = 1280;
			vo_info->resy = 720;
			pixel_clock = 74250000;
			break;
		case VPP_TVSYS_1080I:
			vo_info->resx = 1920;
			vo_info->resy = 1080;
			pixel_clock = 74250060;
			break;
		case VPP_TVSYS_1080P:
			vo_info->resx = 1920;
			vo_info->resy = 1080;
			pixel_clock = 148500000;
			break;
	}
	vpp_set_video_mode(vo_info->resx,vo_info->resy,pixel_clock);
	vo_info->timing.pixel_clock = pixel_clock;
	disp_set_mode(p_disp->tvsys,p_disp->tvconn);
	return 0;
}

static int vo_sda_init(int arg)
{
	vout_t *vo;

	vo = vout_get_info(VOUT_SD_ANALOG);

	printf("vo_sda_init(%d)\n",arg);

	p_disp->tvsys = vo->option[0];
	p_disp->tvconn = vo->option[1];
	govrh_DISP_set_enable(VPP_FLAG_ENABLE);
	vppm_set_DAC_select(1);
	govrh_DAC_set_sense_value(0,p_disp->dac_sense_val,p_disp->dac_sense_val);
	govrh_DAC_set_sense_value(1,p_disp->dac_sense_val,p_disp->dac_sense_val);
	govrh_DAC_set_sense_value(2,p_disp->dac_sense_val,p_disp->dac_sense_val);
	govrh_DAC_set_sense_sel(1);
	govrh_set_dual_disp(1,1);
	disp_EXTTV_set_enable(VPP_FLAG_DISABLE);
#ifdef CONFIG_WMT_INT_DEV_PLUG_DISABLE
	vpp_dac_sense_enable = 0;
	disp_DAC_set_pwrdn(0xFF,VPP_FLAG_DISABLE);
#endif	
	return 0;
}

static int vo_sda_uninit(int arg)
{
	printf("vo_sda_uninit(%d)\n",arg);
	disp_set_enable(VPP_FLAG_DISABLE);
	govrh_DISP_set_enable(VPP_FLAG_DISABLE);
	disp_DAC_set_pwrdn(0xFF,VPP_FLAG_ENABLE);
	vppm_set_DAC_select(0);
	govrh_DAC_set_sense_sel(0);
	govrh_set_dual_disp(0,1);	
	vppif_reg32_write(GOVRH_HSCALE_UP,0);
	return 0;
}

static int vo_sda_suspend(int arg)
{
	printf("vo_sda_suspend(%d)\n",arg);
	disp_set_enable(VPP_FLAG_DISABLE);
	disp_DAC_set_pwrdn(0xFF,VPP_FLAG_ENABLE);	
	return 0;
}

static int vo_sda_resume(int arg)
{
	printf("vo_sda_resume(%d)\n",arg);
	vppm_set_DAC_select(1);	
	disp_set_mode(p_disp->tvsys, p_disp->tvconn);	
//	disp_set_enable(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_sda_chkplug(int arg)
{
	int plugin,sense;

	govrh_DAC_set_pwrdn(0);
	sense = govrh_DAC_monitor_sense();
	plugin = (sense)? 1:0;
	govrh_DAC_set_pwrdn((plugin)? 0:1);
	printf("vo_sda_chkplug %d,0x%x\n",plugin,sense);
	return plugin;
}

int vo_sda_plug_detect(void)
{
	static unsigned int sense_cnt = 0;
	static unsigned int pre_plugin;
	unsigned int plugin,sense_val;

	if( vppif_reg32_read(DISP_EXTTV_EN) ){
		return -1;
	}

	sense_cnt++;
	if( (sense_cnt % p_disp->dac_sense_cnt) != 0 ){
		return -1;
	}

	if( vppif_reg32_read(GOVRH_DAC_PWRDN) == 1 ){
		vppif_reg32_write(GOVRH_DAC_PWRDN, 0);	/* DAC power on */
		sense_cnt--;
		return -1;
	}

	vppif_reg32_write(VPP_DAC_SEL,0);
	sense_val = govrh_DAC_monitor_sense();
	vppif_reg32_write(VPP_DAC_SEL,1);
 	plugin = (sense_val)? 1:0;
	govrh_DAC_set_pwrdn((plugin)? 0:1);
	if( plugin != pre_plugin ){
		pre_plugin = plugin;
		DPRINT("[VOUT] SDA PLUG %s,0x%x\n",(plugin)?"IN":"OUT",sense_val);
	}
	return plugin;
}

vout_ops_t vo_sda_ops = 
{
	.init = vo_sda_init,
	.uninit = vo_sda_uninit,
	.compatible = vo_sda_compatible,
	.visible = vo_sda_visible,
	.config = vo_sda_config,
	.suspend = vo_sda_suspend,
	.resume = vo_sda_resume,
	.chkplug = vo_sda_chkplug,
};

vout_t vo_sda_parm = {
	.ops = &vo_sda_ops,
	.name = "SD_ANALOG",
	.option[0] = VPP_TVSYS_NTSC,
	.option[1] = VPP_TVCONN_CVBS,
	.option[2] = 0,
	.resx = 720,
	.resy = 480,
	.pixclk = 27000000,
};
#endif /* WMT_FTBLK_VOUT_SDA */

/*--------------------------------------- SD_DIGITAL ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_SDD
static int vo_sdd_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_sdd_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_VGA:
		case VOUT_DVI:
		case VOUT_DVO2HDMI:
		case VOUT_LCD:
		case VOUT_SD_ANALOG:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_sdd_visible(int arg)
{
	printf("vo_sdd_visible(%d)\n",arg);
	if( arg ){
		disp_set_enable(VPP_FLAG_ENABLE);
	}
	else {
		disp_set_enable(VPP_FLAG_DISABLE);
	}
	return 0;
}

static int vo_sdd_config(int arg)
{
	vout_t *vo;
	vout_info_t *vo_info;

	printf("vo_sdd_config\n");

	vo = vout_get_info(VOUT_SD_DIGITAL);
	vo_info = (vout_info_t *) arg;
	p_disp->tvsys = vo->option[0] = vo_res_to_tvsys(vo_info->resx,vo_info->resy);
	vpp_set_video_mode(720,(p_disp->tvsys==VPP_TVSYS_NTSC)?480:576,27027060);
	disp_set_mode(p_disp->tvsys,p_disp->tvconn);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
	}
	return 0;
}

static int vo_sdd_init(int arg)
{
	vout_t *vo;

	printf("vo_sdd_init(%d)\n",arg);
	vo = vout_get_info(VOUT_SD_DIGITAL);
	p_disp->tvsys = vo->option[0];
	p_disp->tvconn = vo->option[1];
	govrh_DISP_set_enable(VPP_FLAG_ENABLE);
	vppm_set_DAC_select(1);
	disp_EXTTV_set_enable(VPP_FLAG_ENABLE);
	disp_set_enable(VPP_FLAG_ENABLE);
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);	// enable GPIO setting
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
	}
	return 0;
}

static int vo_sdd_uninit(int arg)
{
	vout_t *vo;

	printf("vo_sdd_uninit(%d)\n",arg);
	vo = vout_get_info(VOUT_SD_DIGITAL);
	disp_set_enable(VPP_FLAG_DISABLE);
	disp_EXTTV_set_enable(VPP_FLAG_DISABLE);
	govrh_DISP_set_enable(VPP_FLAG_DISABLE);
	vppm_set_DAC_select(0);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(VPP_FLAG_ENABLE);
	}
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);	// disable GPIO setting
	return 0;
}

static int vo_sdd_suspend(int arg)
{
	vout_t *vo;

	printf("vo_sdd_suspend(%d)\n",arg);
	vo = vout_get_info(VOUT_SD_DIGITAL);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(VPP_FLAG_ENABLE);
	}
	disp_set_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_sdd_resume(int arg)
{
	vout_t *vo;

	printf("vo_sdd_resume(%d)\n",arg);
	vo = vout_get_info(VOUT_SD_DIGITAL);
	REG32_VAL(0xd8110044) = 0x0;
	REG32_VAL(0xd8110048) = 0x0;
	vppm_set_DAC_select(1);
	disp_set_mode(p_disp->tvsys, p_disp->tvconn);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
	}
	return 0;
}

static int vo_sdd_chkplug(int arg)
{
	vout_t *vo;
	int plugin = 0;

	vo = vout_get_info(VOUT_SD_DIGITAL);
	if( vo->dev_ops ){
		plugin = vo->dev_ops->check_plugin();
	}
	return plugin;
}

vout_ops_t vo_sdd_ops = 
{
	.init = vo_sdd_init,
	.uninit = vo_sdd_uninit,
	.compatible = vo_sdd_compatible,
	.visible = vo_sdd_visible,
	.config = vo_sdd_config,
	.suspend = vo_sdd_suspend,
	.resume = vo_sdd_resume,
	.chkplug = vo_sdd_chkplug,
};

vout_t vo_sdd_parm = {
	.ops = &vo_sdd_ops,
	.name = "SD_DIGITAL",
	.option[0] = VPP_TVSYS_NTSC,
	.option[1] = VPP_TVCONN_CVBS,
	.option[2] = 0,
	.resx = 720,
	.resy = 480,
	.pixclk = 27000000,
};
#endif /* WMT_FTBLK_VOUT_SDD */

/*--------------------------------------- BOOT LOGO ---------------------------------------*/
static int vo_boot_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_boot_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_VGA:
		case VOUT_DVI:
		case VOUT_DVO2HDMI:
		case VOUT_LCD:
		case VOUT_SD_ANALOG:
		case VOUT_SD_DIGITAL:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_boot_visible(int arg)
{
	printf("vo_boot_visible(%d)\n",arg);

	return 0;
}

static int vo_boot_config(int arg)
{
	//vout_info_t *vo_info;

	printf("vo_boot_config\n");

	return 0;
}

static int vo_boot_init(int arg)
{
	vout_t *vo;

	vo = vout_get_info(VOUT_BOOT);

	printf("vo_boot_init(%d)\n",arg);

	return 0;
}

static int vo_boot_uninit(int arg)
{
	printf("vo_boot_uninit(%d)\n",arg);
	return 0;
}

static int vo_boot_suspend(int arg)
{
	return 0;
}

static int vo_boot_resume(int arg)
{
	printf("vo_boot_resume(%d)\n",arg);
	return 0;
}

vout_t vo_boot_parm;
static int vo_boot_chkplug(int arg)
{

	return 0;
}

vout_ops_t vo_boot_ops = 
{
	.init = vo_boot_init,
	.uninit = vo_boot_uninit,
	.compatible = vo_boot_compatible,
	.visible = vo_boot_visible,
	.config = vo_boot_config,
	.suspend = vo_boot_suspend,
	.resume = vo_boot_resume,
	.chkplug = vo_boot_chkplug,
};

vout_t vo_boot_parm = {
	.ops = &vo_boot_ops,
	.name = "BOOT",
	.option[0] = 0,
	.option[1] = 0,
	.option[2] = 0,
	.resx = 0,
	.resy = 0,
	.pixclk = 0,
};

/*--------------------------------------- HDMI ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_HDMI
#ifdef CONFIG_HDMI_INTERRUPT
static irqreturn_t vo_hdmi_plug_interrupt
(
	int irq, 				/*!<; // irq id */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
	void *dev_id 			/*!<; // device id */
#else
	void *dev_id, 			/*!<; // device id */
	struct pt_regs *regs	/*!<; // reg pointer */
#endif
)
{
	DPRINT("vo_hdmi_plug_interrupt %d\n",irq);
	hdmi_check_plugin();
	return IRQ_HANDLED;	
}
#else
struct timer_list hdmi_timer;
unsigned int hdmi_cur_plugin;
int hdmi_status_cnt;
unsigned int hdmi_status_array[1000];
#define VOUT_HDMI_DETECT_TIME 5

int vo_hdmi_plug_detect(void)
{
	unsigned int plugin;

	plugin = hdmi_get_plugin();
	if( plugin != hdmi_cur_plugin ){
		hdmi_cur_plugin = plugin;
		DPRINT("[VOUT] HDMI PLUG %s\n",(plugin)?"IN":"OUT");
		hdmi_set_enable(plugin);
	}
	hdmi_timer.expires = jiffies + VOUT_HDMI_DETECT_TIME * HZ;
	add_timer(&hdmi_timer);

	if( hdmi_status_cnt ){
		hdmi_status_cnt--;
		if( hdmi_status_cnt == 0 ){
			int i,cnt;

			for(i=0;i<1000;i++){
				hdmi_status_array[i] = vppif_reg32_in(0xd806c280);
			}
			for(i=0,cnt=0;i<1000;i++){
				if( (hdmi_status_array[i] & 0xFF000000) == 0x78000000 ){
					cnt++;
				}
			}
			printf("[HDMI] audio status cnt %d\n",cnt);
		}
	}
	
	return plugin;
}
#endif

static int vo_hdmi_init(int arg)
{
	DPRINT("vo_hdmi_init(%d)\n",arg);

	hdmi_init();
	hdmi_set_enable(VPP_FLAG_ENABLE);
	lvds_set_enable(0);
	hdmi_enable_plugin(1);
	
#ifdef CONFIG_HDMI_INTERRUPT
	if ( request_irq(VPP_IRQ_HDMI_HPDH, vo_hdmi_plug_interrupt, SA_INTERRUPT, "hdmi plug", (void *) 0) ) {
		VPPMSG("*E* request HDMI ISR fail\n");
	}
	if ( request_irq(VPP_IRQ_HDMI_HPDL, vo_hdmi_plug_interrupt, SA_INTERRUPT, "hdmi plug", (void *) 0) ) {
		VPPMSG("*E* request HDMI ISR fail\n");
	}
#else
	init_timer(&hdmi_timer);
	hdmi_timer.data = VOUT_HDMI;
	hdmi_timer.function = (void *) vout_plug_detect;
	hdmi_timer.expires = jiffies + VOUT_HDMI_DETECT_TIME * HZ;
	add_timer(&hdmi_timer);
#endif
	return 0;
}

static int vo_hdmi_uninit(int arg)
{
	printf("vo_hdmi_uninit(%d)\n",arg);
	hdmi_enable_plugin(0);
	hdmi_set_enable(VPP_FLAG_DISABLE);
#ifdef CONFIG_HDMI_INTERRUPT
	free_irq(VPP_IRQ_HDMI_HPDH,(void *) 0);
	free_irq(VPP_IRQ_HDMI_HPDL,(void *) 0);
#else
	del_timer(&hdmi_timer);
#endif
	hdmi_cur_plugin = 0;
	return 0;
}

static int vo_hdmi_suspend(int arg)
{
	printf("vo_hdmi_suspend(%d)\n",arg);
	hdmi_set_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_hdmi_resume(int arg)
{
	printf("vo_hdmi_resume(%d)\n",arg);
	hdmi_set_enable(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_hdmi_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_hdmi_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_LVDS:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_hdmi_visible(int arg)
{
	printf("vo_hdmi_visible(%d)\n",arg);

	hdmi_set_enable(arg);
	return 0;
}

static int vo_hdmi_config(int arg)
{
	vout_t *vo;
	vout_info_t *vo_info;
	hdmi_info_t hdmi_info;
	vdo_color_fmt colfmt;
	vpp_timing_t *timing;

	hdmi_set_enable(0);
	vpp_wait_vsync();
	
	vo = vout_get_info(VOUT_HDMI);	
	printf("vo_hdmi_config\n");

	vo_info = (vout_info_t *) arg;
	// 1280x720@60, HDMI pixel clock 74250060 not 74500000
	if( vppif_reg32_read(GOVRH_DVO_ENABLE) ){	
		if( (vo_info->resx == 1280) && (vo_info->resy == 720) ){
			if( vo_info->timing.pixel_clock == 74500000 ){
				vo_info->timing.pixel_clock = 74250060;
			}
		}
	}
	vpp_set_video_mode(vo_info->resx,vo_info->resy,vo_info->timing.pixel_clock);
	colfmt = (vo->option[0]==VDO_COL_FMT_YUV422V)? VDO_COL_FMT_YUV422H:vo->option[0];
	hdmi_info.channel = 2;
	hdmi_info.freq = 48000;
	hdmi_info.outfmt = colfmt;
	timing = vpp_get_video_mode(vo_info->resx,vo_info->resy,vo_info->timing.pixel_clock);
	hdmi_info.vic = VPP_GET_OPT_HDMI_VIC(timing->option);
	hdmi_config(&hdmi_info);
	govrh_set_csc_mode(p_govrh->fb_p->csc_mode);
	hdmi_set_enable(1);
	return 0;
}

static int vo_hdmi_chkplug(int arg)
{
	int plugin;

	plugin = hdmi_get_plugin();
	printf("vo_hdmi_chkplug %d\n",plugin);	
	return plugin;
}

static int vo_hdmi_get_edid(int arg)
{
	printf("vo_hdmi_get_edid\n");
//	hdmi_read_DDC(char addr, char index, char * buf, int length)
	return 0;
}

vout_ops_t vo_hdmi_ops = 
{
	.init = vo_hdmi_init,
	.uninit = vo_hdmi_uninit,
	.compatible = vo_hdmi_compatible,
	.visible = vo_hdmi_visible,
	.config = vo_hdmi_config,
	.suspend = vo_hdmi_suspend,
	.resume = vo_hdmi_resume,
	.chkplug = vo_hdmi_chkplug,
	.get_edid = vo_hdmi_get_edid,
};

vout_t vo_hdmi_parm = {
	.ops = &vo_hdmi_ops,
	.name = "HDMI",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_12 + BIT1,	// default SPDIF
	.option[2] = 0,
	.resx = 1280,
	.resy = 720,
	.pixclk = 74250060,
};
#endif /* WMT_FTBLK_VOUT_HDMI */

/*--------------------------------------- LVDS ---------------------------------------*/
#ifdef WMT_FTBLK_VOUT_LVDS
static int vo_lvds_compatible(int arg)
{
	vout_mode_t mode;

	printf("vo_lvds_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_BOOT:
		case VOUT_DVO2HDMI:
		case VOUT_DVI:
		case VOUT_SD_DIGITAL:
		case VOUT_SD_ANALOG:
#ifndef CONFIG_LCD_VGA_DUAL_OUTPUT
		case VOUT_VGA:
#endif
		case VOUT_HDMI:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_lvds_visible(int arg)
{
	vout_t *vo;

	printf("vo_lvds_visible(%d)\n",arg);
	vo = vout_get_info(VOUT_LVDS);
	lvds_set_enable(arg);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(!arg);
	}
	return 0;
}

static int vo_lvds_config(int arg)
{
	vout_t *vo;
	vout_info_t *vo_info;

	printf("vo_lvds_config\n");

	vo_info = (vout_info_t *) arg;
	vo = vout_get_info(VOUT_LVDS);
	if( vo->dev_ops ){
		vo->dev_ops->config(vo_info);
	}
	vpp_set_video_mode(vo_info->resx,vo_info->resy,vo_info->timing.pixel_clock);
	return 0;
}

static int vo_lvds_init(int arg)
{
	vout_t *vo;
	unsigned int capability;

	printf("vo_lvds_init(%d)\n",arg);

	vo = vout_get_info(VOUT_LVDS);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(&vo->option[0]);
	}
	capability = vo->option[2];
	lvds_set_enable(VPP_FLAG_ENABLE);
	govrh_set_csc_mode(p_govrh->fb_p->csc_mode);
	switch( vo->option[1] ){
		case 15:
			lvds_set_rgb_type(1);
			break;
		case 16:
			lvds_set_rgb_type(3);
			break;
		case 18:
			lvds_set_rgb_type(2);
			break;
		case 24:
		default:
			lvds_set_rgb_type(0);
			break;
	}
	return 0;
}

static int vo_lvds_uninit(int arg)
{
	vout_t *vo;
	
	printf("vo_lvds_uninit(%d)\n",arg);
	lvds_set_enable(VPP_FLAG_DISABLE);
	vo = vout_get_info(VOUT_LVDS);
	if( vo->dev_ops ){
		vo->dev_ops->set_mode(0);
	}
	return 0;
}

static int vo_lvds_suspend(int arg)
{
	vout_t *vo;

	printf("vo_lvds_suspend(%d)\n",arg);

	vo = vout_get_info(VOUT_LVDS);	
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(1);
	}
	lvds_set_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_lvds_resume(int arg)
{
	vout_t *vo;
	
	printf("vo_lvds_resume(%d)\n",arg);

	lvds_set_enable(VPP_FLAG_ENABLE);
	mdelay(150);
	vo = vout_get_info(VOUT_LVDS);
	if( vo->dev_ops ){
		vo->dev_ops->set_power_down(0);
	}
	return 0;
}

static int vo_lvds_chkplug(int arg)
{
	vout_t *vo;

	printf("vo_lvds_chkplug\n");
	
	lvds_init();
	vo = vout_get_info(VOUT_LVDS);
	if( vo->dev_ops ){
		return vo->dev_ops->check_plugin();
	}
	return 0;
}

vout_ops_t vo_lvds_ops = 
{
	.init = vo_lvds_init,
	.uninit = vo_lvds_uninit,
	.compatible = vo_lvds_compatible,
	.visible = vo_lvds_visible,
	.config = vo_lvds_config,
	.suspend = vo_lvds_suspend,
	.resume = vo_lvds_resume,
	.chkplug = vo_lvds_chkplug,
};

vout_t vo_lvds_parm = {
	.ops = &vo_lvds_ops,
	.name = "LVDS",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = 24,
	.option[2] = 0,
	.resx = 1024,
	.resy = 768,
	.pixclk = 65000000,
};
#endif /* WMT_FTBLK_VOUT_LVDS */

vout_t *vo_parm_table[] = {
	0, //	&vo_sda_parm,
	0, //	&vo_sdd_parm,
	&vo_lcd_parm,
	&vo_dvi_parm,
	0, // &vo_hdmi_parm,
	&vo_dvo2hdmi_parm,
	0, // &vo_lvds_parm,
	0, // &vo_vga_parm,
	&vo_boot_parm,
};

/*--------------------------------------- API ---------------------------------------*/
void vout_set_audio(vout_audio_t *arg)
{
	vout_t *vout;
	
	vout = vout_get_info(VOUT_DVO2HDMI);
	if( vout && (vout->status & VPP_VOUT_STS_PLUGIN)){
		if( vout->dev_ops->set_audio ){
			vout->dev_ops->set_audio(arg);
		}
	}

#ifdef WMT_FTBLK_VOUT_HDMI
	vout = vout_get_info(VOUT_HDMI);
	if( vout && (vout->status & VPP_VOUT_STS_PLUGIN)){
		hdmi_config_audio(arg);
	}
//	hdmi_status_cnt = 4;	
#endif	
}

void vout_plug_detect(vout_mode_t mode)
{
	int plugin = -1;

	if( vpp_dac_sense_enable == 0 ){
		return;
	}

	switch(mode){
#ifdef WMT_FTBLK_VOUT_VGA		
		case VOUT_VGA:
			plugin = vo_vga_plug_detect();
			break;
#endif
#ifdef WMT_FTBLK_VOUT_SDA
		case VOUT_SD_ANALOG:
			plugin = vo_sda_plug_detect();
			break;
#endif
#ifdef WMT_FTBLK_VOUT_HDMI
#ifndef CONFIG_HDMI_INTERRUPT
		case VOUT_HDMI:
			plugin = vo_hdmi_plug_detect();
			break;
#endif
#endif
		default:
			break;
	}

	if( plugin >= 0 ){
		vout_t *vo;

		vo = vout_get_info(mode);
		vo->status = (vo->status & ~VPP_VOUT_STS_PLUGIN) | ((plugin)? VPP_VOUT_STS_PLUGIN:0);
	}
}

#define VOUT_EXTDEV_DEFAULT_MODE	VOUT_DVI	// default external vout (VT1632 hw mode)
vout_mode_t vout_extdev_probe(void)
{
	vout_mode_t mode = VOUT_MODE_MAX;
	vout_t *parm = 0;
	vout_dev_ops_t *dev = 0;

	do {
		dev = vout_get_device(dev);
		if( dev == 0 ) break;
		if( dev->init() == 0 ){
			mode = dev->mode;
			parm = vo_parm_table[mode];
			parm->dev_ops = dev;
			break;
		}
		if( dev->mode == VOUT_EXTDEV_DEFAULT_MODE ){
			mode = dev->mode;
			parm = vo_parm_table[mode];
			parm->dev_ops = 0; // dev;
		}
	} while(1);

	if( mode == VOUT_MODE_MAX ){
		mode = VOUT_EXTDEV_DEFAULT_MODE;
		parm = vo_parm_table[mode];
		parm->dev_ops = 0; // dev;
	}

	vout_register(mode,parm);
	printf("[VOUT] ext dev : %s\n",(parm)? parm->name:"NO");
	return mode;
}

#define VOUT_INTDEV_DEFAULT_MODE	VOUT_VGA	// default internal vout (VGA)
vout_mode_t vout_intdev_probe(void)
{
	vout_mode_t mode = VOUT_MODE_MAX;

#ifdef WMT_FTBLK_VOUT_VGA
	vout_register(VOUT_VGA,&vo_vga_parm);
	mode = VOUT_VGA;
#endif
#ifdef WMT_FTBLK_VOUT_SDA
	vout_register(VOUT_SD_ANALOG,&vo_sda_parm);
#endif
#ifdef WMT_FTBLK_VOUT_LVDS
	vout_register(VOUT_LVDS,&vo_lvds_parm);
#endif
#ifdef WMT_FTBLK_VOUT_HDMI
	vout_register(VOUT_HDMI,&vo_hdmi_parm);
#endif
	printf("[VOUT] int dev : %s\n",(mode != VOUT_MODE_MAX)? vo_parm_table[mode]->name:"NO");
	return mode;
}

int vout_exit(void)
{
	vout_set_blank(VOUT_MODE_ALL,1);
	govrh_set_MIF_enable(VPP_FLAG_DISABLE);
	return 0;
}
