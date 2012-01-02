/*++ 
 * linux/drivers/video/wmt/lcd.c
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

#define LCD_C
// #define DEBUG
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <common.h>
#include "lcd.h"
#include "vout.h"
#include "hw_devices.h"
#include "wmt-pwm.h"
#include "wmt_display.h"
#include "../../board/wmt/include/wmt_clk.h"
extern int auto_pll_divisor(enum dev_id dev, enum clk_cmd cmd, int unit, int freq);

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  LCD_XXXX  xxxx    *//*Example*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/
typedef struct {
	lcd_parm_t* (*get_parm)(int arg);
} lcd_device_t;

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/
lcd_device_t lcd_device_array[LCD_PANEL_MAX];

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*----------------------- Backlight --------------------------------------*/
#define pwm_write_reg(addr,val,wait)	\
	REG32_VAL(addr) = val;	\
	while(REG32_VAL(0xd8220040)&=wait);

void pwm_set_control(int no,unsigned int ctrl)
{
	unsigned int addr;

	addr = PWM_CTRL_REG_ADDR + (0x10 * no);
	pwm_write_reg(addr,ctrl,PWM_CTRL_UPDATE << (8*no));
} /* End of pwm_proc */

void pwm_set_scalar(int no,unsigned int scalar)
{
	unsigned int addr;

	addr = PWM_SCALAR_REG_ADDR + (0x10 * no);
	pwm_write_reg(addr,scalar,PWM_SCALAR_UPDATE << (8*no));
}

void pwm_set_period(int no,unsigned int period)
{
	unsigned int addr;

	addr = PWM_PERIOD_REG_ADDR + (0x10 * no);
	pwm_write_reg(addr,period,PWM_PERIOD_UPDATE << (8*no));
}

void pwm_set_duty(int no,unsigned int duty)
{
	unsigned int addr;

	addr = PWM_DUTY_REG_ADDR + (0x10 * no);
	pwm_write_reg(addr,duty,PWM_DUTY_UPDATE << (8*no));
}

unsigned int pwm_get_period(int no)
{
	unsigned int addr;

	addr = PWM_PERIOD_REG_ADDR + (0x10 * no);
	return (REG32_VAL(addr) & 0xFFF);
}

unsigned int pwm_get_duty(int no)
{
	unsigned int addr;

	addr = PWM_DUTY_REG_ADDR + (0x10 * no);
	return (REG32_VAL(addr) & 0xFFF);
}

void pwm_set_gpio(int no,int enable)
{
	unsigned int pwm_pin;
	unsigned int reg;

	/* WM3437 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34370000 ){
		pwm_pin = (no==0)? 0x8000000:0x4000000;
		if( no == 0 ){
			if( enable ){
				REG32_VAL(0xd81100dc) |= pwm_pin;
				REG32_VAL(0xd811005c) &= ~0x08000000;	// PWM0 gpio enable
			}
			else {
				REG32_VAL(0xd811005c) |= pwm_pin;	// PWM0 gpio enable
				REG32_VAL(0xd811009c) |= pwm_pin;	// PWM0 OC
				REG32_VAL(0xd81100dc) &= ~pwm_pin;	// PWM0 OD
			}
		}

		if( no == 1 ){
			reg = REG32_VAL(0xd8110200);
			if( enable ){
				reg |= 0x10;
			}
			else {
				reg &= ~0x10;
			}
			REG32_VAL(0xd8110200) = reg;
		}
	}

	/* WM3426 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34260000 ){
		pwm_pin = (no==0)? PWM_GPIO_BIT_0:PWM_GPIO_BIT_1;
		if( enable ){
			REG32_VAL(PWM_GPIO_OD_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_CTRL_REG) &= ~pwm_pin;
		}
		else {
			REG32_VAL(PWM_GPIO_CTRL_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_OC_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_OD_REG) &= ~pwm_pin;
		}
	}

	/* WM3429 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34290000 ){
		pwm_pin = (no==0)? 0x1:0x2;
		if( enable ){
			REG32_VAL(0xd8110074) |= pwm_pin;
			REG32_VAL(0xd8110014) &= ~pwm_pin;
		}
		else {
			REG32_VAL(0xd8110014) |= pwm_pin;
			REG32_VAL(0xd8110044) |= pwm_pin;
			REG32_VAL(0xd8110074) &= ~pwm_pin;
		}

		if( no == 1 ){
			reg = REG32_VAL(0xd8110200);
			if( enable ){
				reg |= 0x100;
			}
			else {
				reg &= ~0x100;
			}
			REG32_VAL(0xd8110200) = reg;
			REG32_VAL(0xd8110618) &= ~0x200;
		}
	}
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34650000 ){
		pwm_pin = (no==0)? 0x20:0x10;
		if( enable ) {
			REG8_VAL(0xd811004B) &= ~pwm_pin;
		} else {
			REG8_VAL(0xd811008B) |= pwm_pin;
			REG8_VAL(0xd81100CB) &= ~pwm_pin;
			REG8_VAL(0xd811004B) |= pwm_pin;
		}
		/* set to PWM mode */
		if( no == 1 )
			REG32_VAL(0xd8110200) |= 0x100;
	}
}

void pwm_set_enable(int no,int enable)
{
	unsigned int addr,reg,reg1;

	addr = PWM_CTRL_REG_ADDR + (0x10 * no);	
	reg = REG32_VAL(addr);
	if( enable ){
		reg |= PWM_ENABLE;
	}
	else {
		reg &= ~PWM_ENABLE;
	}

	pwm_write_reg(addr,reg,PWM_CTRL_UPDATE << (4*no));
	reg1 = REG32_VAL(addr);
	pwm_set_gpio(no,enable);
}

unsigned int pwm_get_enable(int no)
{
	unsigned int addr,reg;

	addr = PWM_CTRL_REG_ADDR + (0x10 * no);
	reg = REG32_VAL(addr);
	reg &= PWM_ENABLE;
	return reg;
}

void pwm_set_freq(int no,unsigned int freq)
{
	unsigned int clock;
	unsigned int reg;
	//unsigned int value;

	if( (REG32_VAL(0xd8130250) & BIT10) == 0 )	// check pwm power on
		return;
#if 0
	reg = REG32_VAL(0xd8130204);
#if(WMT_CUR_PID == WMT_PID_8425)
	clock = 27000000;	/* system freq 27M */
#else
	clock = 25000000;	/* system freq 25M */
#endif
	if( (reg & BIT8) == 0 ){	// Pre-Divisor Bypass
		clock /= 2;
	}


	value = reg & 0x1F;
	if( value >= 4 ){
		value = value * 2;
	}
	else {
		value = 1;
	}
	clock *= value;
#endif
	clock = auto_pll_divisor(DEV_PWM,CLK_ENABLE,0,0);

/*
	reg = REG8_VAL(0xd8130348);
	reg = reg & 0x1F;
	value = (reg==0)? 32:reg;
	clock /= value;
*/
	reg = (clock / freq / PWM_PERIOD_VAL) -1;

	pwm_set_scalar(no,reg);
}

void pwm_set_level(int no,unsigned int level)
{
	unsigned int duty,period;

	if( (REG32_VAL(0xd8130250) & BIT10) == 0 )	// check pwm power on
		return;

	period = PWM_PERIOD_VAL - 1;
	duty = (level * PWM_PERIOD_VAL / 100);
	duty = (duty)? (duty-1):0;

	pwm_set_period(no,period);
	pwm_set_duty(no,duty);
	//pwm_set_control(no,(level)? 0x35:0x8);
	pwm_set_control(no,(level)? 0x34:0x8);
	pwm_set_gpio(no,level);
}

void lcd_blt_enable(int no,int enable)
{
	pwm_set_enable(no,enable);
	return;
}

void lcd_blt_set_pwm(int no, int level, int freq)
{
	int clock = auto_pll_divisor(DEV_PWM,CLK_ENABLE, 0, 0);
	int period, duty, scalar;

	if( (REG32_VAL(0xd8130250) & BIT10) == 0 )	// check pwm power on
		return;

	clock = clock / freq;
	scalar = 0;
	period = 2000;

	while(period > 1023) {
		scalar++;
		period = clock / scalar;
	}

	duty = (period*level)/100;
	duty = (duty)? (duty-1):0;
	scalar = scalar-1;
	period = period -1;

	pwm_set_period(no,period);
	pwm_set_duty(no,duty);
	pwm_set_scalar(no,scalar);
	pwm_set_control(no,(level)? 0x34:0x8);
	pwm_set_gpio(no,level);
}
#if 0
void lcd_blt_set_level(int no,int level)
{
	pwm_set_level(no,level);
	return;
}

void lcd_blt_set_freq(int no,unsigned int freq)
{
	pwm_set_freq(no,freq);
	return;
}
#endif
/*----------------------- LCD --------------------------------------*/
int lcd_panel_register
(
	int no,						/*!<; //[IN] device number */
	void (*get_parm)(int mode)	/*!<; //[IN] get info function pointer */
)
{
	lcd_device_t *p;

	if( no >= LCD_PANEL_MAX ){
		printf("*E* lcd device no max is %d !\n",LCD_PANEL_MAX);
		return -1;
	}

	p = &lcd_device_array[no];
	if( p->get_parm ){
		printf("*E* lcd device %d exist !\n",no);
		return -1;
	}
	p->get_parm = (void *) get_parm;
//	printk("lcd_device_register %d 0x%x\n",no,p->get_parm);
	return 0;
} /* End of lcd_device_register */

lcd_parm_t *lcd_get_parm(lcd_panel_t id,unsigned int arg)
{
	lcd_device_t *p;

	p = &lcd_device_array[id];
	if( p && p->get_parm ){
		return p->get_parm(arg);
	}
	return 0;
}

/*----------------------- vout device plugin --------------------------------------*/
void lcd_set_power_down(int enable)
{
	lcd_blt_enable(lcd_blt_id,!enable);
}

int lcd_set_mode(unsigned int *option)
{
	if( option ){
		if( (p_lcd = lcd_get_parm((lcd_panel_t) option[0],option[1])) ){
			printf("[LCD] %s (id %d,bpp %d)\n",p_lcd->name,option[0],option[1]);
		}
		else {
			printf("[LCD] *E* lcd %d not support\n",option[0]);
			return -1;
		}
		
		if( p_lcd->initial ){
			p_lcd->initial();
		}
		if ((g_display_vaild&PWMDEFMASK) == PWMDEFTP) {
			lcd_blt_set_pwm(g_pwm_setting.pwm_no, g_pwm_setting.duty, g_pwm_setting.period);
		} else {
			// fan : may need to check PWM power ..
			pwm_set_period(g_pwm_setting.pwm_no, g_pwm_setting.period-1);
			pwm_set_duty(g_pwm_setting.pwm_no, g_pwm_setting.duty-1);
			pwm_set_control(g_pwm_setting.pwm_no, (g_pwm_setting.duty-1)? 0x34:0x8);
			pwm_set_gpio(g_pwm_setting.pwm_no, g_pwm_setting.duty-1);
			pwm_set_scalar(g_pwm_setting.pwm_no, g_pwm_setting.scalar-1);
		}

		option[2] = p_lcd->capability;
	}
	else {
		lcd_blt_enable(g_pwm_setting.pwm_no ,0);
		if( p_lcd && p_lcd->uninitial ){
			p_lcd->uninitial();
		}
	}
	return 0;
}

int lcd_check_plugin(void)
{
	return (p_lcd)? 1:0;
}

int lcd_config(vout_info_t *info)
{
	info->resx = p_lcd->timing.hpixel;
	info->resy = p_lcd->timing.vpixel;
	info->timing = p_lcd->timing;

	return 0;
}

int lcd_init(void)
{
	extern vout_t vo_lcd_parm;
	vout_t *parm;
	lcd_panel_t lcd_panel_id;


	lcd_panel_id = vo_get_lcd_id();
	if( lcd_panel_id >= LCD_PANEL_MAX ){
		vout_register(VOUT_LCD,&vo_lcd_parm);
		return -1;
	}

	parm = &vo_lcd_parm;
	parm->option[0] = lcd_panel_id;			/* [LCD] option1 : panel id */
	parm->option[1] = g_display_param.op2;					/* [LCD] option2 : bit per pixel */
	p_lcd = lcd_get_parm(lcd_panel_id,g_display_param.op2);
	if( p_lcd == 0 ) return -1;
	parm->resx = p_lcd->timing.hpixel;
	parm->resy = p_lcd->timing.vpixel;
	parm->pixclk = p_lcd->timing.pixel_clock;
	return 0;
}

vout_dev_ops_t lcd_vout_dev_ops = {
	.mode = VOUT_LCD,

	.init = lcd_init,
	.set_power_down = lcd_set_power_down,
	.set_mode = lcd_set_mode,
	.config = lcd_config,
	.check_plugin = lcd_check_plugin,
//	.get_edid = lcd_get_edid,
};

int lcd_module_init(void)
{	
	extern vout_t *vo_parm_table[VOUT_MODE_MAX];

	vout_device_register(&lcd_vout_dev_ops);
	vo_parm_table[VOUT_LCD]->dev_ops = &lcd_vout_dev_ops;

	vo_set_lcd_id((int)g_display_param.op1);

	lcd_oem_init();
	lcd_lw700at9003_init();
	lcd_at070tn83_init();
	lcd_a080sn01_init();

	return 0;
} /* End of lcd_module_init */
module_init(lcd_module_init);
/*--------------------End of Function Body -----------------------------------*/
#undef LCD_C
