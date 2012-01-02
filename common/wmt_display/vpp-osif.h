/*++ 
 * linux/drivers/video/wmt/vpp-osif.h
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

#ifndef VPP_OSIF_H
#define VPP_OSIF_H

/*-------------------- DEPENDENCY -------------------------------------*/

/*	following is the C++ header	*/
#ifdef	__cplusplus
extern	"C" {
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/

#define REG32_VAL(addr) (*(volatile unsigned int *)(addr))
#define REG16_VAL(addr) (*(volatile unsigned short *)(addr))
#define REG8_VAL(addr)  (*(volatile unsigned char *)(addr))

#define U32 unsigned int
#define U16 unsigned short
#define U8 unsigned char

/*
#define kmalloc(a,b) 	malloc(a)
#define kfree(a)		free(a)
#define GFP_KERNEL		0
*/
#define module_init(a)


/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  hdmi_xxx_t;  *//*Example*/

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef VPP_OSIF_C
#define EXTERN
#else
#define EXTERN   extern
#endif /* ifdef VPP_OSIF_C */

/* EXTERN int      hdmi_xxx; *//*Example*/

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
/* #define HDMI_XXX_YYY   xxxx *//*Example*/
#ifdef DEBUG
#define DBGMSG(fmt, args...)  DPRINT("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...) do {} while(0)
#endif

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  hdmi_xxx(void); *//*Example*/

#ifdef	__cplusplus
}
#endif
#endif //VPP_OSIF_H

