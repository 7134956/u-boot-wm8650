/*++ 
 * linux/drivers/video/wmt/govm.c
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

#define HDMI_C
// #define DEBUG
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "hdmi.h"
#include "vout.h"
#if 0
#include "../../crypto/wmt-cipher-comm.h"
#include "../../crypto/wmt-cipher-core.h"
#include "../../crypto/wmt-cipher.h"
#endif

/*----------------------- PRIVATE MACRO --------------------------------------*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define HDMI_XXXX    1     *//*Example*/
//#define CONFIG_HDMI_INFOFRAME_DISABLE
//#define CONFIG_HDMI_EDID_DISABLE
#define CONFIG_HDMI_HDCP_DISABLE

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx hdmi_xxx_t; *//*Example*/
typedef enum {
	HDMI_FIFO_SLOT_AVI = 0,
	HDMI_FIFO_SLOT_AUDIO = 1,
	HDMI_FIFO_SLOT_CONTROL = 2,
	HDMI_FIFO_SLOT_MAX = 15
} hdmi_fifo_slot_t;

/*----------EXPORTED PRIVATE VARIABLES are defined in hdmi.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  hdmi_xxx;        *//*Example*/

char hdmi_hdcp_key[320];
char hdmi_hdcp_decode[328] __attribute__((aligned(4)));

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void hdmi_xxx(void); *//*Example*/
#ifndef CONFIG_HDMI_HDCP_DISABLE
extern int Cypher_Action( IN OUT cypher_base_cfg_t *cypher_base );
extern int Cypher_Action_core( IN OUT cypher_base_cfg_t *cypher_base );	
#endif

/*----------------------- Function Body --------------------------------------*/
/*---------------------------- HDMI COMMON API -------------------------------*/
unsigned char hdmi_ecc(unsigned char *buf,int bit_cnt)
{
	#define HDMI_CRC_LEN	9
	
	int crc[HDMI_CRC_LEN],crc_o[HDMI_CRC_LEN];
	int i,j;
	int input,result,result_rev = 0;

	for(i=0;i<HDMI_CRC_LEN;i++){
		crc[i] = 0;
	}

	for(i=0;i<bit_cnt;i++){
		for(j=0;j<HDMI_CRC_LEN;j++){
			crc_o[j] = crc[j];
		}
		input = (buf[i/8] & (1<<(i%8)))? 1:0;
		crc[0] = crc_o[7] ^ input;
		crc[1] = crc_o[0];
		crc[2] = crc_o[1];
		crc[3] = crc_o[2];
		crc[4] = crc_o[3];
		crc[5] = crc_o[4];
		crc[6] = crc_o[5] ^ crc_o[7] ^ input;
		crc[7] = crc_o[6] ^ crc_o[7] ^ input;
		crc[8] = crc_o[7];

		result     = 0;
		result_rev = 0;
		for (j=0;j<HDMI_CRC_LEN-1;j++){
			result     += (crc[j]<<j);
			result_rev += (crc[j]<<(HDMI_CRC_LEN-2-j));
		}
	}

//	DPRINT("[HDMI] crc 0x%x, %x %x %x %x %x %x %x\n",result_rev,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
	
	return result_rev;	
}

unsigned char hdmi_checksum(unsigned char *header,unsigned char *buf,int cnt)
{
	unsigned char sum;
	int i;

	for(i=0,sum=0;i<cnt;i++){
		sum += buf[i];
	}
	for(i=0;i<3;i++){
		sum += header[i];
	}
	return (0 - sum);
}

#ifdef WMT_FTBLK_HDMI
/*---------------------------- HDMI HAL --------------------------------------*/
void hdmi_set_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_ENABLE,enable);
}

void hdmi_set_dvi_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_DVI_MODE_ENABLE,enable);
}

void hdmi_set_DHCP_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_HDCP_ENABLE,enable);
}

void hdmi_set_sync_low_active(vpp_flag_t hsync,vpp_flag_t vsync)
{
	vppif_reg32_write(HDMI_HSYNC_LOW_ACTIVE,hsync);
	vppif_reg32_write(HDMI_VSYNC_LOW_ACTIVE,vsync);
}

void hdmi_set_output_colfmt(vdo_color_fmt colfmt)
{
	unsigned int val;

	switch(colfmt){
		default:
		case VDO_COL_FMT_ARGB:
			val = 0;
			break;
		case VDO_COL_FMT_YUV444:
			val = 1;
			break;
		case VDO_COL_FMT_YUV422H:
		case VDO_COL_FMT_YUV422V:
			val = 2;
			break;
	}
	vppif_reg32_write(HDMI_CONVERT_YUV422,(val==2)?1:0);
	vppif_reg32_write(HDMI_OUTPUT_FORMAT,val);
}

vdo_color_fmt hdmi_get_output_colfmt(void)
{
	unsigned int val;

	val = vppif_reg32_read(HDMI_OUTPUT_FORMAT);
	switch(val){
		default:
		case 0: return VDO_COL_FMT_ARGB;
		case 1: return VDO_COL_FMT_YUV444;
		case 2: return VDO_COL_FMT_YUV422H;
	}
	return VDO_COL_FMT_ARGB;
}

int hdmi_get_plugin(void)
{
	return vppif_reg32_read(HDMI_HOTPLUG_IN);
}

void hdmi_clear_plug_status(void)
{
	vppif_reg32_write(HDMI_HOTPLUG_IN_STS,1);
	vppif_reg32_write(HDMI_HOTPLUG_OUT_STS,1);	
}

void hdmi_enable_plugin(int enable)
{
	vppif_reg32_write(HDMI_HOTPLUG_OUT_INT,enable);
	vppif_reg32_write(HDMI_HOTPLUG_IN_INT,enable);
}

void hdmi_write_fifo(hdmi_fifo_slot_t no,unsigned int *buf,int cnt)
{
	int i;

	if( no > HDMI_FIFO_SLOT_MAX ) return;
#ifdef DEBUG
{
	char *ptr;

	DPRINT("[HDMI] AVI info package %d,cnt %d",no,cnt);
	ptr = buf;
	for(i=0;i<cnt;i++){
		if( (i % 4)==0 ) DPRINT("\n %02d :",i);
		DPRINT(" 0x%02x",ptr[i]);
	}
	DPRINT("\n[HDMI] AVI info package end\n");	
}	
#endif

	vppif_reg32_out(REG_HDMI_FIFO_CTRL,(no << 8));
	cnt = (cnt+3)/4;
	for(i=0;i<cnt;i++){
		vppif_reg32_out(REG_HDMI_WR_FIFO_ADDR+4*i,buf[i]);
	}
	vppif_reg32_write(HDMI_INFOFRAME_WR_STROBE,1);
}

void hdmi_read_fifo(hdmi_fifo_slot_t no,unsigned int *buf,int cnt)
{
	int i;
	
	if( no > HDMI_FIFO_SLOT_MAX ) return;
	
	vppif_reg32_out(REG_HDMI_FIFO_CTRL,(no << 8));	
	vppif_reg32_write(HDMI_INFOFRAME_RD_STROBE,1);
	for(i=0;i<cnt;i++){
		buf[i] = vppif_reg32_in(REG_HDMI_RD_FIFO_ADDR+4*i);
	}
}

#define HDMI_STATUS_START		BIT16
#define HDMI_STATUS_STOP		BIT17
#define HDMI_STATUS_WR_AVAIL	BIT18
#define HDMI_STATUS_HDCP_USE	BIT19
#define HDMI_STATUS_SW_READ		BIT25
int hdmi_DDC_check_status(unsigned int checkbits,int condition)
{
    int status = 1;
    unsigned int i = 0, maxloop = 50;

#if 1
	if( condition ){
		while((vppif_reg32_in(REG_HDMI_I2C_CTRL2) & checkbits) && (i<maxloop)){
			udelay(20); // delay
            if(++i == maxloop) status = 0;
		}
	}
	else {
		while(!(vppif_reg32_in(REG_HDMI_I2C_CTRL2) & checkbits) && (i<maxloop)){
			udelay(20); // delay
            if(++i == maxloop) status = 0;
		}
	}
#else
    if(condition){
        while((REG32_VAL(0xC0B8) & checkbits) && i < maxloop){
			udelay(20);
            if(++i == maxloop) status = FALSE;
        }
    }
    else {
        while(!(REG32_VAL(0xC0B8) & checkbits) && i < maxloop){
            udelay(20);
            if(++i == maxloop) status = FALSE;
        }
    }
#endif	
    return status;
}

void hdmi_DDC_set_freq(unsigned int hz)
{
	unsigned int div;
	
	div = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / hz;
	vppif_reg32_write(HDMI_I2C_CLK_DIVIDER,div);
}

int hdmi_DDC_read(char addr,char index,char *buf,int length)
{
    int status = 1;
#if 1
    unsigned int i = 0;

#ifdef CONFIG_HDMI_EDID_DISABLE
	return status;
#endif

	DPRINT("[HDMI] read DDC\n");

//	hdmi_DDC_set_freq(50000);								// I2C freq 50KHz
	vppif_reg32_write(HDMI_HDCP_ENABLE,1);
	vppif_reg32_write(HDMI_I2C_ENABLE,1);

	// START
	vppif_reg32_write(HDMI_SW_START_REQ,1);					// sw start
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	if( status )
		hdmi_DDC_check_status(HDMI_STATUS_START+HDMI_STATUS_WR_AVAIL, 1);		// wait start & wr data avail

	// Slave address
	vppif_reg32_write(HDMI_WR_DATA,addr);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	if( status )
		hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);		// wait wr data avail

	// Offset
	vppif_reg32_write(HDMI_WR_DATA,index);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	if( status )
		hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);		// wait wr data avail

    // START
	vppif_reg32_write(HDMI_SW_START_REQ,1);					// sw start
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	if( status )
		hdmi_DDC_check_status(HDMI_STATUS_START+HDMI_STATUS_WR_AVAIL, 1);		// wait start & wr data avail

    // Slave Address + 1
	vppif_reg32_write(HDMI_WR_DATA,addr+1);
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
	if( status )
		hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);		// wait wr data avail

    // Read Data
    for(i = 0; i < length; i++)
    {
		vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);			// write data avail
		hdmi_DDC_check_status(HDMI_STATUS_WR_AVAIL, 1);		// wait wr data avail
		hdmi_DDC_check_status(HDMI_STATUS_SW_READ, 0);		// wait sw read not set
        *buf++ = vppif_reg32_read(HDMI_RD_DATA);
		vppif_reg32_write(HDMI_SW_READ,0);					// sw read from HDCP
    }

    // STOP
	vppif_reg32_write(HDMI_SW_STOP_REQ,1);					// sw stop
	vppif_reg32_write(HDMI_WR_DATA_AVAIL,1);				// write data avail
    if(status)
		hdmi_DDC_check_status(HDMI_STATUS_STOP+HDMI_STATUS_WR_AVAIL+HDMI_STATUS_HDCP_USE, 1);	// wait start & wr data avail

	vppif_reg32_write(HDMI_I2C_ENABLE,0);
#else
    unsigned int pMMIO = 0, i = 0, temp = 0, ddata = 0;
	char ckm = 0, data = 0;

	REG32_VAL(0xC000) = REG32_VAL(0xC000) | 0x1;	// enable HDCP
	REG32_VAL(0xC0C4) = (REG32_VAL(0xC0C4) & 0xFC7FFFFF) | 0x800000;	// I2C freq 50KHz
	REG32_VAL(0xC0B8) = 0x1;	// I2C enable

    // START
    REG32_VAL(0xC0B8) = 0x11;	// I2C enable, sw start
    REG32_VAL(0xC0B8) = 0x19;	// I2C enable, sw start, write data avail
    if(status)
	    status = hdmi_check_DDC_status(0x0018, TRUE);	// wait start & wr data avail

    // Slave Address
    temp = addr;
    temp <<= 16;
    temp |= REG32_VAL(0xC0B4) & 0xFF00FFFF;
    REG32_VAL(0xC0B4) = temp;	// write slave address
    REG32_VAL(0xC0B8, 0x0009);	// I2C enable, write data avail
    if(status)
        status = hdmi_check_DDC_status(0x0008, TRUE);	// wait wr data avail

    // Offset
    temp = index;
    temp <<= 16;
    temp |= REG32_VAL(0xC0B4) & 0xFF00FFFF;
    REG32_VAL(0xC0B4) = temp;	// write index
    REG32_VAL(0xC0B8, 0x0009);	// I2C enable, write data avail
    if(status)
        status = hdmi_check_DDC_status(0x0008, TRUE);	// wait wr data avail

    // START
    REG32_VAL(0xC0B8) = 0x11;	// I2C enable, sw start
    REG32_VAL(0xC0B8) = 0x19;	// I2C enable, sw start, write data avail
    if(status)
        status = hdmi_check_DDC_status(0x0018, TRUE);	// wait start & wr data avail

    // Slave Address + 1
    temp = addr + 1;
    temp <<= 16;
    temp |= REG32_VAL(0xC0B4) & 0xFF00FFFF;
    REG32_VAL(0xC0B4) = temp;	// write slave address
    REG32_VAL(0xC0B8) = 0x0009;	// I2C enable, write data avail
    if(status)
        status = hdmi_check_DDC_status(0x0008, TRUE);	// wait wr data avail

    // Read Data
    for(j = 0; j < length; j++)
    {
        REG32_VAL(0xC0B8) = 0x0009;	// I2C enable, write data avail
        hdmi_check_DDC_status(0x0008, TRUE);	// wait wr data avail
        hdmi_check_DDC_status(0x0080, FALSE);	// wait sw read not set

        // correct EDID check sum
        if(j%128 == 127)
        {
            *buf++ = (0x100 - ckm);
			ckm = 0;
        }
        else
        {
        	ckm += (BYTE)((REG32_VAL(0xC0B4) & 0x0000FF00) >> 8);
            *buf++ = (BYTE)((REG32_VAL(0xC0B4) & 0x0000FF00) >> 8);
        }
        REG32_VAL(0xC0B8) = (REG32_VAL(0xC0B8) & ~0x80));	// sw read from HDCP
    }

    // STOP
    REG32_VAL(0xC0B8) = 0x21;	// I2C enable, sw stop
    REG32_VAL(0xC0B8) = 0x29;	// I2C enable, sw stop, write data avail
    if(status)
        status = hdmi_check_DDC_status(0x0828, TRUE);	// wait sw stop, wr data avail, HDCP not use i2c
#endif
    return status;
}

void hdmi_audio_enable(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_AUD_ENABLE,enable);
}

void hdmi_audio_mute(vpp_flag_t enable)
{
	vppif_reg32_write(HDMI_AUD_MUTE,enable);
}

/*----------------------- HDMI API --------------------------------------*/
void hdmi_write_packet(unsigned int header,unsigned char *packet,int cnt)
{
	unsigned char buf[36];
	int i;
	hdmi_fifo_slot_t no;

#ifdef CONFIG_HDMI_INFOFRAME_DISABLE
	return;
#endif
	memcpy(&buf[0],&header,3);
	buf[3] = hdmi_ecc((unsigned char *)&header,24);
	for( i=0; i<cnt/7; i++ ){
		memcpy(&buf[4+8*i],&packet[7*i],7);
		buf[11+8*i] = hdmi_ecc(&packet[7*i],56);
	}

	switch(header & 0xFF){
		case HDMI_PACKET_INFOFRAME_AVI:
			no = HDMI_FIFO_SLOT_AVI;
			break;
		case HDMI_PACKET_INFOFRAME_AUDIO:
			no = HDMI_FIFO_SLOT_AUDIO;
			break;
		default:
			no = HDMI_FIFO_SLOT_CONTROL;
			break;
	}
	hdmi_write_fifo(no,(unsigned int *)buf,(4+8*(cnt/7)));
}

void hdmi_tx_null_packet(void)
{
	hdmi_write_packet(HDMI_PACKET_NULL,0,0);
}

void hdmi_tx_general_control_packet(int mute)
{
	unsigned char buf[7];
	memset(buf,0x0,7);
	buf[0] = (mute)? 0x01:0x10;
	buf[1] = HDMI_COLOR_DEPTH_24 | ( HDMI_PHASE_4 << 4);
	hdmi_write_packet(HDMI_PACKET_GENERAL_CTRL,buf,7);
}

void hdmi_tx_avi_infoframe_packet(vdo_color_fmt colfmt,hdmi_video_code_t vic)
{
	unsigned int header;
	unsigned char buf[28];
	unsigned char temp;

	memset(buf,0x0,28);
	header = HDMI_PACKET_INFOFRAME_AVI + (0x2 << 8) + (0x0d << 16);
	buf[1] = HDMI_SI_NO_DATA + (HDMI_BI_V_H_VALID << 2) + (HDMI_AF_INFO_NO_DATA << 4);
	switch( colfmt ){
		case VDO_COL_FMT_YUV422H:
		case VDO_COL_FMT_YUV422V:
			temp = HDMI_OUTPUT_YUV422;
			break;
		case VDO_COL_FMT_YUV444:
			temp = HDMI_OUTPUT_YUV444;
			break;
		case VDO_COL_FMT_ARGB:
		default:
			temp = HDMI_OUTPUT_RGB;
			break;
	}
	buf[1] += (temp << 5);
	buf[2] = HDMI_ASPECT_RATIO_PIC + (HDMI_PIC_ASPECT_16_9 << 4) + (HDMI_COLORIMETRY_ITU709 << 6);
	buf[3] = 0x84;	
	buf[4] = vic;
	buf[5] = HDMI_PIXEL_REP_NO;
	buf[0] = hdmi_checksum((unsigned char *)&header,buf,28);
	hdmi_write_packet(header,buf,28);
}

void hdmi_tx_audio_infoframe_packet(int channel,int freq)
{
	unsigned int header;
	unsigned char buf[28];

	memset(buf,0x0,28);
	header = HDMI_PACKET_INFOFRAME_AUDIO + (0x1 << 8) + (0x0a << 16);
	buf[1] = channel + (HDMI_AUD_TYPE_REF_STM << 4);
	buf[2] = 0x0;	// HDMI_AUD_SAMPLE_24 + (freq << 2);
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x0;	// 0 db
	buf[0] = hdmi_checksum((unsigned char *)&header,buf,28);
	hdmi_write_packet(header,buf,28);
}

void hdmi_config_audio(vout_audio_t *info)
{
	unsigned int freq;

	hdmi_tx_audio_infoframe_packet(info->channel-1,info->sample_rate);
	hdmi_audio_enable(VPP_FLAG_DISABLE);
	vppif_reg32_out(REG_HDMI_AUD_MODE,0);
	vppif_reg32_write(HDMI_AUD_LAYOUT,(info->channel==8)? 1:0);

	switch(info->sample_rate){
		case 32000: freq = 0x3; break;
		case 44100: freq = 0x0; break;
		case 88200: freq = 0x8; break;
		case 176400: freq = 0xC; break;
		default:
		case 48000: freq = 0x2; break;
		case 96000: freq = 0xA; break;
		case 192000: freq = 0xE; break;
		case 768000: freq = 0x9; break;
	}
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS0,(freq << 24) + 0x4);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS1,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS2,0xb);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS3,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS4,0x0);
	vppif_reg32_out(REG_HDMI_AUD_CHAN_STATUS5,0x0);
	
	vppif_reg32_write(HDMI_AUD_ACR_ENABLE,VPP_FLAG_ENABLE);	// 
	vppif_reg32_write(HDMI_AUD_N_20BITS,6144);
#if 0	// auto detect CTS
	vppif_reg32_write(HDMI_AUD_CTS_SELECT,0);				
#else
	{
		unsigned int cts;

		cts = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / 1000;
		vppif_reg32_write(HDMI_AUD_CTS_LOW_12BITS,cts & 0xFFF);
		vppif_reg32_write(HDMI_AUD_CTS_HI_8BITS,(cts & 0xFF000)>>12);
		vppif_reg32_write(HDMI_AUD_CTS_SELECT,1);
 		vppif_reg32_write(HDMI_AUD_ACR_RATIO,cts);
		DBGMSG("[HDMI] CTS %d\n",cts);
	}
#endif
	vppif_reg32_write(HDMI_AUD_AIPCLK_RATE,0);
	vppif_reg32_out(0xD80Ed98c,0x76543210);
	hdmi_audio_enable(VPP_FLAG_ENABLE);
}

void hdmi_config_video(hdmi_info_t *info)
{
	hdmi_set_output_colfmt(info->outfmt);
	hdmi_tx_avi_infoframe_packet(info->outfmt,info->vic);
}

void hdmi_config(hdmi_info_t *info)
{
	vout_audio_t audio_info;
	unsigned int h_porch;

	vppif_reg32_write(HDMI_INFOFRAME_SELECT,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,0);
	hdmi_config_video(info);
	
	h_porch = vppif_reg32_read(GOVRH_ACTPX_BG);
	vppif_reg32_write(HDMI_HORIZ_BLANK_MAX_PCK,((h_porch-86)/32)-1);
	DBGMSG("[HDMI] h porch %d\n",h_porch);

	audio_info.fmt = 16;
	audio_info.channel = info->channel;
	audio_info.sample_rate = info->freq;
	hdmi_config_audio(&audio_info);

	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_ADDR,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_LEN,1);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,1);
}

/*----------------------- Module API --------------------------------------*/
int hdmi_check_plugin(void)
{
	int plugin;

	hdmi_clear_plug_status();
	plugin = hdmi_get_plugin();
	DPRINT("[HDMI] HDMI plug%s\n",(plugin)?"in":"out");
	hdmi_set_enable(plugin);	
	if( plugin ){
	}
	else {
	}
	return plugin;
}

void hdmi_reg_dump(void)
{
	int i;

	DPRINT("========== HDMI register dump ==========\n");
	for(i=0;i<0x18;i+=16){
		DPRINT("0x%8x : 0x%08x 0x%08x 0x%08x 0x%08x\n",HDMI_BASE_ADDR+i,vppif_reg32_in(HDMI_BASE_ADDR+i),
			vppif_reg32_in(HDMI_BASE_ADDR+i+4),vppif_reg32_in(HDMI_BASE_ADDR+i+8),vppif_reg32_in(HDMI_BASE_ADDR+i+12));
	}
	
	DPRINT("---------- HDMI common ----------\n");
	DPRINT("enable %d,reset %d,dvi %d\n",vppif_reg32_read(HDMI_ENABLE),vppif_reg32_read(HDMI_RESET),vppif_reg32_read(HDMI_DVI_MODE_ENABLE));
	DPRINT("colfmt %d,conv 422 %d,hsync low %d,vsync low %d\n",vppif_reg32_read(HDMI_OUTPUT_FORMAT),vppif_reg32_read(HDMI_CONVERT_YUV422),vppif_reg32_read(HDMI_HSYNC_LOW_ACTIVE),vppif_reg32_read(HDMI_VSYNC_LOW_ACTIVE));
	DPRINT("dbg bus sel %d,state mach %d\n",vppif_reg32_read(HDMI_DBG_BUS_SELECT),vppif_reg32_read(HDMI_STATE_MACHINE_STATUS));	
	DPRINT("eep reset %d,encode %d,eess %d\n",vppif_reg32_read(HDMI_EEPROM_RESET),vppif_reg32_read(HDMI_ENCODE_ENABLE),vppif_reg32_read(HDMI_EESS_ENABLE));
	DPRINT("verify pj %d,auth test %d,cipher %d\n",vppif_reg32_read(HDMI_VERIFY_PJ_ENABLE),vppif_reg32_read(HDMI_AUTH_TEST_KEY),vppif_reg32_read(HDMI_CIPHER_1_1));	
	DPRINT("preamble %d\n",vppif_reg32_read(HDMI_PREAMBLE));	
	DPRINT("wdt divid %d\n",vppif_reg32_read(HDMI_WDT_DIVIDER));	

	DPRINT("---------- HDMI hotplug ----------\n");
	DPRINT("plug %s\n",vppif_reg32_read(HDMI_HOTPLUG_IN)? "in":"out");
	DPRINT("plug in enable %d, status %d\n",vppif_reg32_read(HDMI_HOTPLUG_IN_INT),vppif_reg32_read(HDMI_HOTPLUG_IN_STS));
	DPRINT("plug out enable %d, status %d\n",vppif_reg32_read(HDMI_HOTPLUG_OUT_INT),vppif_reg32_read(HDMI_HOTPLUG_OUT_STS));
	DPRINT("debounce detect %d,sample %d\n",vppif_reg32_read(HDMI_DEBOUNCE_DETECT),vppif_reg32_read(HDMI_DEBOUNCE_SAMPLE));

	DPRINT("---------- HDCP ----------\n");
	DPRINT("enable %d,key encode %d,key path %d\n",vppif_reg32_read(HDMI_HDCP_ENABLE),vppif_reg32_read(HDMI_HDCP_KEY_ENCODE),vppif_reg32_read(HDMI_CPU_WR_HDCP_KEY_PATH));
	DPRINT("key req %d,key read %d,key last %d\n",vppif_reg32_read(HDMI_HDCP_KEY_REQ),vppif_reg32_read(HDMI_HDCP_KEY_READ),vppif_reg32_read(HDMI_HDCP_KEY_LAST));
	DPRINT("src sel %d,ksv list avail %d,ksv ver done %d\n",vppif_reg32_read(HDMI_HDCP_SRC_SEL),vppif_reg32_read(HDMI_KSV_LIST_AVAIL),vppif_reg32_read(HDMI_KSV_VERIFY_DONE));
	DPRINT("delay %d\n",vppif_reg32_read(HDMI_HDCP_DELAY));
	DPRINT("key shuffle mode %d,key debug mode %d\n",vppif_reg32_read(HDMI_HDCP_KEY_SHUFFLE_MODE),vppif_reg32_read(HDMI_HDCP_KEY_DEBUG_MODE));
	DPRINT("TMDS test format %d,enable %d\n",vppif_reg32_read(HDMI_TMDS_TST_FORMAT),vppif_reg32_read(HDMI_TMDS_TST_ENABLE));

	DPRINT("---------- I2C ----------\n");
	DPRINT("enable %d,exit FSM %d,key read %d\n",vppif_reg32_read(HDMI_I2C_ENABLE),vppif_reg32_read(HDMI_FORCE_EXIT_FSM),vppif_reg32_read(HDMI_KEY_READ_WORD));
	DPRINT("clk divid %d,rd data 0x%x,wr data 0x%x\n",vppif_reg32_read(HDMI_I2C_CLK_DIVIDER),vppif_reg32_read(HDMI_RD_DATA),vppif_reg32_read(HDMI_WR_DATA));
	DPRINT("start %d,stop %d,wr avail %d\n",vppif_reg32_read(HDMI_SW_START_REQ),vppif_reg32_read(HDMI_SW_STOP_REQ),vppif_reg32_read(HDMI_WR_DATA_AVAIL));
	DPRINT("status %d,sw read %d,sw i2c req %d\n",vppif_reg32_read(HDMI_I2C_STATUS),vppif_reg32_read(HDMI_SW_READ),vppif_reg32_read(HDMI_SW_I2C_REQ));

	DPRINT("---------- AUDIO ----------\n");
	DPRINT("enable %d,sub pck %d,spflat %d\n",vppif_reg32_read(HDMI_AUD_ENABLE),vppif_reg32_read(HDMI_AUD_SUB_PACKET),vppif_reg32_read(HDMI_AUD_SPFLAT));	
	DPRINT("aud pck insert reset %d,enable %d,delay %d\n",vppif_reg32_read(HDMI_AUD_PCK_INSERT_RESET),vppif_reg32_read(HDMI_AUD_PCK_INSERT_ENABLE),vppif_reg32_read(HDMI_AUD_INSERT_DELAY));
	DPRINT("avmute set %d,clr %d,pixel repete %d\n",vppif_reg32_read(HDMI_AVMUTE_SET_ENABLE),vppif_reg32_read(HDMI_AVMUTE_CLR_ENABLE),vppif_reg32_read(HDMI_AUD_PIXEL_REPETITION));
	DPRINT("acr ratio %d,acr enable %d,mute %d\n",vppif_reg32_read(HDMI_AUD_ACR_RATIO),vppif_reg32_read(HDMI_AUD_ACR_ENABLE),vppif_reg32_read(HDMI_AUD_MUTE));	
	DPRINT("layout %d,pwr save %d,n 20bits %d\n",vppif_reg32_read(HDMI_AUD_LAYOUT),vppif_reg32_read(HDMI_AUD_PWR_SAVING),vppif_reg32_read(HDMI_AUD_N_20BITS));
	DPRINT("cts low 12 %d,hi 8 %d,cts sel %d\n",vppif_reg32_read(HDMI_AUD_CTS_LOW_12BITS),vppif_reg32_read(HDMI_AUD_CTS_HI_8BITS),vppif_reg32_read(HDMI_AUD_CTS_SELECT));
	DPRINT("aipclk rate %d\n",vppif_reg32_read(HDMI_AUD_AIPCLK_RATE));

	DPRINT("---------- INFOFRAME ----------\n");
	DPRINT("sel %d,hor blank pck %d\n",vppif_reg32_read(HDMI_INFOFRAME_SELECT),vppif_reg32_read(HDMI_HORIZ_BLANK_MAX_PCK));
	DPRINT("fifo1 ready %d,addr 0x%x,len %d\n",vppif_reg32_read(HDMI_INFOFRAME_FIFO1_RDY),vppif_reg32_read(HDMI_INFOFRAME_FIFO1_ADDR),vppif_reg32_read(HDMI_INFOFRAME_FIFO1_LEN));
	DPRINT("fifo2 ready %d,addr 0x%x,len %d\n",vppif_reg32_read(HDMI_INFOFRAME_FIFO2_RDY),vppif_reg32_read(HDMI_INFOFRAME_FIFO2_ADDR),vppif_reg32_read(HDMI_INFOFRAME_FIFO2_LEN));
	DPRINT("wr strobe %d,rd strobe %d,fifo addr %d\n",vppif_reg32_read(HDMI_INFOFRAME_WR_STROBE),vppif_reg32_read(HDMI_INFOFRAME_RD_STROBE),vppif_reg32_read(HDMI_INFOFRAME_FIFO_ADDR));

	DPRINT("---------- HDMI test ----------\n");
	DPRINT("ch0 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH0_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH0_TEST_DATA));
	DPRINT("ch1 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH1_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH1_TEST_DATA));
	DPRINT("ch2 enable %d, data 0x%x\n",vppif_reg32_read(HDMI_CH2_TEST_MODE_ENABLE),vppif_reg32_read(HDMI_CH2_TEST_DATA));
}

#ifdef CONFIG_PM
static unsigned int *hdmi_pm_bk;
static unsigned int hdmi_pm_enable;
void hdmi_suspend(int sts)
{
	switch( sts ){
		case 0:	// disable module
			hdmi_pm_enable = vppif_reg32_read(HDMI_ENABLE);
			vppif_reg32_write(HDMI_ENABLE,0);
			break;
		case 1: // disable tg
			break;
		case 2:	// backup register
			hdmi_pm_bk = vpp_backup_reg(HDMI_BASE_ADDR+0x120,0x420); /* 0x120 - 0x420 */
			break;
		default:
			break;
	}
}

void hdmi_resume(int sts)
{
	switch( sts ){
		case 0:	// restore register
			vpp_restore_reg(HDMI_BASE_ADDR+0x120,0x420,hdmi_pm_bk); /* 0x120 - 0x420 */
			hdmi_pm_bk = 0;
			break;
		case 1:	// enable module
			vppif_reg32_write(HDMI_ENABLE,0);
			break;
		case 2: // enable tg
			break;
		default:
			break;
	}
}
#else
#define hdmi_suspend NULL
#define hdmi_resume NULL
#endif

void hdmi_read_key(void)
{
	int i;
	unsigned int key[2];

	DPRINT("[HDMI] read key\n");

	// clear read address
	vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x200000);	// DRV_HDCP_KEY_READ
	vppif_reg32_out(HDMI_BASE_ADDR + 0x124, 0x45500);	// DRV_HDCP_KEY_READ
	vppif_reg32_out(HDMI_BASE_ADDR + 0x120, 0x94ad);	// DRV_HDCP_KEY_READ
	udelay(20);
	vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x400000);	// DRV_HDCP_KEY_READ
	udelay(20);

	// read date	
	for(i=0;i<41;i++){
		vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x200000);		// DRV_HDCP_KEY_READ
		udelay(20);

		if( i == 0 )
			vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x400000);
		vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x100000);      // DRV_HDCP_KEY_LAST_T, DRV_HDCP_KEY_REQ_T
		while((vppif_reg32_in(HDMI_BASE_ADDR + 0x128) & 0x200000) == 0 );
		key[0] = vppif_reg32_in(HDMI_BASE_ADDR + 0x148);        // Key read out 
		key[1] = vppif_reg32_in(HDMI_BASE_ADDR + 0x14C);        // Key read out 
		DPRINT("%d : 0x%08x 0x%08x\n",i,key[0],key[1]);
	}
}

void hdmi_write_key(unsigned int *buf)
{
	int i;
	unsigned int key[2];

	DPRINT("[HDMI] write key\n");

	// clear write address
	vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x200000);	// DRV_HDCP_KEY_READ
	vppif_reg32_out(HDMI_BASE_ADDR + 0x124, 0x45540);	// DRV_HDCP_KEY_READ
	vppif_reg32_out(HDMI_BASE_ADDR + 0x120, 0x94ad);	// DRV_HDCP_KEY_READ
	udelay(20);
	vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x400000);	// DRV_HDCP_KEY_READ
	udelay(20);

	// write date	
	for(i=0;i<41;i++){
		key[0] = buf[2*i]; // 2*i;
		key[1] = buf[2*i+1]; // 2*i+1;
		
		vppif_reg32_out(HDMI_BASE_ADDR + 0x140, key[0]);    // Key read out 
		vppif_reg32_out(HDMI_BASE_ADDR + 0x144, key[1]);    // Key read out 
		udelay(20);
		vppif_reg32_out(HDMI_BASE_ADDR + 0x128, 0x100000);  // DRV_HDCP_KEY_LAST_T, DRV_HDCP_KEY_REQ_T		
		DPRINT("%d : 0x%08x 0x%08x\n",i,key[0],key[1]);
	}
}

void hdmi_hdcp_init(void)
{
	int i;
#if 1	// Demo key
	DPRINT("[HDMI] hdmi_hdcp_init\n");
	vppif_reg32_out(HDMI_BASE_ADDR + 0x3f8, 0x100);

#if 0
{
	unsigned int *ptr;

	ptr = hdmi_hdcp_decode;
	for(i=0;i<82;i++){
		DPRINT("%d : 0x%08x\n",i,ptr[i]);
		vppif_reg32_out(REG_HDMI_HDCP_FIFO_ADDR,ptr[i]);
	}
}

hdmi_read_key();
#endif
hdmi_write_key(hdmi_hdcp_decode);
hdmi_read_key();
#else
	cypher_base_cfg_t cypher;

	DPRINT("[HDMI] hdmi_hdcp_init\n");

	memset(&cypher,0,sizeof(cypher_base_cfg_t));

	cypher.algo_mode = CYPHER_ALGO_AES;
	cypher.input_addr = __pa(hdmi_hdcp_key);
	cypher.output_addr = __pa(hdmi_hdcp_decode);
//	cypher.input_addr = hdmi_hdcp_key;
//	cypher.output_addr = hdmi_hdcp_decode;
	cypher.text_length = 320;
	cypher.dec_enc = CYPHER_DECRYPT;
	cypher.op_mode = CYPHER_OP_ECB_HW_KEY;
	cypher.INC = 0;

	memset(hdmi_hdcp_decode,0,328);
#if 1
	Cypher_Action_core(&cypher);

{
	unsigned int *ptr;

	ptr = hdmi_hdcp_decode;
	for(i=0;i<82;i++){
		vppif_reg32_out(REG_HDMI_HDCP_FIFO_ADDR,ptr[i]);
	}
}
#else
{
	extern u32 *cipher_input_buf_vir;
	extern u32 *cipher_output_buf_vir;
	char *ptr;
	
	ptr = (char *) cipher_input_buf_vir;
	for(i=0;i<320;i++){
		ptr[i] = hdmi_hdcp_key[i];
	}	
	Cypher_Action(&cypher);
	ptr = (char *) cipher_output_buf_vir;
	for(i=0;i<320;i++){
		hdmi_hdcp_decode[i] = ptr[i];
	}
}
#endif

#if 1
	for(i=0;i<320;i++){
		if( (i % 10) == 0 ){
			printk("\n%02d :",i);
		}
		printk(" %02x",hdmi_hdcp_decode[i]);
	}
	printk("\n");
#endif
#endif
	vppif_reg32_write(0xd81104a0,0x3,0,0x0);	// GPIO HDMI clk/data pin pull disable

	vppif_reg32_write(HDMI_I2C_CLK_DIVIDER,0x300);
	vppif_reg32_write(HDMI_WDT_DIVIDER,0xe0000);
	vppif_reg32_write(HDMI_I2C_ENABLE,1);
	vppif_reg32_write(HDMI_EEPROM_RESET,1);
	vppif_reg32_write(HDMI_EESS_ENABLE,1);
	hdmi_set_DHCP_enable(VPP_FLAG_ENABLE);

	vppif_reg32_out(0xd806c14c,0x01000000);
}

void hdmi_init(void)
{
	hdmi_info_t hdmi_info;

	vppif_reg32_write(LVDS_VBG_SEL,2);
	vppif_reg32_write(LVDS_DRV_PDMODE,1);
	vppif_reg32_write(HDMI_HDCP_DELAY,0x40);
	hdmi_set_enable(VPP_FLAG_ENABLE);
	hdmi_set_dvi_enable(VPP_FLAG_DISABLE);
	lvds_set_enable(1);
	lvds_set_enable(0);

	hdmi_info.outfmt = VDO_COL_FMT_ARGB;
	hdmi_info.vic = HDMI_1280x720p60_16x9;
	hdmi_info.channel = 2;
	hdmi_info.freq = 48000;

	hdmi_config(&hdmi_info);
	lvds_set_enable(1);
	lvds_set_enable(0);
	vppif_reg32_out(0xd806c3ec,0x0);
	vppif_reg32_out(0xd806c3e8,0x0);

	vppif_reg32_write(HDMI_INFOFRAME_SELECT,0);
	vppif_reg32_write(HDMI_INFOFRAME_FIFO1_RDY,0);

//	hdmi_hdcp_init();
}
#endif /* WMT_FTBLK_HDMI */

