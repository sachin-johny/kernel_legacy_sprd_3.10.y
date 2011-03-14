
/*
 * Copyright (C) 2010 Innofidei Corporation
 * Author:      sean <zhaoguangyu@innofidei.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * 
 */


#ifndef __INNO_CTL_H_
#define __INNO_CTL_H_
#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * ch_config -channel config parameter
 * @ch_id       :channel id, >= 1
 * @start_timeslot: start timeslot for this channel
 * @timeslot_count: timeslot count 
 * @demod_config: 
 */
struct ch_config {
        unsigned char ch_id;
        unsigned char start_timeslot;
        unsigned char timeslot_count;
        unsigned char demod_config;
};

/**
 * sys_status - 
 * @sync :signal sync status, 1 sync, 0 no sync
 * @signal_strength: signal strength
 * @cur_freq: current frequency
 * @ldpc_err_percent: 
 * @rs_err_percent:
 */
struct sys_status {
        unsigned char sync;
        unsigned char signal_strength;
        unsigned char signal_quality;
        unsigned char cur_freq;
        unsigned char ldpc_err_percent;
        unsigned char rs_err_percent;
//add by mahanghong 20110118
	unsigned long err_status;
};

/**
 * err_info 
 */
struct err_info {
        unsigned int ldpc_total_count;
        unsigned int ldpc_error_count;
        unsigned int rs_total_count;
        unsigned int rs_error_count;
        unsigned short BER;       
        unsigned short SNR;       
};

//add by mahanghong 20110118
/**
 * err_status
 */
typedef enum{
	CAS_OK = 0x00,
	NO_MATCHING_CAS = 0x15,
	CARD_OP_ERROR = 0x17,

	MAC_ERR = 0x80,
	GSM_ERR = 0x81,
	KEY_ERR	= 0x82,
	KS_NOT_FIND	= 0x83,
	KEY_NOT_FIND	= 0x84,
	CMD_ERR	= 0x85,
}ERR_STATUS;

/**
 * uam_param - uam transfer parameter
 * @buf_in :in buf 
 * @len_in :len for in buf
 * @buf_out:out buf
 * @len_out:len for out buf
 * @sw : rsponse status
 */
struct uam_param {
        unsigned char *buf_in;
        unsigned int  len_in;
        unsigned char *buf_out;
        unsigned int  *len_out;
        unsigned short sw;
};

struct cmbbms_sk {
        unsigned char ISMACrypSalt[18];
        unsigned char SKLength;
};

struct cmbbms_isma {
        unsigned char MBBMS_ECMDataType;
        struct cmbbms_sk ISMACrypAVSK[2];
};

//add by mahanghong 20110210
struct cmbbms_cw {
		unsigned char KI_INDEX;
		unsigned char CW_DATA[16];
};

//add by mahanghong 20110218
struct aid_3g {
		unsigned char AIDLength;
		unsigned char AID_DATA[16];
};

struct mbbms_isma_param {
        unsigned char isbase64;
        struct cmbbms_isma mbbms_isma;
};

struct fw_info {
      unsigned char input[36]; 
      unsigned char output[16]; 
};


#define INNO_IOC_MAGIC      'i'
#define INNO_IO_POWERENABLE             _IOW(INNO_IOC_MAGIC, 1, int)
#define INNO_IO_RESET                   _IOW(INNO_IOC_MAGIC, 2, int)

#define INNO_IO_GET_FW_VERSION          _IOR(INNO_IOC_MAGIC, 5, unsigned int)
#define INNO_IO_SET_FREQUENCY           _IOW(INNO_IOC_MAGIC, 6, unsigned char)
#define INNO_IO_GET_FREQUENCY           _IOR(INNO_IOC_MAGIC, 7, unsigned char)
#define INNO_IO_SET_CH_CONFIG           _IOW(INNO_IOC_MAGIC, 8, struct ch_config)
#define INNO_IO_GET_CH_CONFIG           _IOWR(INNO_IOC_MAGIC, 9, struct ch_config)
#define INNO_IO_GET_SYS_STATUS          _IOR(INNO_IOC_MAGIC, 10, struct sys_status)
#define INNO_IO_GET_ERR_INFO            _IOR(INNO_IOC_MAGIC, 11, struct err_info)
#define INNO_IO_GET_CHIP_ID             _IOR(INNO_IOC_MAGIC, 12, int)
#define INNO_IO_UAM_TRANSFER            _IOWR(INNO_IOC_MAGIC, 13, struct uam_param)
#define INNO_IO_UAM_SETOVER             _IO(INNO_IOC_MAGIC, 14)
#define INNO_IO_UAM_SETCARDENV          _IOW(INNO_IOC_MAGIC, 15, unsigned char)
#define INNO_IO_UAM_MBBMS_ISMA          _IOW(INNO_IOC_MAGIC, 16, struct mbbms_isma_param)
#define INNO_IO_UAM_FWINFO              _IOW(INNO_IOC_MAGIC, 17, struct fw_info)
//add by mahanghong 20110210
#define INNO_IO_UAM_SETCW               _IOW(INNO_IOC_MAGIC, 18, struct cmbbms_cw)
//add by mahanghong 20110218
#define INNO_IO_UAM_SetAID3G            _IOW(INNO_IOC_MAGIC, 19, struct aid_3g)

#endif
