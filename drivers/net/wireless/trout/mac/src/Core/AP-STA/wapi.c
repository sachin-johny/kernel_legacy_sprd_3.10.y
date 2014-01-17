//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : wapi.c                                                */
/*                                                                           */
/*  Description       : This file contains all the WEP management related    */
/*                      functions.                                           */
/*                                                                           */
/*  List of Functions : init_wep                                             */
/*                      init_wep_keys                                        */
/*                      add_wep_entry                                        */
/*                      msg_3_timeout                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management.h"
#include "mib.h"
#include "mh.h"
#include "wapi.h"
#include "ce_lut.h"
#include "host_if.h"

//#ifdef IBSS_BSS_STATION_MODE
#include "management_sta.h"
#include "cglobals_sta.h"
//#endif

#define KEYID_LEN 1
#define RESERVD_LEN 1
#define PN_LEN 16
#define MIC_LEN 16

/*****************************************************************************/
/* Global Variable Declarations                                              */
/*****************************************************************************/
UWORD8 g_wapi_oui[3] = {0x00,0x14,0x72};

const UWORD16 frame_cntl_mask = 0x8FC7; //little order,4, 5, 6, 11, 12, 13 should be set 0
const UWORD16 seq_cntl_mask = 0x0F00; //little order,bit 4~15 should be set 0

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_wep                                                 */
/*                                                                           */
/*  Description   : This function initializes the WEP keys                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the WEP keys                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

/* This function checks if the received OUI and the configured OUI are same  */
/* Returns TRUE is the OUI matches ; FALSE otherwise                         */
void init_wapi(void)
{
    /* disable ce enable in pa control*/
	disable_ce();

	/* enable wapi in ext pa control*/
	enable_wapi();
}

INLINE BOOL_T check_wapi_oui(UWORD8* frame, UWORD8 *oui)
{
    if((frame[0] == oui[0]) && (frame[1] == oui[1]) && (frame[2] == oui[2]))
    {
        return BTRUE;
    }

    return BFALSE;
}

UWORD16 set_asoc_req_wapi_ie(UWORD8 *data, UWORD16 index)
{
	UWORD16 wapi_head_index = index;
	UWORD16 wapi_ie_len = 0;
	int i = 0;

	printk("set_asoc_req_wapi_ie 1\n");
	if( mget_wapi_enable() == TV_FALSE )
	{
		return wapi_ie_len;
	}
	printk("set_asoc_req_wapi_ie 2\n");

	data[index] = IWAPIELEMENT;

	/* Skip the Tag number and the length */
	index += 2;

	/* copy wapi version 2 byte */
	memcpy(&data[index],mget_wapi_version(),2);
	index += 2;

	/* copy wapi akm cnt 1 byte*/
	data[index] = mget_wapi_akm_cnt();
	data[index+1] = 0;
	index += 2;
	

	/* copy wapi akm n*4 byte */
	for( i = 0; i < mget_wapi_akm_cnt() ; i++ )
	{
		memcpy(&data[index],mget_wapi_akm(i),4);
		index += 4;
	}

	/* copy wapi pcip cnt 1 byte*/
	data[index] = mget_wapi_pcip_cnt();
	data[index+1] = 0;
	index += 2;

	/* copy wapi pcip policy n*4 byte */
	for( i = 0; i < mget_wapi_pcip_cnt() ; i++ )
	{
		memcpy(&data[index],mget_wapi_pcip_policy(i),4);
		index += 4;
	}

	/* copy wapi grp policy 4 byte */
	memcpy(&data[index],mget_wapi_grp_policy(),4);
	index += 4;

	/* copy WAPI CAP info,we set 0*/
	memset(&data[index],0x00,2);
	index += 2;

	/* copy BKID,we set 0*/
	memset(&data[index],0x00,2);
	index += 2;

	wapi_ie_len = index - wapi_head_index;

	data[wapi_head_index + 1] = wapi_ie_len - 2;

	printk("set_asoc_req_wapi_ie 3,len = %d\n",wapi_ie_len);

	for(i = 0 ;i < wapi_ie_len ; i++)
	{
		printk("%02x ",data[wapi_head_index + i]);
	}
	printk("\n");
	
	return wapi_ie_len;
}

void update_join_req_params_wapi(void *ptr)
{
    bss_dscr_t *bss_dscr = (bss_dscr_t *)ptr;
	int i = 0;

	if( mget_wapi_enable() == TV_FALSE )
	{
		return;
	}
	
	mset_wapi_version( bss_dscr->wapi_version );

    mset_wapi_akm_cnt(bss_dscr->wapi_akm_cnt);

	for(i = 0 ; i < bss_dscr->wapi_akm_cnt ; i++)
	{
		mset_wapi_akm(i,bss_dscr->wapi_akm[i]);
	}

	mset_wapi_pcip_cnt(bss_dscr->wapi_pcip_cnt);

	for( i = 0 ; i < bss_dscr->wapi_pcip_cnt ; i++ )
	{
		mset_wapi_pcip_policy(i,bss_dscr->wapi_pcip_policy[i]);
	}	

	mset_wapi_grp_policy(bss_dscr->wapi_grp_policy);
	mset_wapi_cap(bss_dscr->wapi_cap);
}


void update_scan_response_wapi(UWORD8 *msa, UWORD16 rx_len, UWORD16 offset,
                              UWORD8 dscr_set_index)
{
    UWORD16 index  = offset;
    UWORD16 index1 = offset;
    int i = 0;
	int j = 0;

	//printk("update_scan_response_wapi 1\n");
	//chenq mask
	//if( mget_wapi_enable() == TV_FALSE )
	//{
		//return;
	//}

	//printk("update_scan_response_wapi 2\n");
    while(index < (rx_len - FCS_LEN))
    {
        index1 = index;

        /* Only copy the information element which is supported by this      */
        /* station. If it is a user initiated scan, then anyway it has to be */
        /* copied                                                            */
        if( msa[index] == IWAPIELEMENT )
        {
			//chenq add 2013-06-08
			g_bss_dscr_set[dscr_set_index].wapi_found = BTRUE;
			//chenq add end
		
			/* Reset the RSN Element Parsed condition */
			g_bss_dscr_set[dscr_set_index].rsn_found = BFALSE;
			g_bss_dscr_set[dscr_set_index].dot11i_info = 0;
		
        	//printk("update_scan_response_wapi 3,68 len = %d\n",msa[index + 1]);
			
			//for(i = 0 ;i < msa[index + 1] ; i++)
			//{
				//printk("%02x ",msa[index + i]);
			//}
			//printk("\n");

			
            /* Skip the Tag number and the length */
            index += 2;

			/* save wapi version 2 byte */
			memcpy(g_bss_dscr_set[dscr_set_index].wapi_version,&msa[index],2);
			index += 2;

			/* save wapi akm cnt 1 byte*/
			g_bss_dscr_set[dscr_set_index].wapi_akm_cnt = msa[index];
			j = ( msa[index] > 3 ) ? 3 : msa[index];
			index += 2;

			/* save wapi akm n*4 byte */
			for( i = 0; i < j ; i++ )
			{
				if( !(check_wapi_oui(&msa[index],g_wapi_oui)))
				{
					g_bss_dscr_set[dscr_set_index].wapi_akm_cnt = 0;
					return;
				}
				memcpy(&(g_bss_dscr_set[dscr_set_index].wapi_akm[i]),&msa[index],4);
				index += 4;
			}

			/* save wapi pcip cnt 1 byte*/
			g_bss_dscr_set[dscr_set_index].wapi_pcip_cnt = msa[index];

			j = ( msa[index] > 3 ) ? 3 : msa[index];
			index += 2;

			/* save wapi pcip policy n*4 byte */
			for( i = 0; i < j ; i++ )
			{
				if( !(check_wapi_oui(&msa[index],g_wapi_oui)))
				{
					g_bss_dscr_set[dscr_set_index].wapi_akm_cnt = 0;
					return;
				}
				
				memcpy(&(g_bss_dscr_set[dscr_set_index].wapi_pcip_policy[i]),&msa[index],4);
				index += 4;
			}

			/* save wapi grp policy 4 byte */
			if( !(check_wapi_oui(&msa[index],g_wapi_oui)) )
			{
				g_bss_dscr_set[dscr_set_index].wapi_akm_cnt = 0;
				return;
			}
			memcpy(g_bss_dscr_set[dscr_set_index].wapi_grp_policy,&msa[index],4);
			index += 4;

			memcpy(g_bss_dscr_set[dscr_set_index].wapi_cap,&msa[index],2);
			
			break;
        }
        else
        {
            index1 += msa[index1 + 1] + 2;
            index   = index1;
        }
    }
}

/**************************************************************************/
/*                        wapi MSDU format                                */
/* ---------------------------------------------------------------------- */
/* |MAC Heager|keyID|Res|PN(iv)|PDU(Encryption)|MIC(Encryption)|FCS       */
/* ---------------------------------------------------------------------- */
/* |30/24     |1    |1  |16    |>=1            |16             |4  		  */
/* ---------------------------------------------------------------------- */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/*                        calc mic data format                            */
/* ---------------------------------------------------------------------- */
/* |frame cntl|addr1|addr2|seq cntl|addr3|addr4|qos|keyID|Res|Length      */
/* ---------------------------------------------------------------------- */
/* |2         |6    |6    |2       |6    |6    |2  |1    |1  |2(big order)*/
/* ---------------------------------------------------------------------- */
/*                                                                        */
/**************************************************************************/


UWORD16 wlan_rx_wapi_encryption(UWORD8 * header,
									   UWORD8 * data,UWORD16 data_len,
	                                   UWORD8 * ouput_buf)
{
	UWORD16 offset = 0;
	BOOL_T  qos_in = BFALSE;
	BOOL_T  valid_addr4 = BTRUE;
	
	UWORD8 ptk_header[36] = {0};
	UWORD16 ptk_headr_len = 32;
	UWORD8 * p_ptk_header = ptk_header;
	UWORD8 * p_outputdata = ouput_buf + get_eth_hdr_offset(ETHERNET_HOST_TYPE);
	UWORD8 data_mic[16] = {0};

	UWORD8 * iv  = inc_wapi_pairwise_key_txrsc();
	#if 0
	UWORD8 * iv = NULL;
	#endif
	UWORD8 keyid = mget_wapi_pairwise_key_index();

	int i = 0;

#if 0
	UWORD8 input_pn[] =
{0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36};
//{0x3a,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c};

UWORD8 input_key[] =
//{0x95,0xde,0x27,0x7e,0xb4,0x8d,0x38,0xb1,0x4c,0x61,0x65,0x20,0xfb,0x66,0x32,0x3c};
{0x3c,0x32,0x66,0xfb,0x20,0x65,0x61,0x4c,0xb1,0x38,0x8d,0xb4,0x7e,0x27,0xde,0x95};

UWORD8 input_mic_key[] =
{0x74,0xe0,0x90,0x13,0x0a,0x6d,0x26,0xbd,0x5e,0xda,0xf1,0xa3,0x82,0x95,0x1a,0x9e};
//{0x9e,0x1a,0x95,0x82,0xa3,0xf1,0xda,0x5e,0xbd,0x26,0x6d,0x0a,0x13,0x90,0xe0,0x74};

	mset_wapi_pairwise_key_txrsc(input_pn);
	iv  = inc_wapi_pairwise_key_txrsc();
	mset_wapi_pairwise_mic_key(keyid,input_mic_key);
	mset_wapi_pairwise_pkt_key(keyid,input_key);
#endif

	/* save frame cntl */
	*p_ptk_header     = header[offset]   & (frame_cntl_mask >> 8);
	*(p_ptk_header+1) = header[offset+1] & (frame_cntl_mask & 0xFF);

	if( *p_ptk_header & 0x80 )
	{
		qos_in = BTRUE;
		ptk_headr_len += 2;//add qos len 2 byte
	}

	if( (*(p_ptk_header+1) & 0x03 ) != 0x03) //valid addr4 in case:ToDS==1 && FromDS==1
	{
		valid_addr4 = BFALSE;
	}
	
	p_ptk_header += 2;
	offset += 2;

	/* jump over duration id*/
	offset += 2;
	
	/* save addr1 addr2 */
	memcpy(p_ptk_header,&header[offset],12);
	p_ptk_header += 12;
	offset += 12;

	/* save seq cntl */
	*p_ptk_header     = header[offset+6]   & (seq_cntl_mask >> 8);
	*(p_ptk_header+1) = header[offset+6+1] & (seq_cntl_mask & 0xFF);
	p_ptk_header += 2;

	/* save addr3 */
	memcpy(p_ptk_header,&header[offset],6);
	p_ptk_header += 6;
	offset       += 6;

	/* save addr4 */
	if(valid_addr4)
	{
		memcpy(p_ptk_header,&header[offset],6);
		p_ptk_header += 6;
		offset += 6;
	}
	else
	{
		memset(p_ptk_header,0x00,6);
		p_ptk_header += 6;
	}

	//jump seq cntl
	offset       += 2;
	
	/* save qos */
	if(qos_in)
	{
		memcpy(p_ptk_header,&header[offset],2);
		p_ptk_header += 2;
		offset       += 2;
	}

	/* save keyid */
	*p_ptk_header = keyid;
	p_ptk_header++;
	
	/* reserved */
	*p_ptk_header = 0x00;
	p_ptk_header++;

	/* save data len */
	*p_ptk_header     = (data_len >> 8);
	*(p_ptk_header+1) = data_len & 0xFF;

	#if 0
	printk("chenq_test,mic header(len = %d):\n",ptk_headr_len);
	for( i = 0 ; i < ptk_headr_len ; i++)
	{
		printk("%02x ",ptk_header[i]);	
	}
	printk("\n");

	printk("chenq_test,iv:\n");
	for( i = 0 ; i < 16 ; i++)
	{
		printk("%02x ",iv[i]);	
	}
	printk("\n");

	printk("chenq_test,mic:\n");
	for( i = 0 ; i < 16 ; i++)
	{
		printk("%02x ",*(mget_wapi_pairwise_mic_key(keyid) + i));	
	}
	printk("\n");

	printk("chenq_test,key:\n");
	for( i = 0 ; i < 16 ; i++)
	{
		printk("%02x ",*(mget_wapi_pairwise_pkt_key(keyid) + i));	
	}
	printk("\n");
	#endif
	
	/* calc mic*/
	WapiCryptoSms4Mic(iv,
	                  mget_wapi_pairwise_mic_key(keyid),
	                  ptk_header,ptk_headr_len,data,data_len,data_mic);

	/* add mic to data */
	memcpy(&data[data_len],data_mic,16);
	data_len += 16;

	/* encryption data(inclue mic) & save keyid & iv */

	WapiCryptoSms4(iv,
				   mget_wapi_pairwise_pkt_key(keyid),
				   data,data_len,
	               p_outputdata + 1 + 1 + 16 );

	*p_outputdata    = keyid;
	*(p_outputdata + 1) = 0x00;
	p_outputdata += 2;
	
	for( i = 15 ; i >= 0 ; i-- )
	{
		*p_outputdata = iv[i];
		p_outputdata++;
	}

	return data_len + 1 + 1 + 16;

}

UWORD16 wlan_tx_wapi_decryption(UWORD8 * input_ptk,UWORD16 header_len,UWORD16 data_len,
	                                   UWORD8 * output_buf)
{
	UWORD16 offset = 0;
	BOOL_T  qos_in = BFALSE;
	BOOL_T  valid_addr4 = BTRUE;
	BOOL_T  is_group_ptk = BFALSE;
	
	UWORD8 ptk_header[36] = {0};
	UWORD16 ptk_headr_len = 32;
	UWORD8 * p_ptk_header = ptk_header;
	//UWORD8 * p_outputdata = ouput_buf + get_eth_hdr_offset(ETHERNET_HOST_TYPE);
	UWORD8 data_mic[16] = {0};
	UWORD8 calc_data_mic[16] = {0};
	UWORD8 iv[16] = {0};
	UWORD8 keyid = {0};
	
	UWORD16 ral_data_len = 0;
	UWORD16 encryp_data_len = 0;

	int i = 0;

	/* save calc mic header*/
	
	/* save frame cntl */
	*p_ptk_header     = input_ptk[offset]   & (frame_cntl_mask >> 8);
	*(p_ptk_header+1) = input_ptk[offset+1] & (frame_cntl_mask & 0xFF);

	if( *p_ptk_header & 0x80 )
	{
		qos_in = BTRUE;
		ptk_headr_len += 2;//add qos len 2 byte
	}

	if( (*(p_ptk_header+1) & 0x03 ) != 0x03) //valid addr4 in case:ToDS==1 && FromDS==1
	{
		valid_addr4 = BFALSE;
	}
	
	p_ptk_header += 2;
	offset += 2;

	/* jump over duration id*/
	offset += 2;
	
	/* save addr1 addr2 */
	memcpy(p_ptk_header,&input_ptk[offset],12);
	is_group_ptk = is_group(p_ptk_header);
	p_ptk_header += 12;
	offset += 12;

	/* save seq cntl */
	*p_ptk_header     = input_ptk[offset+6]   & (seq_cntl_mask >> 8);
	*(p_ptk_header+1) = input_ptk[offset+6+1] & (seq_cntl_mask & 0xFF);
	p_ptk_header += 2;

	/* save addr3 */
	memcpy(p_ptk_header,&input_ptk[offset],6);
	p_ptk_header += 6;
	offset       += 6;

	/* save addr4 */
	if(valid_addr4)
	{
		memcpy(p_ptk_header,&input_ptk[offset],6);
		p_ptk_header += 6;
		offset += 6;
	}
	else
	{
		memset(p_ptk_header,0x00,6);
		p_ptk_header += 6;
	}

	//jump seq cntl
	offset       += 2;
	
	/* save qos */
	if(qos_in)
	{
		memcpy(p_ptk_header,&input_ptk[offset],2);
		p_ptk_header += 2;
		offset       += 2;

		//mac h/w offset 2 byte to multiple of 4
		offset       += 2;
	}

	/* save keyid */
	*p_ptk_header = input_ptk[offset];
	keyid         = input_ptk[offset];
	p_ptk_header++;
	offset++;
	
	/* reserved */
	*p_ptk_header = input_ptk[offset];
	p_ptk_header++;
	offset++;

	/* save data len */
	encryp_data_len = data_len - KEYID_LEN - RESERVD_LEN - PN_LEN;
	ral_data_len = data_len - KEYID_LEN - RESERVD_LEN - PN_LEN - MIC_LEN;
	*p_ptk_header     = (ral_data_len >> 8);
	*(p_ptk_header+1) = ral_data_len & 0xFF;

	/* save calc mic header over*/

	/* save iv */
	for( i = 15 ; i >= 0 ; i-- )
	{
		iv[i] = input_ptk[offset];
		offset++;
	}

	#if 0
	printk("chenq_test rx ptk header(head + wapi head = %d):\n",header_len + KEYID_LEN + RESERVD_LEN + PN_LEN);
	for( i = 0 ; i < header_len + KEYID_LEN + RESERVD_LEN + PN_LEN ; i++)
	{
		printk("%02x ",input_ptk[i]);
	}

	printk("all ptk is:\n");
	for( i = 0 ; i < header_len + data_len ; i++)
	{
		printk("%02x ",input_ptk[i]);
	}
	printk("\n");

	
	printk("chenq_test,mic header(len = %d):\n",ptk_headr_len);
	for( i = 0 ; i < ptk_headr_len ; i++)
	{
		printk("%02x ",ptk_header[i]);	
	}
	printk("\n");

	printk("chenq_test,iv:\n");
	for( i = 0 ; i < 16 ; i++)
	{
		printk("%02x ",iv[i]);	
	}
	printk("\n");
	#endif

	/* add adjust here,later...*/
	if(is_group_ptk)
	{
		//if( (iv[15] & 0x01) == 0x00 )
		//{
		//	return 0;
		//}
	}
	else
	{
		if( (iv[15] & 0x01) != 0x01 )
		{
			return 0;
		}
	}


	/* decryption */
	if(is_group_ptk)
	{
		WapiCryptoSms4(iv,
					   mget_wapi_group_pkt_key(keyid),
					   (input_ptk + header_len + KEYID_LEN + RESERVD_LEN + PN_LEN),encryp_data_len,
		                output_buf);
	}
	else
	{
		WapiCryptoSms4(iv,
					   mget_wapi_pairwise_pkt_key(keyid),
					   (input_ptk + header_len + KEYID_LEN + RESERVD_LEN + PN_LEN),encryp_data_len,
		               output_buf);
	}
	memcpy(data_mic,output_buf + ral_data_len,MIC_LEN);

	/* calc mic */
	if(is_group_ptk)
	{
		WapiCryptoSms4Mic(iv,
		                  mget_wapi_group_mic_key(keyid),
		                  ptk_header,ptk_headr_len,
		                  (output_buf),ral_data_len,
		                  calc_data_mic);
	}	
	else
	{
		WapiCryptoSms4Mic(iv,
		                  mget_wapi_pairwise_mic_key(keyid),
		                  ptk_header,ptk_headr_len,
		                  (output_buf),ral_data_len,
		                  calc_data_mic);
	}

	if( memcmp(calc_data_mic,data_mic,MIC_LEN) != 0 )
	{
		return 0;
	}
	else
	{
		return ral_data_len;
	}

}
#endif
