/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : mgmt_p2p.c                                           */
/*                                                                           */
/*  Description       : This file conatins the managemnet functions for      */
/*                      P2P protocol                                         */
/*                                                                           */
/*  List of Functions : get_p2p_attributes                                   */
/*                      p2p_get_attr                                         */
/*                      send_host_p2p_req                                    */
/*                      init_chan_entry_list_p2p                             */
/*                      init_p2p_globals                                     */
/*                      initiate_mod_switch                                  */
/*                      handle_inv_req_to                                    */
/*                      p2p_start_invit_scan_req                             */
/*                      p2p_update_cred_list                                 */
/*                      p2p_handle_prov_disc_req                             */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_P2P
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mgmt_p2p.h"
#include "frame_p2p.h"
#include "host_if.h"
#include "iconfig.h"
#include "mac_init.h"
#include "wps_sta.h"
#ifdef OS_LINUX_CSL_TYPE
#include "csl_linux.h"
#endif /* OS_LINUX_CSL_TYPE */

// add for drv_read_file() and drv_write_file()
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

const UWORD8 g_p2p_social_chan[P2P_NUM_SOCIAL_CHAN] = {CHANNEL1, CHANNEL6,
                                                       CHANNEL11};

/* Mode switch flag */
BOOL_T g_mode_switch_in_prog = BFALSE;

/* Scanning related variables */
p2p_dev_dscr_t g_p2p_dev_dscr_set[MAX_DEV_FOR_SCAN];
UWORD8 g_dev_dscr_set_index                   = 0;
UWORD8 g_p2p_manageability                    = 0; /* Flag for communication */
UWORD8 g_p2p_find_to                          = 25;
SWORD8 g_p2p_match_idx                        = -1;

/* Group Fomation related variables */
UWORD8 g_GO_tie_breaker                       = 0;
BOOL_T g_p2p_GO_role                          = BFALSE;
P2P_INVITE_STATE g_p2p_invit                  = NO_INVITE;

/* For power save */
noa_dscr_t g_noa_dscr[NUM_NOA_DSCR]           = {{0,}};
UWORD8 g_num_noa_sched                        = 0;
UWORD8 g_noa_index_go                         = 0;

/* Saved credential list          */
p2p_persist_list_t g_persist_list             = {0,};
p2p_persist_list_t *g_curr_persist_list       = NULL;

/* Current Dialog token for P2P frames */
UWORD8 g_p2p_dialog_token                     = 0;

// caisf add for p2p, 1117
// loff_t == long long
loff_t drv_write_file(char *file_path, char *str, size_t str_len, loff_t usr_pos)
{
    struct file *fp;
    mm_segment_t fs;
    loff_t pos = usr_pos;

    if(file_path == NULL || str == NULL || str_len <= 0)
    {
        printk("%s: param error(%p,%p,%d)! return.\n", __FUNCTION__,file_path,str,str_len);
        return -1;
    }

    fp = filp_open(file_path, O_RDWR | O_CREAT | O_TRUNC, 0644);//wxb add "O_TRUNC"
    if (IS_ERR(fp)) {
        printk("%s: open file=%s error!\n", __FUNCTION__,file_path);
        return -1;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);

    //pos = 0;
    vfs_write(fp, str, str_len, &pos);


    filp_close(fp, NULL);
    set_fs(fs);

    return pos;
}

// loff_t == long long
loff_t drv_read_file(char *file_path, char *str, size_t str_len, loff_t usr_pos)
{
    struct file *fp;
    mm_segment_t fs;
    loff_t pos = usr_pos;

    if(file_path == NULL || str == NULL || str_len <= 0)
    {
        printk("%s: param error(%p,%p,%d)! return.\n", __FUNCTION__,file_path,str,str_len);
        return -1;
    }

    fp = filp_open(file_path, O_RDONLY, 0644);
    if (IS_ERR(fp)) {
        printk("%s: open file=%s error!\n", __FUNCTION__,file_path);
        return -1;
    }

	fs = get_fs();
    set_fs(KERNEL_DS);

    //pos = 0;
    vfs_read(fp, str, str_len, &pos);

    filp_close(fp, NULL);
    set_fs(fs);

    return pos;
}


#ifndef MAC_HW_UNIT_TEST_MODE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_p2p_attributes                                       */
/*                                                                           */
/*  Description   : This function looks for a P2P IE and merges 2 or more    */
/*                  (consecutive or not) P2P IE's in a local memory buffer   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC frame start address                */
/*                  2) Index to the start of tagged parameters               */
/*                  3) Rx frame length                                       */
/*                  4) Pointer to the combined length of all attributes      */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    : The function searches the given frame for P2P IEs and    */
/*                  copies the content (after P2P OUI field) of the P2P IEs  */
/*                  found, in order, to a local memory buffer.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Direct: Pointer to the start of the P2P attribute list   */
/*                  Indirect: Combined length of all the attributes          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 *get_p2p_attributes(UWORD8 *msa, UWORD16 index, UWORD16 rx_len,
                           UWORD16 *comb_attr_len)
{
    UWORD8 *attr_ptr = NULL;
	UWORD16 num_bytes = 0;

	/* Initialize combined attribute length to be returned to 0 */
    *comb_attr_len = 0;

	/* Allocate a local memory buffer to hold the P2P attributes */
	attr_ptr = mem_alloc(g_local_mem_handle, MAX_COMB_P2P_ATTR_LEN);

	if(NULL == attr_ptr)
    {
		/* Buffer for P2P attributes could not be allocated, return NULL */
		return attr_ptr;
    }

    /* Loop and search for P2P IEs. Note that P2P IEs may not be present as  */
    /* consecutive IEs (observed with Wi-Fi test bed devices). Therefore,    */
    /* the complete frame is parsed for all P2P IEs that are present.        */
    while(index < (rx_len - FCS_LEN))
    {
        if(BTRUE == is_p2p_ie(msa + index))
        {
			/* Copy the P2P IE attribute field to the buffer if it is found */
			num_bytes = msa[index + 1] - P2P_OUI_LEN;
			memcpy((attr_ptr + *comb_attr_len),
			       (msa + index + IE_HDR_LEN + P2P_OUI_LEN), num_bytes);
			(*comb_attr_len) += num_bytes;
        }

	    /* Move to the next IE. */
	    index += (msa[index + 1] + IE_HDR_LEN);
    }

    /* If no P2P IE was found (i.e. combined attribute length is 0), then    */
    /* free the attribute pointer buffer and return NULL                     */
    if(0 == (*comb_attr_len))
    {
		mem_free(g_local_mem_handle, attr_ptr);
		attr_ptr = NULL;
    }

    return attr_ptr;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_get_attr                                             */
/*                                                                           */
/*  Description   : This function returns the pointer to the required        */
/*                  attribute.                                               */
/*                                                                           */
/*  Inputs        : 1) required attribute id                                 */
/*                  2) pointer to concatenated P2P attribute buffer          */
/*                  3) length of the concatenated P2P attribute buffer       */
/*                  4) pointer to the current attribute length               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This functions assumes that all P2P attributes are       */
/*                  concatenated in a single buffer. This function searches  */
/*                  for the given attribute and returns the pointer to the   */
/*                  start of the attribute data and the length of the        */
/*                  attribute                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Direct: pointer to the start of attribute data           */
/*                  Indirect: Length of the attribute                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *p2p_get_attr(UWORD8 attr_id, UWORD8 *data, UWORD16 ie_len,
                     UWORD16 *attr_len)
{
    UWORD16 index = 0;

    while(index < ie_len)
    {
        if(attr_id == data[index])
        {
            /* Attribute found */
            *attr_len = GET_ATTR_LEN(data+index);
            return (data + index + P2P_ATTR_HDR_LEN);
        }

        index += GET_ATTR_LEN(data+index) + P2P_ATTR_HDR_LEN;
    }

    return NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_host_p2p_req                                        */
/*                                                                           */
/*  Description   : This function sends an incoming P2P Action request to    */
/*                  the host.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming frame MAC header              */
/*                  2) Rx length                                             */
/*                                                                           */
/*  Globals       : g_info_id                                                */
/*                                                                           */
/*  Processing    : This function prepares an 'I' (information) type config  */
/*                  packet with WID type as WID_P2P_REQUEST, WID Data as the */
/*                  incoming frame contents (excluding the MAC header, FCS   */
/*                  and initial Action frame header upto the OUI Sub type    */
/*                  field). The same is then sent to the host.               */
/*                  This function should be used to send incoming requests   */
/*                  to the host if the device cannot take any decision and   */
/*                  requires user intervention to initiate further action.   */
/*                  For example, it can be used when an Invitation Request   */
/*                  is received but no information is available with the     */
/*                  device to accept/reject the same. The request is then    */
/*                  sent to the host. The user may accept/reject the same.   */
/*                  If accepted, appropriate WIDs shall be set to initiate   */
/*                  further action as desired.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_host_p2p_req(UWORD8 *msa, UWORD16 rx_len)
{
#if 0 /* TBD: This is currently not supported. Host-side support required. */
    UWORD8  *info_msg       = 0;
    UWORD8  *info_buf       = 0;
    UWORD16 p2p_req_msg_len = 0;
    UWORD16 req_len         = 0;

    info_buf = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);

    if(info_buf == NULL)
    {
        return;
    }

    info_msg = info_buf + get_config_pkt_hdr_len(get_host_config_if_type());

    /* Send the Action frame content (excluding the MAC header and FCS) from */
    /* the OUI Sub Type field onward.                                        */
    req_len = rx_len - (MAC_HDR_LEN + FCS_LEN) - P2P_PUB_ACT_OUI_SUBTYPE_OFF;

    p2p_req_msg_len = MSG_DATA_OFFSET + WID_LENGTH_OFFSET + 2 + req_len;

    /* Prepare the P2P Request message to be sent to the host */

    /* WID_P2P_REQUEST (Binary WID) Message Format                           */
    /* --------------------------------------------------------------------- */
    /* | Message Type | ID | Length | WID Type | WID Length | WID Data     | */
    /* --------------------------------------------------------------------- */
    /* | 1            | 1  | 2      | 2        | 2          | req_len      | */
    /* --------------------------------------------------------------------- */
    info_msg[0] = 'I';
    info_msg[1] = g_info_id++;
    info_msg[2] = (p2p_req_msg_len) & 0xFF;
    info_msg[3] = ((p2p_req_msg_len) & 0xFF00) >> 8;
    info_msg[4] = WID_P2P_REQUEST & 0xFF;
    info_msg[5] = (WID_P2P_REQUEST >> 8) & 0xFF;
    info_msg[6] = (req_len) & 0xFF;
    info_msg[7] = ((req_len) & 0xFF00) >> 8;

    /* Copy the WID data */
    memcpy((info_msg + 8), (msa + MAC_HDR_LEN + P2P_PUB_ACT_OUI_SUBTYPE_OFF),
           req_len);

    /* Send the information message to the host */
    send_host_rsp(info_buf, p2p_req_msg_len, get_host_config_if_type());
#endif /* TBD: This is currently not supported. Host-side support required. */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_chan_entry_list_p2p                                 */
/*                                                                           */
/*  Description   : This function adds the Operating class and the           */
/*                  corresponding list of channel to the channel entry list  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_chan_entry_list                                    */
/*                  g_p2p_len_chan_entry_list                                */
/*                                                                           */
/*  Processing    : This function adds the Operating Classes and the list of */
/*                  channels to the channel entry list depending upon the    */
/*                  country of operation. It also updates the length of the  */
/*                  channel entry list. For the current usage IEEE Draft     */
/*                  P802.11-REVmb is refered and Operating class 12 is used  */
/*                  for US and Operating 30 and 31 are used for JAPAN.       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void init_chan_entry_list_p2p(void)
{
    UWORD8 index  = 0;
    UWORD8 ch_idx = 0;
    UWORD8 freq   = get_current_start_freq();
    UWORD8 max_ch = get_max_num_channel(freq);

    if(freq == RC_START_FREQ_5) /* P2P in 5 GHz not supported */
        return;

    /* MD-TBD : To be updated correctly based on Regulatory domains */
#ifdef P2P_NO_COUNTRY
    g_p2p_chan_entry_list[index++] = P2P_OPERATING_CLASS1;

    index++; /* Skip Length field - to be updated later */

    for(ch_idx = 0; ch_idx < max_ch; ch_idx++)
    {
		if(is_ch_idx_supported(freq, ch_idx) == BTRUE)
    {
			/* Get the channel number corresponding to the index and add it  */
			/* to the channel list array                                     */
			g_p2p_chan_entry_list[index++] = get_ch_num_from_idx(freq, ch_idx);
    }
    }

    /* Update the IE length field and the global channel entry IE length */
    g_p2p_chan_entry_list[1] = (index - IE_HDR_LEN);
    g_p2p_len_chan_entry_list = index;
#endif /* P2P_NO_COUNTRY */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_p2p_globals                                         */
/*                                                                           */
/*  Description   : This funtion initializes the globals that are common for */
/*                  AP and STA                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : It initializes all p2p global variables to the default   */
/*                  values                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void init_p2p_globals(void)
{
	TROUT_FUNC_ENTER;
    init_chan_entry_list_p2p();
    g_dev_dscr_set_index                   = 0;
    g_p2p_manageability                    = 0;
    mem_set(g_noa_dscr, 0, NUM_NOA_DSCR * sizeof(noa_dscr_t));
    set_num_noa_sched(0);
    g_GO_tie_breaker                       = get_random_byte() & 1;
    g_p2p_match_idx                        = -1;
    g_p2p_invit                            = NO_INVITE;
    g_p2p_opp_ps                           = BFALSE;
    set_noa_index(0);

    set_p2p_find_to(25);
    
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initiate_mod_switch                                      */
/*                                                                           */
/*  Description   : This funtion handles the switching of modules between    */
/*                  AP and STA                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_current_settings                                       */
/*                  g_current_len                                            */
/*                                                                           */
/*  Processing    : It saves all the WID's and resets MAC so that no more    */
/*                  operation happens. It copies the saved WID's to MAC HW   */
/*                  shared memory and then calls a function to switch the    */
/*                  modules                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void initiate_mod_switch(mac_struct_t *mac)
{
#if 0 //caisf add for P2P
    UWORD8 *p2p_saved_settings = (UWORD8 *)SHARED_DSCR_MEM_POOL_BASE;
#else
    WORD32 write_len = 0, p2p_file_pos = 0;
    UWORD8 *p2p_saved_settings = (UWORD8 *)kmalloc(P2P_WID_CONFIG_MEM_SIZE, GFP_KERNEL);
    if (NULL == p2p_saved_settings)
    {
       TROUT_DBG4("no men\n");
	return;
    }
    memset(p2p_saved_settings, 0, P2P_WID_CONFIG_MEM_SIZE);
#endif


	/* Switching to other mode has started - This flag is set to allow   */
	/* the setting of certain WIDs that should not be modified by the    */
	/* user but need to be restored to default values during a switch.   */
    set_switch_in_progress(BTRUE);

    /* Restore default settings for any required WIDs for switching */
    restore_default_mode_wids();

    TROUT_DBG4("caisf set_device_mode\n");
    /* Change the device mode to NOT_CONFIGURED */
    set_device_mode(NOT_CONFIGURED);

    /* Save the current configuration before resetting */
    save_wids();

    /* Reset MAC - Bring down PHY and MAC H/W, disable MAC interrupts and    */
    /* release all OS structures.                                            */
    reset_mac(mac, BTRUE);
    
    /* No need to restore back the configuration. Copy the settings to the  */
    /* shared MAC HW memory                                                 */
    strcpy((WORD8 *)p2p_saved_settings, P2P_MAGIC_STR);
    p2p_saved_settings[P2P_MAGIC_STR_LEN + 1] = g_current_len & 0xff;
    p2p_saved_settings[P2P_MAGIC_STR_LEN + 2] = (g_current_len >> 8) & 0xff;
    
    memcpy(&p2p_saved_settings[P2P_MAGIC_STR_LEN + 3], g_current_settings,
           g_current_len);

#if 1 //caisf add for P2P
    //caisf mod wid store style. 1118
    // write wid config from file
    if(g_current_len > P2P_WID_CONFIG_MEM_SIZE)
        TROUT_DBG4("P2P: g_current_len=%d\n",g_current_len);

    write_len = drv_write_file(P2P_CONFIG_FILE_PATH, p2p_saved_settings,
            g_current_len + P2P_MAGIC_STR_LEN + 3, p2p_file_pos);  //wxb modify
    if(write_len <= 0)
    {
        printk("%s-%d: write file:%s ERROR!\n",__FUNCTION__,__LINE__,
            P2P_CONFIG_FILE_PATH);
        kfree(p2p_saved_settings);
        return;
    }

    kfree(p2p_saved_settings);
#endif

    switch_operating_mode();
	TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_inv_req_to                                        */
/*                                                                           */
/*  Description   : This function handles the invitation request timeout     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_invit                                              */
/*                                                                           */
/*  Processing    : This function is called when a timeout occurs after the  */
/*                  invitation request is sent. It sets the target device id */
/*                  to NULL and the invitation flag to BFALSE                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void handle_inv_req_to(mac_struct_t *mac)
{
    mset_p2p_trgt_dev_id("\0\0\0\0\0\0");
    mset_p2p_invit_dev_id("\0\0\0\0\0\0");
    g_p2p_invit = NO_INVITE;
    process_scan_itr_comp_mode(mac);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_start_invit_scan_req                                 */
/*                                                                           */
/*  Description   : This function calls the core mode function to handle the */
/*                  invitation scan request                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the core mode function to handle the */
/*                  invitation scan request                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void p2p_start_invit_scan_req(void)
{
    p2p_start_invit_scan_req_mode();
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_update_cred_list                                     */
/*                                                                           */
/*  Description   : This function updates the persistent credential list     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the persistent credential list.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void p2p_update_cred_list(void)
{

    /* Update the group capability bitmap */
    g_persist_list.grp_cap = get_p2p_grp_cap();

    if(BFALSE == g_p2p_GO_role)
    {
        g_persist_list.num_cl = 1;
        mac_addr_cpy(g_persist_list.cl_list[0], mget_p2p_trgt_dev_id());
    }
    else
    {
        g_persist_list.grp_cap |= P2PCAPBIT_GO;
    }

    /* Update the Authentication type and Security Mode */
    g_persist_list.auth_type = get_auth_type();
    g_persist_list.sec_mode  = get_802_11I_mode();

    /* Update the PSK */
    g_persist_list.key_len = mget_RSNAConfigPSKPassPhraseLength();
    memcpy(g_persist_list.psk, mget_RSNAConfigPSKPassPhraseValue(),
           g_persist_list.key_len);

    /* Update the SSID */
    strcpy((WORD8 *)g_persist_list.ssid, mget_DesiredSSID());

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_handle_prov_disc_req                                 */
/*                                                                           */
/*  Description   : This function process the provision discovery request frm*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address of the received frm  */
/*                  2) Address of the device that sent the frm               */
/*                  3) Length of the incoming frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the configuration method present  */
/*                  in the provision discovery request frame and checks if   */
/*                  it matches with the one of the config methods supported  */
/*                  by the device. If the check is successful then it sends  */
/*                  the response frame with the same config method as in the */
/*                  request frame else it sends a NULL config method in the  */
/*                  response frame                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_handle_prov_disc_req(UWORD8 *msa, UWORD8 *sa, UWORD16 rx_len)
{
    UWORD16 config_method = 0;

	TROUT_FUNC_ENTER;
    /* Call the function to extract the config method present in the WSC IE */
    config_method = wps_get_config_method(msa, rx_len);

    /* If the config method is keypad then we do not do any further       */
    /* and send the config method in prov disc resp frame as keypad. This */
    /* is done because our present wps doesnot advertise keypad as its    */
    /* supported config methods. This has to be updated after checking the*/
    /* behaviour of test bed devices                                      */
    if(WPS_CONFIG_METH_KEYPAD != config_method)
    {
	    /* Check if the config method set in the provision discovery request */
	    /* matches with one of the deivces config methods                    */
	    if(0 != (get_wps_config_method() & config_method))
	    {
	        /* Send the message to host */
	        send_host_p2p_req(msa, rx_len);
	    }
	    else
	    {
	        config_method = 0;
        }
    }

    /* Send provision disc resp with common config method*/
    send_prov_disc_resp(msa, config_method, sa);
    TROUT_FUNC_EXIT;
}

#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_P2P */
