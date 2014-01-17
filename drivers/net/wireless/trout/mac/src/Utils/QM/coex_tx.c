#include "imem_if.h"
#include "qmu_if.h"
#include "spi_interface.h"
#include "trout_share_mem.h"
#include "management_11n.h"
#include "core_mode_if.h"
#include "qmu.h"

#include "qmu_tx.h"

#ifdef IBSS_BSS_STATION_MODE

UWORD32 *g_trout_self_cts_null_data_buf = NULL;
UWORD32 *g_trout_ps_null_data_buf = NULL;
BOOL_T g_wifi_bt_coex = BFALSE;


void wifi_bt_coexist_init(void)
{
	enable_txptr_update();
	
	g_wifi_bt_coex = BFALSE;
	g_trout_self_cts_null_data_buf = NULL;
	g_trout_ps_null_data_buf = NULL;
}


/* NOTE: This function is used in BT/WiFI coexist, WiFi send a CTS befor NULL */
/* data frame.                                                                */
void init_coex_self_cts_null_data(UWORD8 psm, BOOL_T is_qos, UWORD8 priority)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};
    UWORD8      frame_len = MAC_HDR_LEN;
    UWORD8      serv_class = NORMAL_ACK;
	UWORD8      phy_rate = 0;
	qmu_tx_handle_t *tx_handle = NULL;
	buffer_desc_t buff_list;
	UWORD32 trout_dscr, trout_buf;
	UWORD8 pr;
	
	//dbg.
	//UWORD8 tmp_dscr[TX_DSCR_BUFF_SZ];
	//UWORD8 tmp_pkt[128];

	if(g_trout_self_cts_null_data_buf != NULL)
		return;

	se = (sta_entry_t *)find_entry(mget_bssid());
    if(NULL == se)
    {
        TROUT_DBG3("%s: Err, send null data in not associated stage!\n", __func__);
        /* This is an exception case and should never occur */
        return;
    }
	
    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        TROUT_DBG4("%s: No Memory for NULL frame\n", __func__);
        return;
    }

    /* Set the Frame Control field of the NULL frame. */
#ifdef MAC_WMM
    if(is_qos == BTRUE)
    {
        set_frame_control(msa, (UWORD16)QOS_NULL_FRAME);
        q_num = get_txq_num(priority);
    }
    else
#endif /* MAC_WMM */
    {
        set_frame_control(msa, (UWORD16)NULL_FRAME);
        q_num       = HIGH_PRI_Q;
    }

    set_pwr_mgt(msa, (UWORD8)psm);
    set_to_ds(msa, 1);

    if(psm == 0)
        reset_machw_ps_pm_tx_bit();
    else
        set_machw_ps_pm_tx_bit();

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());

    if(is_qos == BTRUE)
    {
		set_qos_control(msa, priority, serv_class);
        frame_len += 2;
    }

    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
    if(tx_dscr == NULL)
    {
		TROUT_DBG3("No Mem for NULL TX DSCR \n");
        /* Free the memory allocated for the buffer */
        pkt_mem_free(msa);
        return;
    }

    /* Set the transmit rate from the station entry */
    //tx_rate = get_tx_rate_to_sta(se);
	tx_rate = 6;	//use permanent speed of 24Mbps.
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    phy_rate = get_phy_rate(tx_rate);
	/*junbin.wang add for long preamble.20131118*/
	pr = get_preamble(phy_rate);
	if(IS_RATE_11B(phy_rate) == BTRUE)
		phy_rate |= (BIT2 & (pr << 2));
	
    /* If AUTORATE_FEATURE is not defined or retransmission auto rate is not */
    /* enabled set all retry rates to the transmit rate                      */
    retry_set[0] = (phy_rate << 24) | (phy_rate << 16) | (phy_rate << 8) | (phy_rate);
    retry_set[1] = retry_set[0];

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, serv_class, phy_tx_mode, retry_set);
    //set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);	//adjust pkt length here!!!
    //set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);
    set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 2300);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

	//printk("before covent:\n");
	//hex_dump("tx_dscr", tx_dscr, TX_DSCR_BUFF_SZ);
	//hex_dump("tx_pkt", msa, frame_len);

	tx_handle = &g_q_handle.tx_handle;
	g_trout_self_cts_null_data_buf = (UWORD32 *)(tx_handle->tx_mem_end + COEX_SLOT_INFO_SIZE);

	trout_dscr = (UWORD32)g_trout_self_cts_null_data_buf;
	trout_buf = trout_dscr + TX_DSCR_BUFF_SZ;
	printk("%s: dscr=0x%x, buf=0x%x\n", __func__, trout_dscr, trout_buf);
	
	set_tx_dscr_buffer_addr(tx_dscr, trout_buf);
	set_tx_dscr_host_dscr_addr(tx_dscr, 0);
	set_tx_dscr_next_addr(tx_dscr, 0);
	
	get_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr, (UWORD32)(&buff_list), 0);
	//printk("sub_msdu_info0: buff_hdl=0x%x, data_offset=%d, data_len=%d\n", 
	//			buff_list.buff_hdl, buff_list.data_offset, buff_list.data_length);
	
	buff_list.buff_hdl = (UWORD8 *)trout_buf;
	set_tx_dscr_submsdu_buff_info(tx_dscr, (UWORD32)(&buff_list), 0);
	
	set_tx_dscr_submsdu_info(tx_dscr, trout_dscr+TX_DSCR_LEN);
	
	//printk("sub_msdu_info1: buff_hdl=0x%x, data_offset=%d, data_len=%d\n", 
	//			buff_list.buff_hdl, buff_list.data_offset, buff_list.data_length);
	if((buff_list.data_offset + TX_DSCR_BUFF_SZ) > COEX_SELF_CTS_NULL_DATA_SIZE)
	{
		g_trout_self_cts_null_data_buf = NULL;

		pkt_mem_free(msa);
		mem_free((mem_handle_t *)g_shared_dscr_mem_handle, (void *)tx_dscr);
		TROUT_DBG4("Err: null date length is too large!\n");
		return;
	}
	
	TROUT_DBG4("%s: write null data config to trout!\n", __func__);
	host_write_trout_ram((void *)trout_buf, (void *)msa, buff_list.data_offset);	//write pkt content.
	host_write_trout_ram((void *)trout_dscr, (void *)tx_dscr, TX_DSCR_BUFF_SZ);	//write tx dscr.

	//printk("after covent:\n");
	//host_read_trout_ram(tmp_dscr, (void *)trout_dscr, TX_DSCR_BUFF_SZ);
	//hex_dump("trout_dscr", tmp_dscr, TX_DSCR_BUFF_SZ);

	//host_read_trout_ram(tmp_pkt, (void *)trout_buf, frame_len);
	//hex_dump("trout_pkt", tmp_pkt, frame_len);

	pkt_mem_free(msa);
	mem_free((mem_handle_t *)g_shared_dscr_mem_handle, (void *)tx_dscr);
}


void init_coex_ps_null_data(UWORD8 psm, BOOL_T is_qos, UWORD8 priority)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};
    UWORD8      frame_len = MAC_HDR_LEN;
    UWORD8      serv_class = NORMAL_ACK;
	UWORD8      phy_rate = 0;
	qmu_tx_handle_t *tx_handle = NULL;
	buffer_desc_t buff_list;
	UWORD32 trout_dscr, trout_buf;
	UWORD8 pr;

	//dbg.
	//UWORD8 tmp_dscr[TX_DSCR_BUFF_SZ];
	//UWORD8 tmp_pkt[128];

	if(g_trout_ps_null_data_buf != NULL)
		return;

	se = (sta_entry_t *)find_entry(mget_bssid());
    if(NULL == se)
    {
        TROUT_DBG3("%s: Err, send null data in not associated stage!\n", __func__);
        /* This is an exception case and should never occur */
        return;
    }
	
    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        TROUT_DBG4("%s: No Memory for NULL frame\n", __func__);
        return;
    }

    /* Set the Frame Control field of the NULL frame. */
#ifdef MAC_WMM
    if(is_qos == BTRUE)
    {
        set_frame_control(msa, (UWORD16)QOS_NULL_FRAME);
        q_num = get_txq_num(priority);
    }
    else
#endif /* MAC_WMM */
    {
        set_frame_control(msa, (UWORD16)NULL_FRAME);
        q_num       = HIGH_PRI_Q;
    }

    set_pwr_mgt(msa, (UWORD8)psm);
    set_to_ds(msa, 1);

    if(psm == 0)
        reset_machw_ps_pm_tx_bit();
    else
        set_machw_ps_pm_tx_bit();

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());

    if(is_qos == BTRUE)
    {
		set_qos_control(msa, priority, serv_class);
        frame_len += 2;
    }

    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
    if(tx_dscr == NULL)
    {
		TROUT_DBG3("No Mem for NULL TX DSCR \n");
        /* Free the memory allocated for the buffer */
        pkt_mem_free(msa);
        return;
    }

    /* Set the transmit rate from the station entry */
    //tx_rate = get_tx_rate_to_sta(se);
	tx_rate = 24;	//use permanent speed of 24Mbps.
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    phy_rate = get_phy_rate(tx_rate);

	/*junbin.wang add for long preamble.20131118*/
	pr = get_preamble(phy_rate);
	if(IS_RATE_11B(phy_rate) == BTRUE)
		phy_rate |= (BIT2 & (pr << 2));
    /* If AUTORATE_FEATURE is not defined or retransmission auto rate is not */
    /* enabled set all retry rates to the transmit rate                      */
    retry_set[0] = (phy_rate << 24) | (phy_rate << 16) | (phy_rate << 8) | (phy_rate);
    retry_set[1] = retry_set[0];

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, serv_class, phy_tx_mode, retry_set);
    //set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);	//adjust pkt length here!!!
    set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);
    //set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 2300);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

	//printk("before covent:\n");
	//hex_dump("tx_dscr", tx_dscr, TX_DSCR_BUFF_SZ);
	//hex_dump("tx_pkt", msa, frame_len);

	tx_handle = &g_q_handle.tx_handle;
	g_trout_ps_null_data_buf = 
		(UWORD32 *)(tx_handle->tx_mem_end + COEX_SLOT_INFO_SIZE + COEX_SELF_CTS_NULL_DATA_SIZE);

	trout_dscr = (UWORD32)g_trout_ps_null_data_buf;
	trout_buf = trout_dscr + TX_DSCR_BUFF_SZ;
	printk("%s: dscr=0x%x, buf=0x%x\n", __func__, trout_dscr, trout_buf);
	
	set_tx_dscr_buffer_addr(tx_dscr, trout_buf);
	set_tx_dscr_host_dscr_addr(tx_dscr, 0);
	set_tx_dscr_next_addr(tx_dscr, 0);
	
	get_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr, (UWORD32)(&buff_list), 0);
	//printk("sub_msdu_info0: buff_hdl=0x%x, data_offset=%d, data_len=%d\n", 
	//			buff_list.buff_hdl, buff_list.data_offset, buff_list.data_length);
	
	buff_list.buff_hdl = (UWORD8 *)trout_buf;
	set_tx_dscr_submsdu_buff_info(tx_dscr, (UWORD32)(&buff_list), 0);
	
	set_tx_dscr_submsdu_info(tx_dscr, trout_dscr+TX_DSCR_LEN);
	
	//printk("sub_msdu_info1: buff_hdl=0x%x, data_offset=%d, data_len=%d\n", 
	//			buff_list.buff_hdl, buff_list.data_offset, buff_list.data_length);
	if((buff_list.data_offset + TX_DSCR_BUFF_SZ) > COEX_PS_NULL_DATA_SIZE)
	{
		g_trout_ps_null_data_buf = NULL;

		pkt_mem_free(msa);
		mem_free((mem_handle_t *)g_shared_dscr_mem_handle, (void *)tx_dscr);
		TROUT_DBG4("Err: null date length is too large!\n");
		return;
	}
	
	TROUT_DBG4("%s: write null data config to trout!\n", __func__);
	host_write_trout_ram((void *)trout_buf, (void *)msa, buff_list.data_offset);	//write pkt content.
	host_write_trout_ram((void *)trout_dscr, (void *)tx_dscr, TX_DSCR_BUFF_SZ);	//write tx dscr.

	//printk("after covent:\n");
	//host_read_trout_ram(tmp_dscr, (void *)trout_dscr, TX_DSCR_BUFF_SZ);
	//hex_dump("trout_dscr", tmp_dscr, TX_DSCR_BUFF_SZ);

	//host_read_trout_ram(tmp_pkt, (void *)trout_buf, frame_len);
	//hex_dump("trout_pkt", tmp_pkt, frame_len);

	pkt_mem_free(msa);
	mem_free((mem_handle_t *)g_shared_dscr_mem_handle, (void *)tx_dscr);
}


void  exit_from_coexist_mode(void)
{
	UWORD32 *tx_start = NULL;
	UWORD32 tmp32[36];
	UWORD32 *tx_dscr = NULL;
	UWORD8 q_num;
	int slot;

	host_read_trout_ram((void *)tmp32, (void *)(tx_shareram[0].slot_info), sizeof(UWORD32));
	tx_dscr = (UWORD32 *)(tmp32[0]);

	TROUT_DBG4("%s: tx_dscr_start=0x%p\n", __func__, tx_dscr);
	
	while(tx_dscr != NULL)
	{
		host_read_trout_ram((void *)tmp32, (void *)tx_dscr, sizeof(UWORD32) * 36);
		TROUT_DBG4("tx_dscr: 0x%p, status: %d\n", tx_dscr, ((tmp32[0] >> 29) & 0x3));
		if((tx_start == NULL) && (((tmp32[0] >> 29) & 0x3) == (UWORD32)(PENDING)))
		{
			tx_start = tx_dscr;
			q_num = tmp32[32] & 0xFF;
			
			TROUT_DBG4("exit from coex: q_num=%d, tx_start=0x%p\n", q_num, tx_start);
			break;
		}
		tx_dscr = (UWORD32 *)(tmp32[4]);
	}

	//debug use.
	for(slot=0; slot<TX_SHARERAM_SLOTS; slot++)
	{
		if((tx_shareram[slot].state == TX_SHARERAM_BUSY) && (tx_shareram[slot].pkt_num > 0))
		{
			TROUT_DBG4("after exit from coex, slot%d pkt_num=%d, state=%d\n", 
						slot, tx_shareram[slot].pkt_num, tx_shareram[slot].state);
		}
    }

	enable_txptr_update();
	
	if(tx_start != NULL)
	{
		if(q_num >= NUM_MAX_EDCA_Q)
		{
			TROUT_DBG4("Err: invalid q_num(%d)\n", q_num);
			return;
		}

		TROUT_DBG4("exit from coex, tx not complete, restart it...\n");
		host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)tx_dscr)), 
									(UWORD32)(g_qif_table[q_num].addr));
	}
}



void coex_null_data_init(void)
{
	init_coex_self_cts_null_data(STA_ACTIVE, BFALSE, 0);	//use txq 0.
	init_coex_ps_null_data(STA_ACTIVE, BFALSE, 0);
}

#endif	/* IBSS_BSS_STATION_MODE */

