/* linux/arch/arm/mach-sc8800s/dma.c
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <mach/dma.h>

static void dma_channel_start(int dma_chn, int on_off);
static void dma_channel_set_software_req(int dma_chn, int on_off);

struct sprd_irq_handler {
    void (*handler)(int dma, void *dev_id);
    void *dev_id;
    u32 dma_uid;//wong, add
    u32 used_ago;//marking the channel used before new API done
};

static inline void dma_set_reg(u32 val, u32 reg){ 
    __raw_writel(val, reg);
}

static inline u32 dma_get_reg(u32 reg){
    return __raw_readl(reg);
}


//static struct sprd_irq_handler sprd_irq_handlers[DMA_CH_NUM];//original code
static struct sprd_irq_handler sprd_irq_handlers[DMA_CHN_NUM];

static irqreturn_t sprd_dma_irq(int irq, void *dev_id)
{ 
    u32 irq_status = __raw_readl(DMA_INT_STS);
    while (irq_status) {
        int i = 31 - __builtin_clz(irq_status);
        irq_status &= ~(1<<i);
#if 1 || SC88XX_PCM_DMA_SG_CIRCLE
        dma_reg_write(DMA_TRANSF_INT_CLR, i, 1, 1);
        dma_reg_write(DMA_BURST_INT_CLR, i, 1, 1);
        dma_reg_write(DMA_LISTDONE_INT_CLR, i, 1, 1);
#endif
        if (sprd_irq_handlers[i].handler)
            sprd_irq_handlers[i].handler(i, sprd_irq_handlers[i].dev_id);
        else printk(KERN_ERR "DMA channel %d needs handler!\n", i);
/* #if 1 || SC88XX_PCM_DMA_SG_CIRCLE
        dma_reg_write(DMA_TRANSF_INT_CLR, i, 1, 1);
        dma_reg_write(DMA_BURST_INT_CLR, i, 1, 1);
        dma_reg_write(DMA_LISTDONE_INT_CLR, i, 1, 1);
#endif */
    }
    return IRQ_HANDLED;
}

int sprd_request_dma(int ch_id, void (*irq_handler)(int, void *), void *data)
{
    unsigned long flags;
    local_irq_save(flags);
    if (sprd_irq_handlers[ch_id].handler) {
        printk(KERN_WARNING "%s: dma channel %d is busy\n", __func__, ch_id);
	local_irq_restore(flags);
        return -EBUSY;
    }
    sprd_irq_handlers[ch_id].handler = irq_handler;
    sprd_irq_handlers[ch_id].dev_id = data;
    local_irq_restore(flags);
    return 0;
}
EXPORT_SYMBOL_GPL(sprd_request_dma);

void sprd_free_dma(int ch_id){
    sprd_irq_handlers[ch_id].handler = NULL;
}

//wong
void dma_channel_handlers_init(void){
     int i;
     for(i=DMA_CHN_MIN; i<DMA_CHN_NUM; i++){
        sprd_irq_handlers[i].handler = NULL;
        sprd_irq_handlers[i].dev_id = NULL;
        sprd_irq_handlers[i].dma_uid = 0;
        sprd_irq_handlers[i].used_ago = 0;
     }
}

void sprd_dma_check_channel(void){
     int i;
     for(i=0; i<DMA_CHN_NUM; i++){
        if(sprd_irq_handlers[i].handler == NULL){
	  pr_debug("=== dma channel:%d is not occupied ====\n", i);
	}
     }
     for(i=0; i<DMA_CHN_NUM; i++){
	  pr_debug("=== sprd_irq_handlers[%d].handler:%p ====\n", i, sprd_irq_handlers[i].handler);
	  pr_debug("=== sprd_irq_handlers[%d].dma_uid:%u ====\n", i, sprd_irq_handlers[i].dma_uid);
	  pr_debug("=== sprd_irq_handlers[%d].used_ago:%u ====\n", i, sprd_irq_handlers[i].used_ago);
     }
} 
EXPORT_SYMBOL_GPL(sprd_dma_check_channel);

/**
 * dma_set_uid: dedicate uid to a dma channel
 * @dma_chn: dma channel id, in range 0~31
 * @dma_uid: dma uid
 **/
void dma_set_uid(u32 dma_chn, u32 dma_uid){
       if(dma_chn > DMA_CHN_MAX){
          pr_warning("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }
       
       u32 dma_chn_uid_reg = DMA_CHN_UID_BASE + DMA_UID_UNIT*(dma_chn/DMA_UID_UNIT);
       int chn_uid_shift = DMA_UID_SHIFT_STP*(dma_chn%DMA_UID_UNIT);
       
       __raw_writel( (~(DMA_UID_MASK<<chn_uid_shift))&__raw_readl(dma_chn_uid_reg), dma_chn_uid_reg);
       dma_uid = dma_uid << chn_uid_shift; 
       dma_uid |= __raw_readl(dma_chn_uid_reg);
       __raw_writel(dma_uid, dma_chn_uid_reg);
       pr_debug("**** dma_chn_uid_reg:0x%x, 0x%x ****\n", dma_chn_uid_reg, __raw_readl(dma_chn_uid_reg) );

}


/**
 * dma_check_channel: check the whole dma channels according to uid, return the one not occupied
 * @uid: dma uid
 **/
u32 dma_check_channel(u32 uid){
    u32 chn;
    
    //we should set uid 0 when memory to memory(software request), NOT LIKE CHIP SPEC!!
    if( (uid<DMA_UID_MIN)||(uid>DMA_UID_MAX) ){
       printk("!!!! DMA UID:%u IS Beyond Valid Range %d~%d !!!!\n", uid, DMA_UID_MIN, DMA_UID_MAX);
       return -1;
    }else if(uid == DMA_UID_SOFTWARE){
       for(chn=DMA_CHN_SOFTWARE_START; chn<=DMA_CHN_SOFTWARE_END; chn++){
          if((sprd_irq_handlers[chn].handler==NULL) && (sprd_irq_handlers[chn].used_ago!=1)){ 
             return chn;
          }
       }
    }else{
       //suppose the last 28 channels were dedicated for hardware
       for(chn=DMA_CHN_HARDWARE_START; chn<=DMA_CHN_HARDWARE_END; chn++){
          if((sprd_irq_handlers[chn].handler==NULL) && (sprd_irq_handlers[chn].used_ago!=1)){ 
             return chn;
          }
       }
    }
    return -1; 
    
}

void dma_channel_int_clr(u32 chn){
     __raw_bits_or(1<<chn, DMA_BURST_INT_CLR);
     __raw_bits_or(1<<chn, DMA_TRANSF_INT_CLR);
     __raw_bits_or(1<<chn, DMA_LISTDONE_INT_CLR);
}
void dma_channel_workmode_clr(chn_id){
     __raw_bits_and(~(1<<chn_id), DMA_LINKLIST_EN);
     __raw_bits_and(~(1<<chn_id), DMA_SOFTLINK_EN);
}

/***
* sprd_dma_request: request dma channel resources, return the valid channel number
* @uid: dma uid
* @irq_handler: irq_handler of dma users
* @data: parameter pass to irq_handler
***/
int sprd_dma_request(u32 uid, void (*irq_handler)(int, void *), void *data){
    unsigned long flags;
    int ch_id;
    local_irq_save(flags);
    ch_id = dma_check_channel(uid);
    if(ch_id<DMA_CHN_MIN){
       printk("!!!! DMA UID:%u Is Already Requested !!!!\n", uid);
       local_irq_restore(flags);
       return -1;
    }	    
    pr_debug("++++ requested dma channel:%u +++", ch_id);
    sprd_irq_handlers[ch_id].handler = irq_handler;
    sprd_irq_handlers[ch_id].dev_id = data;
    sprd_irq_handlers[ch_id].dma_uid = uid;
    sprd_irq_handlers[ch_id].used_ago = 1;
    local_irq_restore(flags);

    dma_channel_start(ch_id, OFF); 
    dma_channel_set_software_req(ch_id, OFF);
    dma_channel_int_clr(ch_id);
    dma_channel_workmode_clr(ch_id);
    dma_set_uid(ch_id, uid); 
    return ch_id;
}
EXPORT_SYMBOL_GPL(sprd_dma_request);

/**
 * sprd_dma_free: free the occupied dma channel
 * @chn_id: dma channel occupied 
 */
void sprd_dma_free(u32 chn_id)
{
    if(chn_id > DMA_CHN_MAX){
        printk("!!!! dma channel id is out of range:%u~%u !!!!\n", DMA_CHN_MIN, DMA_CHN_MAX);
	return;
    }
    dma_channel_start(chn_id, OFF);
    dma_channel_set_software_req(chn_id, OFF);
    sprd_dma_set_irq_type(chn_id, BLOCK_DONE, OFF);
    sprd_dma_set_irq_type(chn_id, TRANSACTION_DONE, OFF);
    sprd_dma_set_irq_type(chn_id, LINKLIST_DONE, OFF);
    sprd_irq_handlers[chn_id].handler = NULL;
    sprd_irq_handlers[chn_id].dev_id = NULL;
    sprd_irq_handlers[chn_id].dma_uid = DMA_UID_SOFTWARE;//default UID value
    sprd_irq_handlers[chn_id].used_ago = 0;//not occupied
    dma_channel_workmode_clr(chn_id);
    dma_set_uid(chn_id, DMA_UID_SOFTWARE);//default UID value 
}
EXPORT_SYMBOL_GPL(sprd_dma_free);

//wong add
//
static void dma_channel_start(int dma_chn, int on_off){
       switch(on_off){
	     case ON:
	         __raw_bits_and(~(1<<dma_chn), DMA_CHx_DIS);
	         __raw_bits_or(1<<dma_chn, DMA_CHx_EN);
                 break;
	     case OFF:
	         __raw_bits_and(~(1<<dma_chn), DMA_CHx_EN);
	         __raw_bits_or((1<<dma_chn), DMA_CHx_DIS);
                 break;
	     default:
	         printk("??? dma_channel_start??? what you mean?\n");
       }

       return;
}
static void dma_channel_set_software_req(int dma_chn, int on_off){
       if(dma_chn > DMA_CHN_MAX){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }

       switch(on_off){
	  case ON:
	    __raw_bits_or(1<<dma_chn, DMA_SOFT_REQ);
            break;
	  case OFF:
	    __raw_bits_and(~(1<<dma_chn), DMA_SOFT_REQ);
	    break;
	  default:
	    printk("??? channel:%d, DMA_SOFT_REQ, ON or OFF \n", dma_chn);
       }

       return;
}
/**
 * sprd_dma_channel_start: start one dma channel transfer
 * @chn_id: dma channel id, in range 0 to 31
 **/
void sprd_dma_channel_start(u32 chn_id){
     u32 uid;
     
     if( chn_id > DMA_CHN_MAX ){
         printk("!!! channel id:%u out of range %u~%u !!!\n", chn_id, DMA_CHN_MIN, DMA_CHN_MAX);
         return;
     }
     dma_channel_start(chn_id, ON);
     uid = sprd_irq_handlers[chn_id].dma_uid;
     if(uid == DMA_UID_SOFTWARE){
       dma_channel_set_software_req(chn_id, ON);
     }
//original idea, but uid must be set 0 if software request dma,
#if 0
     if((uid<22)||(uid==29)||(uid==30)){ //memory to devices or reserse
       //__raw_bits_or(1<<chn_id, DMA_CHx_EN);
     }else{
       dma_set_uid(chn_id, DMA_SOFT0);//must be 0, NOT LIKE THE SPEC! 
       dma_channel_set_software_req(chn_id, ON);
       printk("==== channel number:%d, uid:%d ====\n", chn_id, uid);
     }
#endif
     return;
}
EXPORT_SYMBOL_GPL(sprd_dma_channel_start);

/**
 * sprd_dma_channel_start: stop one dma channel transfer
 * @chn_id: dma channel id, in range 0 to 31
 **/
void sprd_dma_channel_stop(u32 chn_id){
     u32 uid;

     if( chn_id > DMA_CHN_MAX ){
         printk("!!! channel id:%u out of range %u~%u !!!\n", chn_id, DMA_CHN_MIN, DMA_CHN_MAX);
         return;
     }
     
     dma_channel_start(chn_id, OFF);
     uid = sprd_irq_handlers[chn_id].dma_uid;
     if(uid == DMA_UID_SOFTWARE){
       dma_channel_set_software_req(chn_id, OFF);
     }
//original idea, but uid must be set 0 if software request dma,
#if 0     
     if((uid<22)||(uid==29)||(uid==30)){ //memory to devices or reserse
       //__raw_bits_and(~(1<<chn_id), DMA_CHx_EN);
     }else{
       dma_channel_set_software_req(chn_id, OFF);
       //__raw_bits_and(~(1<<chn_id), DMA_SOFT_REQ);//memory to memory
     }
#endif     
     return;
}
EXPORT_SYMBOL_GPL(sprd_dma_channel_stop);

/**
 * sprd_dma_set_irq_type: enable or disable dma interrupt
 * @dma_chn: dma channel id, in range 0 to 31
 * @dma_done_type: BLOCK_DONE, TRANSACTION_DONE or LINKLIST_DONE
 * @on_off: ON:enable interrupt,
 *          OFF:disable interrupt
 **/
void sprd_dma_set_irq_type(u32 dma_chn, dma_done_type irq_type, u32 on_off){
      if(dma_chn > DMA_CHN_MAX){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }

       switch(irq_type){
	  case LINKLIST_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_LISTDONE_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_LISTDONE_INT_EN);
		      break;

		      default:
		         printk(" LLD_MODE, INT_EN ON OR OFF???\n");
		       
	      }	      
	  break;
	  	         
	  case BLOCK_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_BURST_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_BURST_INT_EN);
		      break;

		      default:
		         printk(" BURST_MODE, INT_EN ON OR OFF???\n");
	      }	      
	  break;	         
	  
	  case TRANSACTION_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_TRANSF_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_TRANSF_INT_EN);
		      break;

		      default:
		         printk(" TRANSACTION_MODE, INT_EN ON OR OFF???\n");
	      }	      
	  break;
	  
	  default:
	      printk("??? WHICH IRQ TYPE YOU SELECT ???\n");	         
       }
}
EXPORT_SYMBOL_GPL(sprd_dma_set_irq_type);

/**
 * sprd_dma_set_chn_pri: set dma channel priority
 * @chn: dma channel
 * @pri: channel priority, in range lowest 0 to highest 3,
 */
void sprd_dma_set_chn_pri(u32 chn,  u32 pri){
     u32 shift;
     u32 reg_val;
     if( chn > DMA_CHN_MAX ){
         printk("!!! channel id:%u out of range %u~%u !!!\n", chn, DMA_CHN_MIN, DMA_CHN_MAX);
         return;
     }
     if( pri > DMA_MAX_PRI ){
         printk("!!! channel priority:%u out of range %d~%d !!!!\n", pri, DMA_MIN_PRI, DMA_MAX_PRI);
	 return;
     }
     shift = chn%16;
     switch(chn/16){
	 case 0:
             reg_val = dma_get_reg(DMA_PRI_REG0);
	     reg_val &= ~(DMA_MAX_PRI<<(2*shift));  
	     reg_val |= pri<<(2*shift);
	     dma_set_reg(reg_val, DMA_PRI_REG0);
	     break;
	 case 1:
             reg_val = dma_get_reg(DMA_PRI_REG1);
	     reg_val &= ~(DMA_MAX_PRI<<(2*shift));  
	     reg_val |= pri<<(2*shift);
	     dma_set_reg(reg_val, DMA_PRI_REG1);
	     break;
         default:
             printk("!!!! WOW, WOW, WOW, chn:%u, pri%u !!!\n", chn, pri);	     
     }
     return;
}
EXPORT_SYMBOL_GPL(sprd_dma_set_chn_pri);

/**
 * dma_channel_config: configurate a dma channel
 * @chn: dma channel id
 * @desc: dma channel configuration descriptor
 */

void dma_channel_config(u32 chn, struct sprd_dma_channel_desc *desc){
    u32 chn_cfg = 0;
    u32 chn_elem_postm = 0;
    u32 chn_src_blk_postm = 0;
    u32 chn_dst_blk_postm = 0;
   
     chn_cfg |= ( desc->cfg_swt_mode_sel   | 
                  desc->cfg_src_data_width |
                  desc->cfg_dst_data_width |
                  desc->cfg_req_mode_sel   |
                  desc->cfg_src_wrap_en    |
                  desc->cfg_dst_wrap_en    |
                  (desc->cfg_blk_len&CFG_BLK_LEN_MASK)    
	          );
    chn_elem_postm = ((desc->src_elem_postm & SRC_ELEM_POSTM_MASK)<<SRC_ELEM_POSTM_SHIFT) |
                     (desc->dst_elem_postm & DST_ELEM_POSTM_MASK);
    chn_src_blk_postm = (desc->src_burst_mode)|(desc->src_blk_postm & SRC_BLK_POSTM_MASK);
    chn_dst_blk_postm = (desc->dst_burst_mode)|(desc->dst_blk_postm & DST_BLK_POSTM_MASK);

    dma_set_reg(chn_cfg, DMA_CHx_CFG0(chn) );
    dma_set_reg(desc->total_len, DMA_CHx_CFG1(chn) );
    dma_set_reg(desc->src_addr, DMA_CHx_SRC_ADDR(chn) );
    dma_set_reg(desc->dst_addr, DMA_CHx_DEST_ADDR(chn) );
    dma_set_reg(desc->llist_ptr, DMA_CHx_LLPTR(chn) );
    dma_set_reg(chn_elem_postm, DMA_CHx_SDEP(chn) );
    dma_set_reg(chn_src_blk_postm, DMA_CHx_SBP(chn) );
    dma_set_reg(chn_dst_blk_postm, DMA_CHx_DBP(chn) );
}

/**
 * sprd_dma_channel_config: configurate dma channel
 * @chn: dma channel 
 * @work_mode: dma work mode, normal mode as default
 * @dma_cfg: dma channel configuration descriptor
 **/
void sprd_dma_channel_config(u32 chn, dma_work_mode work_mode, struct sprd_dma_channel_desc *dma_cfg){
    switch(work_mode){
        case DMA_NORMAL:
	     break;
	case DMA_LINKLIST:
	     __raw_bits_and(~(1<<chn), DMA_SOFTLINK_EN);
	     __raw_bits_or(1<<chn, DMA_LINKLIST_EN);
 	     break;
	case DMA_SOFTLIST:
	     __raw_bits_and(~(1<<chn), DMA_LINKLIST_EN);
	     __raw_bits_or(1<<chn, DMA_SOFTLINK_EN);
 	     break;
	default:
	     printk("???? Unsupported Work Mode You Seleced ????\n");
	     return;
    }
    dma_channel_config(chn, dma_cfg );
}
EXPORT_SYMBOL_GPL(sprd_dma_channel_config);

/**
 * sprd_dma_softlist_config: configurate dma softlist mode
 * @softlist_desc: dma softlist mode  configuration descriptor
 **/
void sprd_dma_softlist_config(struct sprd_dma_softlist_desc *softlist_desc){
     u32 dma_softlist_sts;
     if(softlist_desc){ 
       dma_softlist_sts=((softlist_desc->softlist_req_ptr&SOFTLIST_REQ_PTR_MASK)<<SOFTLIST_REQ_PTR_SHIFT) |                       (softlist_desc->softlist_cnt & SOFTLIST_CNT_MASK);
	dma_set_reg(softlist_desc->softlist_base_addr, DMA_SOFTLIST_BASEADDR);
	dma_set_reg(softlist_desc->softlist_size, DMA_SOFTLIST_SIZE);
	dma_set_reg(softlist_desc->softlist_cnt_incr, DMA_SOFTLIST_CMD);
	dma_set_reg(dma_softlist_sts, DMA_SOFTLIST_STS);
	dma_set_reg(softlist_desc->softlist_base_addr, DMA_SOFTLIST_BASEADDR);
     }
     return;
}
EXPORT_SYMBOL_GPL(sprd_dma_softlist_config);

/**
 * sprd_dma_softlist_config: configurate dma wrap address
 * @wrap_addr: dma wrap address descriptor
 **/
void sprd_dma_wrap_addr_config(struct sprd_dma_wrap_addr *wrap_addr){
     if(wrap_addr){
         dma_set_reg(wrap_addr->wrap_start_addr, DMA_WRAP_START);
         dma_set_reg(wrap_addr->wrap_end_addr, DMA_WRAP_END);
     }
     return;
}
EXPORT_SYMBOL_GPL(sprd_dma_wrap_addr_config);

void dma_mark_used_channels(void){
     sprd_irq_handlers[0].used_ago = 1;
     sprd_irq_handlers[DMA_VB_DA0].used_ago = 1;
     sprd_irq_handlers[DMA_VB_DA1].used_ago = 1;
     sprd_irq_handlers[DMA_VB_AD0].used_ago = 1;
     sprd_irq_handlers[DMA_VB_AD1].used_ago = 1;
     sprd_irq_handlers[DMA_SPI_TX].used_ago = 1;
     sprd_irq_handlers[DMA_SPI_RX].used_ago = 1;
     sprd_irq_handlers[22].used_ago = 1;//for NAND, software request
}
//wong add
//

int sprd_irq_handler_ready(int ch_id)
{
    return sprd_irq_handlers[ch_id].handler != NULL;
}
EXPORT_SYMBOL_GPL(sprd_irq_handler_ready);

void sprd_dma_setup(sprd_dma_ctrl *ctrl)
{
    int ch_id = ctrl->ch_id;
    u32 ch_base = DMA_CHx_CTL_BASE + (ch_id * 0x20);
    int interrupt_type = ctrl->interrupt_type; // TRANS_DONE_EN;
    sprd_dma_desc *dma_desc = ctrl->dma_desc;
    dma_addr_t dma_desc_phy = ctrl->dma_desc_phy;
    u32 modes = ctrl->modes; // DMA_LINKLIST;
    u32 wrap_addr_start = 0, wrap_addr_end = 0;
    u32 softlist_size = 0, softlist_baseaddr = 0, softlist_cmd = 0;
	
    // DMA Channel Control
    dma_reg_write(DMA_CHx_EN, ch_id, 0, 1); // stop hard transfer for hard channel
    dma_reg_write(DMA_SOFT_REQ, ch_id, 0, 1); // stop soft transfer for soft channel

    // Clear dma mode
    dma_reg_write(DMA_LINKLIST_EN, ch_id, 0, 1); // disable linklist
    dma_reg_write(DMA_SOFTLINK_EN, ch_id, 0, 1); // disable softlink

    // Clear all interrupt type
    dma_reg_write(DMA_LISTDONE_INT_EN, ch_id, 0, 1);
    dma_reg_write(DMA_BURST_INT_EN, ch_id, 0, 1);
    dma_reg_write(DMA_TRANSF_INT_EN, ch_id, 0, 1);

    // Clear all interrupt status
    dma_reg_write(DMA_TRANSF_INT_CLR, ch_id, 1, 1);
    dma_reg_write(DMA_BURST_INT_CLR, ch_id, 1, 1);
    dma_reg_write(DMA_LISTDONE_INT_CLR, ch_id, 1, 1);

    if (interrupt_type & LLIST_DONE_EN)
        __raw_bits_or(1<<ch_id, DMA_LISTDONE_INT_EN);
    if (interrupt_type & BURST_DONE_EN)
        __raw_bits_or(1<<ch_id, DMA_BURST_INT_EN);
    if (interrupt_type & TRANS_DONE_EN)
        __raw_bits_or(1<<ch_id, DMA_TRANSF_INT_EN);

    if (modes & DMA_WRAP) {
        __raw_writel(wrap_addr_start, DMA_WRAP_START);
        __raw_writel(wrap_addr_end, DMA_WRAP_END);
    }

    // set user id
    dma_reg_write(DMA_CHN_UID_BASE + (ch_id & ~0x03), 
                  (ch_id & 0x03) << 3,
                  (ch_id >= DMA_CHN_SOFTWARE_START && ch_id <= DMA_CHN_SOFTWARE_END) ? DMA_SOFT0:ch_id,
                  0x1f);

    if (modes & DMA_LINKLIST) {
#if 0
        __raw_writel(dma_desc->cfg  , ch_base + 0x00);
        __raw_writel(dma_desc->tlen , ch_base + 0x04);
        __raw_writel(dma_desc->dsrc , ch_base + 0x08);
        __raw_writel(dma_desc->ddst , ch_base + 0x0c);
        __raw_writel(dma_desc->llptr, ch_base + 0x10);
        __raw_writel(dma_desc->pmod , ch_base + 0x14);
        __raw_writel(dma_desc->sbm  , ch_base + 0x18);
        __raw_writel(dma_desc->dbm  , ch_base + 0x1c);
#else
        __raw_writel(0, ch_base + 0x00);
        __raw_writel(0, ch_base + 0x04);
        __raw_writel(dma_desc->dsrc, ch_base + 0x08);
        __raw_writel(0, ch_base + 0x0c);
        __raw_writel(dma_desc_phy, ch_base + 0x10);
        __raw_writel(0, ch_base + 0x15);
        __raw_writel(0, ch_base + 0x18);
        __raw_writel(0, ch_base + 0x1c);
#endif
        dma_reg_write(DMA_LINKLIST_EN, ch_id, 1, 1); // enable channel ch_id linklist mode dma
        return;
    } else if (modes & DMA_SOFTLIST) {
        dma_reg_write(DMA_SOFTLINK_EN, ch_id, 1, 1); // enable softlink
        // Size of the request list. When the DMA reaches the end of the list, 
        // it starts from the beginning again if SOFT_LIST_CNT != 0. 
        // For example, if SOFTLIST_SIZE is 5, then SW can append maximum (N) 5 requests to DMA. 
        // And if SOFTLIST_EN is active, this value cannot be zero. 
        __raw_writel(softlist_size, DMA_SOFTLIST_SIZE);
        __raw_writel(softlist_baseaddr, DMA_SOFTLIST_BASEADDR);
        __raw_writel(softlist_cmd, DMA_SOFTLIST_CMD);
        return;
    }
    // DMA_NORMAL process
    __raw_writel(dma_desc->cfg  , ch_base + 0x00);
    __raw_writel(dma_desc->tlen , ch_base + 0x04);
    __raw_writel(dma_desc->dsrc , ch_base + 0x08);
    __raw_writel(dma_desc->ddst , ch_base + 0x0c);
    __raw_writel(dma_desc->llptr, ch_base + 0x10);
    __raw_writel(dma_desc->pmod , ch_base + 0x14);
    __raw_writel(dma_desc->sbm  , ch_base + 0x18);
    __raw_writel(dma_desc->dbm  , ch_base + 0x1c);
}
EXPORT_SYMBOL_GPL(sprd_dma_setup);

void sprd_dma_setup_cfg(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int autodma_src,
            int autodma_dst,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen)
{
    sprd_dma_desc *dma_desc = ctrl->dma_desc;
    int autodma_burst_step_src = autodma_burst_mod_src & DMA_BURST_STEP_ABS_SIZE_MASK;
    int autodma_burst_step_dst = autodma_burst_mod_dst & DMA_BURST_STEP_ABS_SIZE_MASK;
    int pmod = 0; // src & dst element postm next block offset value is forced to 0 [luther.ge]
    int width = 0;

    ctrl->ch_id = ch_id;
    ctrl->modes = dma_modes;
    ctrl->interrupt_type = interrupt_type;

    if (src_data_width == 32) width |= DMA_SDATA_WIDTH32;
    else if (src_data_width == 16) width |= DMA_SDATA_WIDTH16;
    else width |= DMA_SDATA_WIDTH8;

    if (dst_data_width == 32) width |= DMA_DDATA_WIDTH32;
    else if (dst_data_width == 16) width |= DMA_DDATA_WIDTH16;
    else width |= DMA_DDATA_WIDTH8;

    if (autodma_src != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_src << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_src << 31; // inc or dec direction bit for block dma
    }
    if (autodma_dst != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_dst << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_dst << 15; // inc or dec direction bit for block dma
    }

    dma_desc->cfg = DMA_LIT_ENDIAN | width | DMA_REQMODE_TRANS | burst_size;
    dma_desc->tlen = tlen;
    dma_desc->dsrc = dsrc;
    dma_desc->ddst = ddst;
    dma_desc->llptr = 0;
    dma_desc->pmod = pmod;
    dma_desc->sbm = autodma_burst_step_src | (autodma_burst_mod_src & ~DMA_BURST_STEP_ABS_SIZE_MASK);
    dma_desc->dbm = autodma_burst_step_dst | (autodma_burst_mod_dst & ~DMA_BURST_STEP_ABS_SIZE_MASK);
}
EXPORT_SYMBOL_GPL(sprd_dma_setup_cfg);

void sprd_dma_setup_cfg_ext(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int autodma_src,
            int autodma_dst,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen)
{
    sprd_dma_desc *dma_desc = ctrl->dma_desc;
    int autodma_burst_step_src = autodma_burst_mod_src & DMA_BURST_STEP_ABS_SIZE_MASK;
    int autodma_burst_step_dst = autodma_burst_mod_dst & DMA_BURST_STEP_ABS_SIZE_MASK;
    int pmod = 0; // src & dst element postm next block offset value is forced to 0 [luther.ge]
    int width = 0;

    ctrl->ch_id = ch_id;
    ctrl->modes = dma_modes;
    ctrl->interrupt_type = interrupt_type;

    if (src_data_width == 32) width |= DMA_SDATA_WIDTH32;
    else if (src_data_width == 16) width |= DMA_SDATA_WIDTH16;
    else width |= DMA_SDATA_WIDTH8;

    if (dst_data_width == 32) width |= DMA_DDATA_WIDTH32;
    else if (dst_data_width == 16) width |= DMA_DDATA_WIDTH16;
    else width |= DMA_DDATA_WIDTH8;

    if (autodma_src != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_src << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_src << 31; // inc or dec direction bit for block dma
	pmod = 1<<16;
    }
    if (autodma_dst != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_dst << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_dst << 15; // inc or dec direction bit for block dma
	pmod = 1;
    }
	
    dma_desc->cfg = width | DMA_REQMODE_NORMAL | burst_size;
    dma_desc->tlen = tlen;
    dma_desc->dsrc = dsrc;
    dma_desc->ddst = ddst;
    dma_desc->llptr = 0;
    dma_desc->pmod = pmod;
    dma_desc->sbm = 0;//autodma_burst_step_src | (autodma_burst_mod_src & ~DMA_BURST_STEP_ABS_SIZE_MASK);
    dma_desc->dbm = 0;//autodma_burst_step_dst | (autodma_burst_mod_dst & ~DMA_BURST_STEP_ABS_SIZE_MASK);
}
EXPORT_SYMBOL_GPL(sprd_dma_setup_cfg_ext);



void sprd_dma_setup_cfg_pmod(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int endian,
            int autodma_src,
            int autodma_dst,
            int src_ele_step,
            int des_ele_step,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen)
{
    sprd_dma_desc *dma_desc = ctrl->dma_desc;
    int autodma_burst_step_src = autodma_burst_mod_src & DMA_BURST_STEP_ABS_SIZE_MASK;
    int autodma_burst_step_dst = autodma_burst_mod_dst & DMA_BURST_STEP_ABS_SIZE_MASK;
    int pmod = des_ele_step |(src_ele_step<<16);
    int width = 0;

    ctrl->ch_id = ch_id;
    ctrl->modes = dma_modes;
    ctrl->interrupt_type = interrupt_type;

    if (src_data_width == 32) width |= DMA_SDATA_WIDTH32;
    else if (src_data_width == 16) width |= DMA_SDATA_WIDTH16;
    else width |= DMA_SDATA_WIDTH8;

    if (dst_data_width == 32) width |= DMA_DDATA_WIDTH32;
    else if (dst_data_width == 16) width |= DMA_DDATA_WIDTH16;
    else width |= DMA_DDATA_WIDTH8;

    if (autodma_src != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_src << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_src << 31; // inc or dec direction bit for block dma
    }
    if (autodma_dst != DMA_NOCHANGE) {
        autodma_burst_step_src |= autodma_dst << 25; // inc or dec direction bit for burst dma
        // pmod |= autodma_dst << 15; // inc or dec direction bit for block dma
    }

    dma_desc->cfg = endian | width | DMA_REQMODE_TRANS | burst_size;
    dma_desc->tlen = tlen;
    dma_desc->dsrc = dsrc;
    dma_desc->ddst = ddst;
    dma_desc->llptr = 0;
    dma_desc->pmod = pmod;
    dma_desc->sbm = autodma_burst_step_src | (autodma_burst_mod_src & ~DMA_BURST_STEP_ABS_SIZE_MASK);
    dma_desc->dbm = autodma_burst_step_dst | (autodma_burst_mod_dst & ~DMA_BURST_STEP_ABS_SIZE_MASK);
}
EXPORT_SYMBOL_GPL(sprd_dma_setup_cfg_pmod);

static int sprd_dma_init(void)
{
    int ret;

    dma_reg_write(AHB_CTL0, 6, 1, 1); // DMA Enable
    __raw_writel(0, DMA_CHx_EN);
    __raw_writel(0, DMA_LINKLIST_EN);
    __raw_writel(0, DMA_SOFT_REQ);
    __raw_writel(0, DMA_PRI_REG0);
    __raw_writel(0, DMA_PRI_REG1);
    __raw_writel(-1,DMA_LISTDONE_INT_CLR);
    __raw_writel(-1,DMA_TRANSF_INT_CLR);
    __raw_writel(-1,DMA_BURST_INT_CLR);
    __raw_writel(0, DMA_LISTDONE_INT_EN);
    __raw_writel(0, DMA_TRANSF_INT_EN);
    __raw_writel(0, DMA_BURST_INT_EN);

    /*set hard/soft burst wait time*/
    dma_reg_write(DMA_CFG, 0, DMA_HARD_WAITTIME, 0xff);
    dma_reg_write(DMA_CFG,16, DMA_SOFT_WAITTIME, 0xffff);

    /*register dma irq handle to host*/
    ret = request_irq(IRQ_DMA_INT, sprd_dma_irq, 0, "sprd-dma", NULL);

    /*enable dma int*/
    if (ret == 0) {
        dma_reg_write(INT_IRQ_EN, 21, 1, 1);
        printk(KERN_INFO "request dma irq ok\n");
    } else printk(KERN_ERR "request dma irq failed %d\n", ret);

    //wong
    //initialize the sprd_irq_handlers
    dma_channel_handlers_init( ); 
    //mark channels used before new API setup, avoid these channels to be requested dynamicly
    dma_mark_used_channels( );

    return ret;
}

arch_initcall(sprd_dma_init);

MODULE_DESCRIPTION("SPRD DMA Module");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
