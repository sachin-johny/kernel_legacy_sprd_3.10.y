/******************************************************************************
 ** File Name:      ldo_drv.c                                             *
 ** Author:         Yi.Qiu                                                 *
 ** DATE:           01/09/2009                                                *
 ** Copyright:      2007 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    This file defines the basic function for ldo management.  *
 ******************************************************************************/

/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 01/09/2009     Yi.Qiu        Create.                                   *
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/
#include <linux/bug.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>

#include <mach/regs_global.h>
#include <mach/bits.h>
#include <mach/ldo.h>

#define LDO_INVALID_REG	0xFFFFFFFF
#define LDO_INVALID_BIT		0xFFFFFFFF

#ifndef NULL
#define NULL 0
#endif

#define SCI_ASSERT(condition) BUG_ON(!(condition))  
#define SCI_PASSERT(condition, format...)  \
	do {		\
		if(!(condition)) { \
			pr_err("function :%s\r\n", __FUNCTION__);\
			BUG();	\
		} \
	}while(0)
	
typedef enum 
{
	SLP_BIT_CLR = 0,
	SLP_BIT_SET
}SLP_BIT_DEF_E;

typedef struct  
{
	LDO_ID_E id;
	unsigned int bp_reg;
	unsigned int bp;
	unsigned int bp_rst;
	unsigned int level_reg_b0;
	unsigned int b0;
	unsigned int b0_rst;
	unsigned int level_reg_b1;
	unsigned int b1;
	unsigned int b1_rst;
	unsigned int valid_time;
	unsigned int init_level;
	int ref;
}LDO_CTL_T, * LDO_CTL_PTR;

typedef struct  
{
	SLP_LDO_E id;
	unsigned int ldo_reg;
	unsigned int mask;
	SLP_BIT_DEF_E value;
	int valid;
	unsigned int reserved;
}SLP_LDO_CTL_T, * SLP_LDO_CTL_PTR;


const LDO_CTL_T ldo_ctl_8800H[] = 
{    

	{LDO_LDO_USB,		GR_ANATST_CTL, 	BIT_12, 	BIT_13,	NULL,			NULL, 	NULL, 	
						NULL,			NULL, 	NULL,	NULL,	LDO_VOLT_LEVEL_MAX,		NULL},

       /* modified by zhengfei.xiao for SC8800H5 */
#ifdef CONFIG_CHIP_VER_8800H5
       {LDO_LDO_AVBO,		GR_LDO_CTL0, 	BIT_16, 	BIT_17,	GR_LDO_CTL3,	BIT_8, 	BIT_9, 	
						GR_LDO_CTL3,	BIT_10, 	BIT_11,	NULL,	LDO_VOLT_LEVEL_MAX,		NULL},
#endif
	{LDO_LDO_MAX,	NULL,			NULL,	NULL,	NULL,			NULL,	NULL,	
					NULL,			NULL,	NULL,	NULL,	LDO_VOLT_LEVEL_MAX,		NULL}
};      

const SLP_LDO_CTL_T slp_ldo_ctl_8800H[] =
{
	{SLP_LDO_MAX,	NULL,			NULL,	SLP_BIT_SET,	1,	NULL}
};

static  LDO_CTL_PTR Ldo_Get_Cfg(void)
{
	return ldo_ctl_8800H;
}

/**---------------------------------------------------------------------------*
 **                         Global variables                                  *
 **---------------------------------------------------------------------------*/

LDO_CTL_PTR g_ldo_ctl_tab = NULL;


/*****************************************************************************/
//  Description:  Slp_Ldo_Get_Cfg
//	Global resource dependence: NONE
//  Author: 
//	Note:    Slp_Ldo_Get_Cfg
/*****************************************************************************/
static SLP_LDO_CTL_PTR Slp_Ldo_Get_Cfg(void)
{	
	return  slp_ldo_ctl_8800H;
}

/**---------------------------------------------------------------------------*
 **                         Function Declaration                              *
 **---------------------------------------------------------------------------*/

/*****************************************************************************/
//  Description:  Turn on the LDO specified by input parameter ldo_id  
//	Global resource dependence: NONE
//  Author:  Tao.Feng && Yi.Qiu
//	Note:    return value = LDO_ERR_OK if operation is executed successfully           
/*****************************************************************************/
static  LDO_CTL_PTR LDO_GetLdoCtl(LDO_ID_E ldo_id)
{
	int i = 0;
	LDO_CTL_PTR ctl = NULL;
	
	SCI_ASSERT(NULL != g_ldo_ctl_tab);
	
	for(i=0; g_ldo_ctl_tab[i].id != LDO_LDO_MAX; i++)
    	{
    		if( g_ldo_ctl_tab[i].id == ldo_id)
    		{
    			ctl = &g_ldo_ctl_tab[i];
    			break;
    		}
    	}
		
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	return ctl;
}

/*****************************************************************************/
//  Description:  Turn on the LDO specified by input parameter ldo_id  
//	Global resource dependence: NONE
//  Author:  Tao.Feng && Yi.Qiu
//	Note:    return value = LDO_ERR_OK if operation is executed successfully           
/*****************************************************************************/
 LDO_ERR_E LDO_TurnOnLDO(LDO_ID_E ldo_id)
{
	unsigned int reg_val;	
	LDO_CTL_PTR ctl = NULL;
	unsigned long flags;
	
	ctl = LDO_GetLdoCtl(ldo_id);
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	//SCI_DisableIRQ();
	local_irq_save(flags);
	
	SCI_PASSERT(ctl->ref >= 0, ("ctl->ref = %d", ctl->ref));
	if(ctl->ref == 0) 
		REG_SETCLRBIT(ctl->bp_reg, ctl->bp_rst, ctl->bp);

	ctl->ref++;
	
	//SCI_RestoreIRQ();
	local_irq_restore(flags);

	return LDO_ERR_OK;
} 

/*****************************************************************************/
//  Description:  Turo off the LDO specified by parameter ldo_id
//	Global resource dependence: NONE
//  Author: Tao.Feng && Yi.Qiu
//	Note:           
/*****************************************************************************/
 LDO_ERR_E LDO_TurnOffLDO(LDO_ID_E ldo_id)
{
	unsigned int reg_val;
	LDO_CTL_PTR ctl = NULL;
	unsigned long flags;
	
	ctl = LDO_GetLdoCtl(ldo_id);
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	//SCI_DisableIRQ();
	local_irq_save(flags);
	
        if(ctl->ref > 0)
            ctl->ref--;
	
	if(ctl->ref == 0)
		REG_SETCLRBIT(ctl->bp_reg, ctl->bp, ctl->bp_rst);
	
	//SCI_RestoreIRQ();
	local_irq_restore(flags);
	
	return LDO_ERR_OK;
}

/*****************************************************************************/
//  Description: Find the LDO status -- ON or OFF
//	Global resource dependence: 
//  Author: Tao.Feng && Yi.Qiu
//	Note: return SCI_TRUE means LDO is ON, SCI_FALSE is OFF        
/*****************************************************************************/
 int LDO_IsLDOOn(LDO_ID_E ldo_id)
{
	unsigned int  masked_val = 0;
	LDO_CTL_PTR ctl = NULL;

	ctl = LDO_GetLdoCtl(ldo_id);
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	masked_val = __raw_readl(ctl->bp_reg) & ctl->bp;

	return (masked_val ? 0 : 1);
}

/*****************************************************************************/
//  Description:  change the LDO voltage level specified by parameter ldo_id
//	Global resource dependence: 
//  Author: Tao.Feng && Yi.Qiu   
//	Note:           
/*****************************************************************************/
 LDO_ERR_E LDO_SetVoltLevel(LDO_ID_E ldo_id, LDO_VOLT_LEVEL_E volt_level)

{
	unsigned int reg_val;
	unsigned int b0_mask,b1_mask;
	LDO_CTL_PTR  ctl = NULL;

	b0_mask = (volt_level & BIT_0)?~0:0;
	b1_mask = (volt_level & BIT_1)?~0:0;

	ctl = LDO_GetLdoCtl(ldo_id);
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	if(ctl->level_reg_b0 == NULL)
	{
		return LDO_ERR_ERR;
	}

	if(ctl->level_reg_b0 == ctl->level_reg_b1)
	{
		SET_LEVEL(ctl->level_reg_b0, b0_mask, b1_mask, ctl->b0, ctl->b0_rst, ctl->b1, ctl->b1_rst);
	}
	else
	{
		SET_LEVELBIT(ctl->level_reg_b0, b0_mask, ctl->b0, ctl->b0_rst);
		SET_LEVELBIT(ctl->level_reg_b1, b1_mask, ctl->b1, ctl->b1_rst);
	}
	
	return LDO_ERR_OK;
}

/*****************************************************************************/
//  Description: Get LDO voltage level
//	Global resource dependence: 
//  Author: Tao.Feng && Yi.Qiu   
//	Note:           
/*****************************************************************************/
 LDO_VOLT_LEVEL_E LDO_GetVoltLevel(LDO_ID_E ldo_id)
{
	unsigned int level_ret = 0;
	LDO_CTL_PTR ctl = NULL;

	ctl = LDO_GetLdoCtl(ldo_id);
	SCI_PASSERT(ctl != NULL, ("ldo_id = %d", ldo_id));

	if(ctl->level_reg_b0 == ctl->level_reg_b1)
	{
		GET_LEVEL(ctl->level_reg_b0, ctl->b0, ctl->b1, level_ret);
	}
	else
	{
		GET_LEVELBIT(ctl->level_reg_b0, ctl->b0, BIT_0, level_ret);
		GET_LEVELBIT(ctl->level_reg_b1, ctl->b1, BIT_1, level_ret);
	}

	return level_ret;
}


/*****************************************************************************/
//  Description:  Shut down any LDO that do not used when system enters deepsleep
//	Global resource dependence: s_ldo_reopen[]
//  Author: Tao.Feng && Yi.Qiu   
//	Note:           
/*****************************************************************************/
void LDO_DeepSleepInit(void)
{
	int i;
	SLP_LDO_CTL_PTR  slp_ldo_ctl_tab;
	
	slp_ldo_ctl_tab = Slp_Ldo_Get_Cfg();

	SCI_ASSERT(NULL != slp_ldo_ctl_tab);

	for(i=0; slp_ldo_ctl_tab[i].id != SLP_LDO_MAX; i++)
    	{
		if(slp_ldo_ctl_tab[i].value == SLP_BIT_SET)
			//TB_REG_OR(slp_ldo_ctl_tab[i].ldo_reg, slp_ldo_ctl_tab[i].mask);
			__raw_bits_or(slp_ldo_ctl_tab[i].mask, slp_ldo_ctl_tab[i].ldo_reg);
		else
			//TB_REG_AND(slp_ldo_ctl_tab[i].ldo_reg, ~slp_ldo_ctl_tab[i].mask);
			__raw_bits_and(~slp_ldo_ctl_tab[i].mask, slp_ldo_ctl_tab[i].ldo_reg);
    	}

}

/*****************************************************************************/
//  Description:    this function is used to initialize LDO voltage level.
//	Global resource dependence: 
//  Author: Tao.Feng && Yi.Qiu
//	Note:           
/*****************************************************************************/
static  int __init LDO_Init(void)
{
	int i;
	
	g_ldo_ctl_tab = Ldo_Get_Cfg();

	SCI_ASSERT(NULL != g_ldo_ctl_tab);

	for(i=0; g_ldo_ctl_tab[i].id != LDO_LDO_MAX; i++)
    	{
    		if( g_ldo_ctl_tab[i].init_level != LDO_VOLT_LEVEL_MAX)
    			LDO_SetVoltLevel(g_ldo_ctl_tab[i].id, g_ldo_ctl_tab[i].init_level);

		g_ldo_ctl_tab[i].ref = 0;
    	}

	//deepsleep init set for ldo
	LDO_DeepSleepInit();
	
	return LDO_ERR_OK;
}

arch_initcall(LDO_Init);