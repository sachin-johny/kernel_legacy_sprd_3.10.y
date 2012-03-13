/******************************************************************************
 ** File Name:      dc_product cfg.h                                          *
 ** Author:                                                            *
 ** DATE:                                                           *
 ** Copyright:      2012 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:                                                              *
 ** Note:                                                      				  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE              NAME            DESCRIPTION                             * 
 *****************************************************************************/
#ifndef _DC_PRODUCT_CFG_H_
#define _DC_PRODUCT_CFG_H_


#include <linux/types.h>
//#include "dcam_common.h"
#include <mach/sensor_drv.h>

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/
/**---------------------------------------------------------------------------*
 **                         Macros                                            *
 **---------------------------------------------------------------------------*/
#define DC_PRO_CFG_DEBUG 1

#ifdef DC_PRO_CFG_DEBUG
#define DC_PRO_CFG_PRINT printk 
#else
#define DC_PRO_CFG_PRINT(...)
#endif
#define DC_PRO_CFG_ERR printk 


/**---------------------------------------------------------------------------*
 **                         Data Prototype                                    *
 **---------------------------------------------------------------------------*/
typedef enum
{
    DC_MAX_VIDEO_MODE_QCIF=0x01,
    DC_MAX_VIDEO_MODE_QVGA,
    DC_MAX_VIDEO_MODE_CIF,
    DC_MAX_VIDEO_MODE_VGA,
    DC_MAX_VIDEO_MODE_MAX
}DC_MAX_VIDEO_MODE_E;

typedef enum
{
    DC_PRODUCT_FLASH_TYPE_DISABLE=0x00,
    DC_PRODUCT_FLASH_TYPE_LED,
    DC_PRODUCT_FLASH_TYPE_XENON,
    DC_PRODUCT_FLASH_TYPE_MAX
}DC_PRODUCT_FLASH_TYPE_TYPE_E;

typedef enum
{
    DC_PRODUCT_MEM_TYPE_NAND=0x00,
    DC_PRODUCT_MEM_TYPE_NOR,
    DC_PRODUCT_MEM_TYPE_MAX
}DC_PRODUCT_MEM_TYPE_E;

typedef enum
{
    DC_PRODUCT_IOCTL_CMD_MAX,
}DC_PRODUCT_IOCTL_CMD_E;


typedef struct _dc_product_mem_cfg_tag
{
    BOOLEAN exif_eb;
    BOOLEAN thumbnail_eb;
    BOOLEAN reverse1;
    BOOLEAN reverse0;
    uint32_t dc_mode_mem;
    uint32_t vt_mode_mem;
    DC_MAX_VIDEO_MODE_E max_video_mode;
    DC_PRODUCT_FLASH_TYPE_TYPE_E flash_type;
    DC_PRODUCT_MEM_TYPE_E mem_type;
}DC_PRODUCT_CFG_T, *DC_PRODUCT_CFG_T_PTR;

typedef struct _dc_product_cfg_func_tab_tag
{
    uint32_t (*get_productcfg)(uint32_t param);
    uint32_t (*get_exifprimarypridesc)(uint32_t param);
    uint32_t (*get_exifspecuser)(uint32_t param);
    uint32_t (*get_product_ioctl)(DC_PRODUCT_IOCTL_CMD_E ctl_cmd, void *param);
}DC_PRODUCT_CFG_FUNC_TAB_T, *DC_PRODUCT_CFG_FUNC_TAB_T_PTR;

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif


//typedef uint8_t		BOOLEAN;

/**---------------------------------------------------------------------------*
 **                         Public Functions                                  *
 **---------------------------------------------------------------------------*/
PUBLIC DC_PRODUCT_CFG_FUNC_TAB_T_PTR DC_GetDcProductCfgFun(void); 
/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* _DC_PRODUCT_CFG_H_ */
