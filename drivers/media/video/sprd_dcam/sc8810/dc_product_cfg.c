/******************************************************************************
 ** File Name:      dc_cfg.c                                           *
 ** Author:         Tim.zhu                                             *
 ** DATE:           11/16/2009                                                *
 ** Copyright:      2009 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    This file defines the product configure dc parameter    *
 **                                                                           *
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 11/16/2009     Tim.zhu	  Create.                         		  *
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/ 
//#include "os_api.h"
#include <mach/dc_product_cfg.h>
//#include "dc_cfg.h"
//#include "dal_time.h"
//#include "lcd_cfg.h"

#include <mach/jpeg_exif_header_k.h>

typedef int8_t 	int8;


#define SCI_TRUE 1 
#define SCI_FALSE 0

/*lint -save -e551 */	
/**---------------------------------------------------------------------------*
 **                         Debugging Flag                                    *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
extern   "C" 
{
#endif
/*lint -save -e785 */
/*----------------------------------------------------------------------------*
**                         Local Function Prototype                           *
**---------------------------------------------------------------------------*/
LOCAL uint32 _DC_GetProductCfgInfo(uint32 param);
LOCAL uint32 _DC_GetExifPrimaryPriDescInfo(uint32 param);
LOCAL uint32 _DC_GetExifSpecUserInfo(uint32 param);

/**---------------------------------------------------------------------------*
 **                         Macro Definition                                  *
 **---------------------------------------------------------------------------*/
#define DC_MEM_SIZE (3*1024*1024)
#define DV_MEM_SIZE (3*1024*1024)
#define VT_MEM_SIZE (3*1024*1024) 

/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL EXIF_PRI_DESC_T s_dc_exif_pri_desc_info=
{
    {
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01
    },
    "Default Date",             //Date
    "ImageDescription",       //ImageDescription
    "Maker",                     //Make
    "Model",                    //Model
    "Software Version v0.0.0",  //Software
    "Artist",                   //Artist
    "CopyRight"                 //Copyright
};

LOCAL uint8 exif_user_comments[20] = {"User Comments"};

LOCAL EXIF_SPEC_USER_T s_dc_exif_spec_user_info = {0};

LOCAL DC_PRODUCT_CFG_T s_dc_product_cfg_info=
{
    SCI_TRUE,
    SCI_TRUE,
    SCI_FALSE,
    SCI_FALSE,
    DC_MEM_SIZE,
    DV_MEM_SIZE,
    VT_MEM_SIZE,
    DC_MAX_VIDEO_MODE_CIF,
    DC_PRODUCT_FLASH_TYPE_DISABLE
};

// custom cfg function
LOCAL DC_PRODUCT_CFG_FUNC_TAB_T s_dc_product_cfg_fun=
{
    _DC_GetProductCfgInfo,
    _DC_GetExifPrimaryPriDescInfo,
    _DC_GetExifSpecUserInfo
};
/**---------------------------------------------------------------------------*
 **                     Public Function Prototypes                            *
 **---------------------------------------------------------------------------*/

/*****************************************************************************/
//  Description:  This function is used to get dc custom cfg
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
LOCAL uint32 _DC_GetProductCfgInfo(uint32 param)
{    
    return (uint32)&s_dc_product_cfg_info;
}

/*****************************************************************************/
//  Description:  This function is used to get exif primary pri desc
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
LOCAL uint32 _DC_GetExifPrimaryPriDescInfo(uint32 param)
{
    EXIF_PRI_DESC_T* exif_ptr=&s_dc_exif_pri_desc_info;
#ifdef KERNEL_TIME
    SCI_DATE_T       cur_date = {0};
    SCI_TIME_T       cur_time = {0};

    *(uint32*)&s_dc_exif_pri_desc_info.valid = (uint32)0x7F;
    TM_GetSysDate(&cur_date);
    TM_GetSysTime(&cur_time);
    
	sprintf((int8 *)exif_ptr->DateTime, 
            "%04d:%02d:%02d %02d:%02d:%02d", 
            cur_date.year, 
            cur_date.mon, 
            cur_date.mday,
            cur_time.hour, 
            cur_time.min,
            cur_time.sec);
#endif
//    sprintf((int8*)s_dc_exif_pri_desc_info.Copyright, "%s", "CopyRight");
//    sprintf((int8*)s_dc_exif_pri_desc_info.ImageDescription, "%s", "ImageDescription");
    sprintf((int8*)s_dc_exif_pri_desc_info.Make, "%s", "Spreadtrum");    
    sprintf((int8*)s_dc_exif_pri_desc_info.Model, "%s", "SP8810ga");    
 //   sprintf((int8*)s_dc_exif_pri_desc_info.Software, "%s", "Test Version v0.0.0.1");        
 //   sprintf((int8*)s_dc_exif_pri_desc_info.Artist, "%s", "Artist");        
    sprintf((int8*)s_dc_exif_pri_desc_info.Copyright, "%s", "CopyRight, Spreadtrum, 2012");        
    return (uint32)exif_ptr;
}

/*****************************************************************************/
//  Description:  This function is used to get exif primary pri desc
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
LOCAL uint32 _DC_GetExifSpecUserInfo(uint32 param)
{
    EXIF_SPEC_USER_T* exif_ptr=&s_dc_exif_spec_user_info;
/*   
    s_dc_exif_spec_user_info.valid.MakerNote = 0;
    s_dc_exif_spec_user_info.valid.UserComment = 0;
    s_dc_exif_spec_user_info.UserComment.count = 1;
    s_dc_exif_spec_user_info.UserComment.count = strlen((char*)exif_user_comments);
    s_dc_exif_spec_user_info.UserComment.ptr = (void*)exif_user_comments;
    s_dc_exif_spec_user_info.UserComment.type = EXIF_ASCII;
    s_dc_exif_spec_user_info.UserComment.size = 1;
*/
    DC_PRO_CFG_PRINT("DC_PRODUCT: _DC_GetExifSpecUserInfo, valid.UserComment %d \n",s_dc_exif_spec_user_info.valid.UserComment);	
    return (uint32)exif_ptr;
}

/*****************************************************************************/
//  Description:  This function is used to get dc custom cfg function    
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC DC_PRODUCT_CFG_FUNC_TAB_T_PTR DC_GetDcProductCfgFun(void)
{
    return &s_dc_product_cfg_fun;
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
/*lint -restore */ 
#ifdef   __cplusplus
}
#endif  // end of dc_cfg.c    

