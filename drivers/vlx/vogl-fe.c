/*****************************************************************************
 *                                                                           *
 *  Component: VLX Virtual OpenGL ES (VOGL).                                 *
 *             VOGL frontend kernel driver implementation.                   *
 *                                                                           *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.              *
 *                                                                           *
 *  #ident  "%Z%%M% %I%     %E% Red Bend Software"                           *
 *                                                                           *
 *  Contributor(s):                                                          *
 *    Sebastien Laborie <sebastien.laborie@redbend.com>                      *
 *                                                                           *
 *****************************************************************************/

#define DEBUG

#include <linux/module.h>
#include "vogl.h"

MODULE_DESCRIPTION("VOGL Front-End Driver");
MODULE_AUTHOR("Sebastien Laborie <sebastien.laborie@redbend.com>");
MODULE_LICENSE("GPL");

    /*
     *
     */
    static int
vrpq_prop_get (Vlink* vlink, unsigned int type, void* prop)
{
    switch (type) {
    case VRPQ_PROP_PMEM_SIZE:
    {
	NkPhSize* sz = (NkPhSize*) prop;
	*sz = 10240 * 1024;
	break;
    }
    case VRPQ_PROP_RING_REQS_MAX:
    {
	VrpqRingIdx* reqs_max = (VrpqRingIdx*) prop;
	*reqs_max = 1024;
	break;
    }
    default:
	return -EINVAL;
    }
    return 0;
}

    /*
     *
     */
static VrpqDrv vrpq_clt_drv = {
    .name         = "vrpq-clt-vogl",
    .major        = VOGL_CLT_MAJOR,
    .resc_id_base = 0,
    .prop_get     = vrpq_prop_get,
};

    /*
     *
     */
    static int
vogl_clt_vlink_init (Vlink* vlink)
{
    VLINK_DTRACE(vlink, "vogl_clt_vlink_init called\n");
    return vrpq_clt_vlink_init(&vrpq_clt_drv, vlink);
}

    /*
     *
     */
    static int
vogl_clt_drv_init (VlinkDrv* vogl_clt_drv)
{
    printk(KERN_INFO "vogl_clt_drv_init called\n");
    return vrpq_clt_drv_init(vogl_clt_drv, &vrpq_clt_drv);
}

    /*
     *
     */
    static void
vogl_clt_drv_cleanup (VlinkDrv* vogl_clt_drv)
{
    printk(KERN_INFO "vogl_clt_drv_cleanup called\n");
    vrpq_clt_drv_cleanup(&vrpq_clt_drv);
}

    /*
     *
     */
static VlinkDrv vogl_clt_drv = {
    .name       = "vogl",
    .init       = vogl_clt_drv_init,
    .cleanup    = vogl_clt_drv_cleanup,
    .vlink_init = vogl_clt_vlink_init,
    .flags      = VLINK_DRV_TYPE_CLIENT,
};

    /*
     *
     */
    static int
vogl_fe_module_init (void)
{
    int diag;

    if ((diag = vlink_drv_probe(&vogl_clt_drv)) != 0) {
	return diag;
    }
    vlink_drv_startup(&vogl_clt_drv);

    return 0;
}

    /*
     *
     */
    static void
vogl_fe_module_exit (void)
{
    vlink_drv_shutdown(&vogl_clt_drv);
    vlink_drv_cleanup(&vogl_clt_drv);
}

module_init(vogl_fe_module_init);
module_exit(vogl_fe_module_exit);
