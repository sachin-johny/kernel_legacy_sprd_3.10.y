/*****************************************************************************
 *                                                                           *
 *  Component: VLX Virtual OpenGL ES (VOGL).                                 *
 *             VOGL backend kernel driver implementation.                    *
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

MODULE_DESCRIPTION("VOGL Back-End Driver");
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
static VrpqDrv vrpq_srv_drv = {
    .name         = "vrpq-srv-vogl",
    .major        = VOGL_SRV_MAJOR,
    .resc_id_base = 0,
    .prop_get     = vrpq_prop_get,
};

    /*
     *
     */
    static int
vogl_srv_vlink_init (Vlink* vlink)
{
    VLINK_DTRACE(vlink, "vogl_srv_vlink_init called\n");
    return vrpq_srv_vlink_init(&vrpq_srv_drv, vlink);
}

    /*
     *
     */
    static int
vogl_srv_drv_init (VlinkDrv* vogl_srv_drv)
{
    printk(KERN_INFO "vogl_srv_drv_init called\n");
    return vrpq_srv_drv_init(vogl_srv_drv, &vrpq_srv_drv);
}

    /*
     *
     */
    static void
vogl_srv_drv_cleanup (VlinkDrv* vogl_srv_drv)
{
    printk(KERN_INFO "vogl_srv_drv_cleanup called\n");
    vrpq_srv_drv_cleanup(&vrpq_srv_drv);
}

    /*
     *
     */
static VlinkDrv vogl_srv_drv = {
    .name       = "vogl",
    .init       = vogl_srv_drv_init,
    .cleanup    = vogl_srv_drv_cleanup,
    .vlink_init = vogl_srv_vlink_init,
    .flags      = VLINK_DRV_TYPE_SERVER,
};

    /*
     *
     */
    static int
vogl_be_module_init (void)
{
    int diag;

    if ((diag = vlink_drv_probe(&vogl_srv_drv)) != 0) {
	return diag;
    }
    vlink_drv_startup(&vogl_srv_drv);

    return 0;
}

    /*
     *
     */
    static void
vogl_be_module_exit (void)
{
    vlink_drv_shutdown(&vogl_srv_drv);
    vlink_drv_cleanup(&vogl_srv_drv);
}

module_init(vogl_be_module_init);
module_exit(vogl_be_module_exit);
