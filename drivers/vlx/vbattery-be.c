/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VBattery Backend Interface
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Vladimir Grouzdev (vladimir.grouzdev@vlx.com)
 *    Adam Mirowski (adam.mirowski@vlx.com)
 *
 ****************************************************************
 */

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#define VBATTERY_BE

#include <nk/nkern.h>
#include <vlx/vbattery_common.h>
#include "vrpc.h"

#define TRACE(format, args...)  printk("VBAT-BE: " format, ## args)
#define ETRACE(format, args...) printk("VBAT-BE: [E] " format, ## args)
#if 0
#define DTRACE(format, args...) printk("VBAT-BE: [D] " format, ## args)
#else
#define DTRACE(format, args...)
#endif

    extern int
vbattery_be_register_client (struct notifier_block* nblk);

    extern int
vbattery_be_unregister_client (struct notifier_block* nblk);

typedef struct vbat_t {
    struct power_supply* psy;	// power supply
    struct vrpc_t*	 vrpc;	// RPC link
    void*                data;	// RPC data
    vrpc_size_t          msize;	// maximum data size
    struct vbat_t*       next;	// next battery
} vbat_t;

static vbat_t*               _vbats;
static struct notifier_block _vbat_nblk;

    static vbat_t*
_vbat_lookup (struct power_supply* psy, struct vrpc_t* vrpc)
{
    vbat_t* vbat = _vbats;
    while (vbat) {
	if ((vbat->psy == psy) &&
	    (vrpc_peer_id(vbat->vrpc) == vrpc_peer_id(vrpc))) {
	    break;
	}
	vbat = vbat->next;
    }
    return vbat;
}

    static int
_psy_vprop_is_string (vbat_power_supply_property_t vproperty)
{
    return (vproperty == VBAT_POWER_SUPPLY_PROP_MODEL_NAME) ||
	   (vproperty == VBAT_POWER_SUPPLY_PROP_MANUFACTURER) ||
	   (vproperty == VBAT_POWER_SUPPLY_PROP_SERIAL_NUMBER);
}

    static vrpc_size_t
_vbat_get_string (vbat_t* vbat, const char* s, vbat_res_t* res)
{
    unsigned int len = strlen(s);

    if (len > (vbat->msize - 5)) {
	len = vbat->msize - 5;
    }
    memcpy(&res->value, s, len);
    ((char*) &res->value)[len] = 0;

    return (5 + len);
}

    static vrpc_size_t
_vbat_get_name (vbat_t* vbat, vbat_res_t* res)
{
    res->res = 0;
    return _vbat_get_string(vbat, vbat->psy->name, res);
}

    static vrpc_size_t
_vbat_get_vtype (vbat_t* vbat, vbat_res_t* res)
{
    const vbat_power_supply_type_t vtype = vbat_type2vtype (vbat->psy->type);

    if ((int) vtype < 0) {
	ETRACE("cannot map type %d to vtype\n", vbat->psy->type);
	res->res = -ESRCH;
	return 4;
    }
    res->res   = 0;
    res->value = vtype;
    return sizeof(vbat_res_t);
}

    static vrpc_size_t
_vbat_get_vprop_max (vbat_t* vbat, vbat_res_t* res)
{
    res->res   = 0;
    res->value = vbat->psy->num_properties;
    return sizeof(vbat_res_t);
}

    static vrpc_size_t
_vbat_get_vprop_vid (vbat_t* vbat, nku32_f arg, vbat_res_t* res)
{
    struct power_supply* psy = vbat->psy;
    vbat_power_supply_property_t vproperty;

    if (arg >= psy->num_properties) {
	res->res = -EINVAL;
	return 4;
    }
    vproperty = vbat_property2vproperty (psy->properties [arg]);
    if ((int) vproperty < 0) {
	ETRACE("cannot map property %d to vproperty\n", psy->properties [arg]);
	res->res = -ESRCH;
	return 4;
    }
    res->res   = 0;
    res->value = vproperty;
    return sizeof(vbat_res_t);
}

    static vrpc_size_t
_vbat_get_vprop_value (vbat_t* vbat,
		       const vbat_power_supply_property_t vproperty,
		       vbat_res_t* res)
{
    const enum power_supply_property property =
	vbat_vproperty2property (vproperty);
    union power_supply_propval val;
    struct power_supply*       psy = vbat->psy;

    if ((int) property < 0) {
	res->res = -ESRCH;
	return 4;
    }
    if ((res->res = psy->get_property (psy, property, &val))) {
	return 4;
    }
    if (_psy_vprop_is_string (vproperty)) {
	return _vbat_get_string(vbat, val.strval, res);
    }
    switch (vproperty) {
    case VBAT_POWER_SUPPLY_PROP_STATUS:
	res->value = vbat_power_supply_status2vstatus (val.intval);
	if ((int) res->value < 0) {
	    ETRACE("Could not virtualize status %d\n", val.intval);
	    res->value = VBAT_POWER_SUPPLY_STATUS_UNKNOWN;
	}
	break;

    case VBAT_POWER_SUPPLY_PROP_HEALTH:
	res->value = vbat_power_supply_health2vhealth (val.intval);
	if ((int) res->value < 0) {
	    ETRACE("Could not virtualize health %d\n", val.intval);
	    res->value = VBAT_POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	break;

    case VBAT_POWER_SUPPLY_PROP_TECHNOLOGY:
	res->value = vbat_power_supply_technology2vtechnology (val.intval);
	if ((int) res->value < 0) {
	    ETRACE("Could not virtualize technology %d\n", val.intval);
	    res->value = VBAT_POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}
	break;

    default:
	res->value = val.intval;
	break;
    }
    return sizeof(vbat_res_t);
}

    static vrpc_size_t
_vbat_call (void* cookie, vrpc_size_t size)
{
    vbat_t*     vbat = cookie;
    vbat_req_t* req  = vbat->data;
    vbat_res_t* res  = vbat->data;

    if ((vbat->msize < sizeof(vbat_res_t)) || (size != sizeof(vbat_req_t))) {
	return 0;
    }
    switch (req->cmd) {
    case VBAT_CMD_GET_NAME:
	return _vbat_get_name (vbat, res);

    case VBAT_CMD_GET_VTYPE:
	return _vbat_get_vtype (vbat, res);

    case VBAT_CMD_GET_VPROP_MAX:
	return _vbat_get_vprop_max (vbat, res);

    case VBAT_CMD_GET_VPROP_ID:
	return _vbat_get_vprop_vid (vbat, req->arg, res);

    case VBAT_CMD_GET_VPROP_VAL:
	return _vbat_get_vprop_value (vbat, req->arg, res);

    default:
	break;
    }
    return 0;
}

    static int
_vbat_create (struct power_supply* psy, struct vrpc_t* vrpc)
{
    int     res;
    vbat_t* vbat;

    vbat = kzalloc(sizeof(vbat_t), GFP_KERNEL);
    if (!vbat) {
	ETRACE("memory allocation failed\n");
	return 0;
    }

    vbat->psy   = psy;
    vbat->vrpc  = vrpc;
    vbat->data  = vrpc_data(vrpc);
    vbat->msize = vrpc_maxsize(vrpc);

    if ((vbat->msize < sizeof(vbat_req_t)) ||
        (vbat->msize < sizeof(vbat_res_t))) {
	ETRACE("not enough VRPC shared memory -> %d\n", vbat->msize);
	kfree(vbat);
	return 0;
    }

    if ((res = vrpc_server_open(vrpc, _vbat_call, vbat, 0))) {
	ETRACE("VRPC open failed -> %d\n", res);
	kfree(vbat);
	return 0;
    }

    DTRACE("VBAT %s -> %d created\n", psy->name, vrpc_peer_id(vrpc));

    vbat->next = _vbats;
    _vbats     = vbat;

    return 1;
}

    static void
_vbat_destroy (vbat_t* vbat)
{
    vbat_t** link = &_vbats;

    DTRACE("VBAT %s -> %d destroyed\n",
	   vbat->psy->name, vrpc_peer_id(vbat->vrpc));

    vrpc_close(vbat->vrpc);
    vrpc_release(vbat->vrpc);

    while (*link != vbat) link = &(*link)->next;
    *link = vbat->next;

    kfree(vbat);
}

    static void
_vbat_setup (struct power_supply* psy)
{
    struct vrpc_t* vrpc = 0;
    while ((vrpc = vrpc_server_lookup(VBAT_VRPC_NAME, vrpc))) {
	if (_vbat_lookup(psy, vrpc)) {
	    vrpc_release(vrpc);
	} else {
	    if (!_vbat_create(psy, vrpc)) {
		vrpc_release(vrpc);
	    }
	}
    }
}

    static int
_vbat_notify (struct notifier_block* nblk,
	      unsigned long          event,
	      void*                  data)
{
    struct power_supply* psy = data;
    if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
	_vbat_setup(psy);
    }
    return NOTIFY_DONE;
}

    static int __init
_vbat_init (void)
{
    int res;
    _vbat_nblk.notifier_call = _vbat_notify;
    if ((res = vbattery_be_register_client(&_vbat_nblk))) {
	ETRACE("client registration failed (%d)", res);
	return res;
    }
    TRACE("module loaded\n");
    return 0;
}

    static void __exit
_vbat_exit (void)
{
    vbattery_be_unregister_client(&_vbat_nblk);
    while (_vbats) {
	_vbat_destroy(_vbats);
    }
    TRACE("module unloaded\n");
}

MODULE_DESCRIPTION("Virtual battery backend driver on top of VLX");
MODULE_AUTHOR("Vladimir Grouzdev <vladimir.grouzdev@vlx.com> - VirtualLogix");
MODULE_LICENSE("Proprietary");

module_init(_vbat_init);
module_exit(_vbat_exit);
