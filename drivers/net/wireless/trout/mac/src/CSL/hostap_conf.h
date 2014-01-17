#ifndef _HOSTAP_CONF_H
#define _HOSTAP_CONF_H

#include <linux/types.h>
#define HOSTAP_CONF_FILE_NAME "/data/misc/wifi/hostapd.conf"

typedef struct _HOSTAP_CONF{
char wpa_psk[128];
unsigned int len;
}hostap_conf_t,*phostap_conf_t;

void hostap_conf_destry(phostap_conf_t);
phostap_conf_t hostap_conf_load(char * filename);
#endif
