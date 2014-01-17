#ifndef MAC_STRING_H
#define MAC_STRING_H

#include <linux/types.h>
int hexstr2bin(const char *hex, u8 *buf, size_t len);
int hex2byte(const char *hex);
#endif
