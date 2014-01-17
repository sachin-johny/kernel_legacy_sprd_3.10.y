#ifndef _SIM_INCLUDE_DATA_HOOK_SPACE_H
#define _SIM_INCLUDE_DATA_HOOK_SPACE_H

extern int data_hook_in(char *src, int size, int from_kern);
extern int data_hook_out(char *dst, int size, int to_kern);

#endif