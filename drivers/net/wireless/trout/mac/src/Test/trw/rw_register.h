#ifndef _TROUT_RW_REGISTER
#define _TROUT_RW_REGISTER

#ifdef TROUT_DEBUG_RW_PROC
int  __init trout_rw_init(void);
void  __exit trout_rw_exit(void);
#else
#define trout_rw_init()
#define trout_rw_exit()
#endif

#endif
