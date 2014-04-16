#ifndef _TROUT_RUNMODE_H
#define _TROUT_RUNMODE_H


enum RUNMODE_MODE{
	MODE_START=0,
	SINGLE_CPU_MODE=1,
	SMP_MODE=2,
	MODE_END
};

#define TROUT_RUNMODE_DIR "driver/trout_runmode"
#define TROUT_RUNMODE_NODE "runmode"

#ifdef TROUT_RUNMODE_PROC
int  __init trout_runmode_init(void);
void  __exit trout_runmode_exit(void);
int  trout_get_runmode(void);
void trout_runmode_lock(void);
void trout_runmode_unlock(void);
#else
#define trout_runmode_init()
#define trout_runmode_exit()
#define trout_get_runmode() (SMP_MODE)
#endif


#endif
