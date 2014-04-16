#ifdef TROUT_METER
#ifndef _METER_H
#define _METER_H
#include "meter_define.h"
#include "meter_time.h"
#include <linux/mutex.h>


typedef struct _STEP{
	char name[256];
	struct list_head node;
	pmeter_time time;
}meter_step_t,*pmeter_step;

typedef struct _METER{
	char name[128];
	struct mutex lock;
	struct mutex  list_lock;
	struct list_head step_list;
	pmeter_time start_time;
	pmeter_time end_time;
	pmeter_step pre_step;
	enum METER_PRECISION precision;
	enum METER_TYPE type;
	struct list_head node;
}meter_t,*pmeter_t;

void  meter_init(void);
void  meter_exit(void);
int meter_start(const char * name, enum METER_PRECISION percision,enum METER_TYPE type);
int meter_step(const char * mname,const char * fmt,...);
int meter_end(const char * name);
#endif

#else

#define meter_init()
#define  meter_exit()
#define meter_start(x, y,z)
#define meter_step(z,y,...)
#define meter_end(x)

#endif
