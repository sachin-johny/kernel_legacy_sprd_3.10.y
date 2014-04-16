#ifndef _METER_TIME_H
#define _METER_TIME_H
#include "meter_define.h"

typedef struct _METER_TIME_DRIVER{
	const char name[64];
	void * (*malloc)(void);
	void  (*free)(void *);
	int   (*get)(void *);
	int (*sub)(void *,void *,void *);
	char *   (*print)(void *,char *buf);
	int (*greater)(void*,void*);
	int (*zero)(void*);
}meter_time_driver,*pmeter_time_driver;

typedef struct _METER_TIME{
	enum METER_PRECISION precision;
	pmeter_time_driver driver;
	void * private;
}meter_time,*pmeter_time;
pmeter_time meter_time_malloc(enum METER_PRECISION precision);
void meter_time_free(pmeter_time time);
int meter_time_get(pmeter_time time);
int meter_time_sub(pmeter_time a,pmeter_time b,pmeter_time * c);
char * meter_time_print(pmeter_time time,char * buf);
int meter_time_greater(pmeter_time a,pmeter_time b);
int meter_time_zero(pmeter_time a);

#endif
