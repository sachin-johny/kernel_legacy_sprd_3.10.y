#ifdef TROUT_TRACE_DBG

#ifndef TRACE_LOG_FUC_H
#define TRACE_LOG_FUC_H

void trout_trace_exit(void);
int  trout_trace_init(int buf_size,int element_size,int log_delay);
int  trout_trace_reinit(void);
void trout_trace_hex_dump(int debuglevel,int filenum,
	                            int linenum,unsigned char * ptr, int len);
void trout_trace_print_log(int debuglevel,int filenum,
	                          int linenum,char * format,...);
#endif
#endif	/* TROUT_TRACE_DBG */

