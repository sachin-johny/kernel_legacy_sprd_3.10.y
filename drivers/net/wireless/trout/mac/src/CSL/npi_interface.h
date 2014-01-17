#ifndef _NPI_INTERFACE_H
#define _NPI_INTERFACE_H
#include <linux/wireless.h>

#define NPI_CTL_CODE  SIOCDEVPRIVATE + 5

enum NPI_CMD_CODE{
	SET_MAC,
	TX_START,
	TX_STOP,
	RX_START,
	RX_STOP,
	SET_RATE,
	SET_CHANNEL,
	GET_CHANNEL,
	SET_TX_POWER,
	GET_TX_POWER,
	GET_RX_COUNT,
	SET_RX_COUNT,
	GET_RX_ERROR,
	GET_RSSI,
	SET_TX_MODE,
	CONNECT_AP,
	DISCONNECT_AP,
	GET_RX_STATUS,
	MAX_CMD_CODE
};

typedef struct _NPI_CMD{
	enum NPI_CMD_CODE cmd;
	union{
		char * buffer;
		int int_val;
		double d_val;
	}param;
	int param_buffer_len;
	union{
		char * buffer;
		int * int_val;
		double * d_val;
	}result;
	int  * result_buffer_len;
	int * result_code;
}NPI_CMD,*PNPI_CMD;


#define ENTRY(x,y) {x,y}

typedef int (*dispatch_fun_t)(PNPI_CMD);

struct cmd_entry{
	enum NPI_CMD_CODE cmd;
	dispatch_fun_t op;
};

#endif
