#ifndef _TROUT_RW_INTERFACE_H
#define _TROUT_RW_INTERFACE_H


enum EMODULE{
	REG,
	WIFI_RAM,
	BT_RAM,
	POWER,
	LEGACY,
	MODULE_MAX
};

enum DEBUG_LEVEL{
	DEBUG_INFO,
	DEBUG_ERROR,
	DEBUG_LEVEL_MAX
};

enum CONFIG_MODE{
	BIN_MODE=1,
	ASIC_MODE
};

#define MODULE_MASK 0x01
#define ADDR_MASK 0x02
#define DEBUG_MASK 0x04
#define ALL_MASK (MODULE_MASK|ADDR_MASK|DEBUG_MASK)

struct trout_rw_config{
	enum EMODULE module;
	 unsigned int addr;	
	enum DEBUG_LEVEL debug_level;
	unsigned int flag;
};

#define TROUT_DIR "driver/trout"

#define TROUT_CONFIG_NODE "rw_config"
#define TROUT_DATA_NODE "rw_buffer"
#endif
