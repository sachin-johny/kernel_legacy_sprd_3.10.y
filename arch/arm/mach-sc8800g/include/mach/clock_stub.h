/*
     clock infomation shared by linux & rtos.

	 wangliwei. 2011-03-10
 */

#ifndef	__ARCH_ARM_SC8800G2_CLOCK_STUB_H
#define	__ARCH_ARM_SC8800G2_CLOCK_STUB_H



/* clock flags. */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define CONFIG_PARTICIPANT	(1 << 10)	/* Fundamental clock */
#define ENABLE_ON_INIT	(0x1UL << 11)	/* enable on framework init. */
#define INVERT_ENABLE           (1 << 12)       /* 0 enables, 1 disables */

/* clksel flags. */
#define RATE_IN_SC8800G2	(0x1UL << 0)


/* which level the clock belongs to, AHB or APB. */
#define	DEVICE_AHB	(0x1UL << 20)
#define	DEVICE_APB	(0x1UL << 21)
#define	DEVICE_VIR	(0x1UL << 22)
#define    DEVICE_AWAKE (0x1UL << 23)
#define DEVICE_TEYP_MASK (DEVICE_AHB | DEVICE_APB | DEVICE_VIR | DEVICE_AWAKE)

#define	CLOCK_NUM		128
#define	MAX_CLOCK_NAME_LEN	16

/* resource id. */
#define	RES_CLOCK_STUB_MEM	0
#define	RES_CLOCK_NAME_MEM	1


 struct clock_stub {
	unsigned char *name;
	unsigned int  flags;
	int usecount;
};

#endif
