#ifndef _BT_H_
#define _BT_H_

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define INLINE static __inline

/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

typedef int            WORD32;
typedef short          WORD16;
typedef char           WORD8;
typedef unsigned int   UWORD32;
typedef unsigned short UWORD16;
typedef unsigned char  UWORD8;
typedef signed char    SWORD8;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {BFALSE = 0,
              BTRUE  = 1
} BOOL_T;



/* bluetooth ram addr, bytes  */
#define rBT_RAM_CODE_SPACE     (0x0)
#define rBT_RAM_PARAMS_SPACE   (0x4050U)
// BT ram enable/disable regs
#define COM_REG_ARM7_CTRL           (0x4015 << 2)
#define COM_REG_BT_IRAM_EN          (0x4013 << 2)
#define SYS_REG_CLK_CTRL1           (0x0021 << 2)


#define DUMP_BT_CODE_LEN  (96*1024)

/* debug flags, combine by adding/bitwise OR */
enum debuglevel {
   DEBUG_PRINT	  = 0x00, /* only print all bt regs  */
	DEBUG_DUMPBTREG	  = 0x01, /* dump all bt regs  */
	DEBUG_OPENHCIDUMPALL	  = 0x02, /* open hcidump all */
	DEBUG_ANY	  = 0xff, /* print message if any of the others is activated */
};



/* This function chooses between standard library or Ittiam implementation */
/* of the memset function depending upon the platform.                     */
INLINE void *mem_set(void *buff, WORD32 val,  UWORD32 num)
{
#ifdef MWLAN
    return itm_memset(buff, val, num);
#else  /* MWLAN */
    return memset(buff, val, num);
#endif /* MWLAN */
}

#endif
