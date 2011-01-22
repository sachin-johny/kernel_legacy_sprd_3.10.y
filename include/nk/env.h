/*
 ****************************************************************
 *
 * Component = NKERNEL
 * 
 * #ident  "@(#)env.h 1.6     07/02/01 VirtualLogix"
 * 
 * Contributor(s):
 * 
 ****************************************************************
 */

#ifndef _NK_BOOT_ENV_H
#define _NK_BOOT_ENV_H

#define ENV_MAGIC 0xa3889a8a

typedef struct EnvDesc {
    int	    envMagic;
    int	    envChecksum;
    int	    envDataOffset;
    int	    envSize;
    int	    envMaxSize;
    char*   envPtr;
} EnvDesc;

#endif
