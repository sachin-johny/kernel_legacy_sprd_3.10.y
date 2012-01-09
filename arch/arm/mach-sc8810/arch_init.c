#include <linux/module.h>

#include <mach/cache-l3x0.h>

extern void arch_init_neon(void);


static int __init arch_init(void)
{
	arch_init_neon();

#ifdef CONFIG_CACHE_L2X0_310
	sp_init_l3x0();
#endif
	
	return 0;
}






early_initcall(arch_init);


