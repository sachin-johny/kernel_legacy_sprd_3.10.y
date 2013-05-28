#include <linux/device.h>
#include <linux/err.h>

struct class *sprd_class;
EXPORT_SYMBOL(sprd_class);

static int __init rhea_class_create(void)
{
	sprd_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sprd_class)) {
		pr_err("Failed to create class(sec)!\n");
		return PTR_ERR(sprd_class);
	}

	return 0;
}

subsys_initcall(rhea_class_create);

