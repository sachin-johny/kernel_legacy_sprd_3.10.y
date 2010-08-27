#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>

static struct input_dev *input;

static int sprdummy_open(struct input_dev *dev)
{
	return 0;
}

static void sprdummy_close(struct input_dev *dev)
{ }

static int __init sprdummy_init(void)
{
	input = input_allocate_device();

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input, ABS_X, 0, 1000, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 1000, 1, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 0xff, 0, 0);

	input->name = "sprdummy";
	input->id.bustype = BUS_HOST;
	input->open = sprdummy_open;
	input->close = sprdummy_close;

	return input_register_device(input);
}

static void __exit sprdummy_exit(void)
{
    input_unregister_device(input);
}

MODULE_DESCRIPTION("sprdummy Touchscreen driver");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");

module_init(sprdummy_init);
module_exit(sprdummy_exit);
