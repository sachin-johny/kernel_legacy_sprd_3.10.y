/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/opp.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>

#ifdef CONFIG_BUS_MONITOR
#include <mach/bm_sc8830.h>
#endif

extern u32 emc_clk_set(u32 new_clk, u32 sene);
extern u32 emc_clk_get(void);

enum scxx30_dmc_type {
	TYPE_DMC_SCXX30 ,
};

enum dmcclk_level_idx {
	LV_0 = 0,
	LV_1,
	LV_2,
	LV_3,
	LV_4,
	_LV_END
};

struct dmc_opp_table {
	unsigned int idx;
	unsigned long clk;  /* KHz */
	unsigned long volt; /* uv */
	unsigned long bandwidth; /* MB/s, max: clk*2*32/8 */
};

static struct dmc_opp_table scxx30_dmcclk_table[] = {
	{LV_0, 532000, 1200000, 4256},
	{LV_1, 400000, 1200000, 3200},
	{LV_2, 332000, 1200000, 2656},
	{LV_3, 200000, 1200000, 1600},
	{0, 0, 0},
};

struct dmcfreq_data {
	enum scxx30_dmc_type type;
	struct device *dev;
	struct devfreq *devfreq;
	bool disabled;
	struct opp *curr_opp;

	struct notifier_block pm_notifier;
	unsigned long last_jiffies;
	struct mutex lock;
};

#define SCXX30_LV_NUM (LV_4)
#define SCXX30_MAX_FREQ (532000)
#define SCXX30_MIN_FREQ (200000)
#define SCXX30_INITIAL_FREQ SCXX30_MAX_FREQ
#define SCXX30_POLLING_MS (500)
#define BOOT_TIME	(40*HZ)
static u32 boot_done;

static unsigned long scxx30_max_freq(struct dmcfreq_data *data)
{
	switch (data->type) {
	case TYPE_DMC_SCXX30:
		return SCXX30_MAX_FREQ;
	default:
		pr_err("Cannot determine the device id %d\n", data->type);
		return (-EINVAL);
	}
}

static unsigned long scxx30_min_freq(struct dmcfreq_data *data)
{
	switch (data->type) {
	case TYPE_DMC_SCXX30:
		return SCXX30_MIN_FREQ;
	default:
		pr_err("Cannot determine the device id %d\n", data->type);
		return (-EINVAL);
	}
}

/*
* convert bandwidth request to DDR freq
* @bw, request bandwidth, KB
* @return, KHz
*/
static int scxx30_convert_bw_to_freq(int bw)
{
	int freq;

	freq = 0;
	/*freq = dmc_convert_bw_to_freq(bw);*/
	/*
	* freq(KHz)*2(DDR)*32(BUS width)/8 = bw(KB)*8(efficiency ratio)
	*/
	freq = bw;

	return freq;
}

static int scxx30_dmc_target(struct device *dev, unsigned long *_freq,
				u32 flags)
{
	int err = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device,
							dev);
	struct dmcfreq_data *data = platform_get_drvdata(pdev);
	struct opp *opp = devfreq_recommended_opp(dev, _freq, flags);
	unsigned long freq = opp_get_freq(opp);
	unsigned long old_freq = emc_clk_get()*1000 ;

	if(time_before(jiffies, boot_done)){
		return 0;
	}

	if (IS_ERR(opp))
		return PTR_ERR(opp);

	pr_debug("*** %s, old_freq:%luKHz, freq:%luKHz ***\n", __func__, old_freq, freq);

	if (old_freq == freq)
		return 0;

	dev_dbg(dev, "targetting %lukHz %luuV\n", freq, opp_get_voltage(opp));

	mutex_lock(&data->lock);

	if (data->disabled)
		goto out;
	freq = freq/1000; /* conver KHz to MHz */
	err = emc_clk_set(freq, 1);
	data->curr_opp = opp;
	pr_debug("*** %s, old_freq:%luKHz, set emc done, err:%d, current freq:%uKHz ***\n",
			__func__, old_freq, err, emc_clk_get()*1000 );

out:
	mutex_unlock(&data->lock);
	return err;
}

static int scxx30_dmc_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
#ifdef CONFIG_BUS_MONITOR
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	u32 total_bw;
	u64 trans_bw;
	u32 interval;
	dmc_mon_cnt_stop();
	trans_bw = (u64)dmc_mon_cnt_bw(); /* total access: B */
	dmc_mon_cnt_clr();
	dmc_mon_cnt_start();
	interval = jiffies - data->last_jiffies;
	data->last_jiffies = jiffies;

	stat->current_frequency = emc_clk_get() * 1000; /* KHz */
	/* stat->current_frequency = opp_get_freq(data->curr_opp); */
	total_bw = (stat->current_frequency)*8; /* freq*2*32/8 */
	pr_debug("*** %s, trans_bw:%lluB, curr freq:%lu, total_bw:%uKB ***\n",
			__func__, trans_bw, stat->current_frequency, total_bw);

	/*
	* TODO: efficiency ratio could be more accurate??
	*/
	if(interval){
		stat->busy_time = (u32)div_u64(trans_bw*HZ, interval); /* BW: B/s */
		stat->total_time = total_bw*125 ;   /* BW: KB*1000/8(efficiency ratio) B/s */
	}else{
		stat->busy_time = 0 ;
		stat->total_time = 0;
	}
	pr_debug("*** %s, interval:%u, busy_time:%lu, totoal_time:%lu ***\n",
				__func__, interval, stat->busy_time, stat->total_time );
#else
	stat->busy_time = 0 ;
	stat->total_time = 0 ;
#endif
	return 0;
}


static void scxx30_dmc_exit(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);

	devfreq_unregister_opp_notifier(dev, data->devfreq);

	return;
}

static struct devfreq_dev_profile scxx30_dmcfreq_profile = {
	.initial_freq	= SCXX30_INITIAL_FREQ,
	.polling_ms	= SCXX30_POLLING_MS,
	.target		= scxx30_dmc_target,
	.get_dev_status	= scxx30_dmc_get_dev_status,
	.exit		= scxx30_dmc_exit,
};

static int scxx30_init_tables(struct dmcfreq_data *data)
{
	int i, err;

	switch (data->type) {
	case TYPE_DMC_SCXX30:
		for (i = LV_0; i < SCXX30_LV_NUM; i++) {
			err = opp_add(data->dev, scxx30_dmcclk_table[i].clk,
					scxx30_dmcclk_table[i].volt);
			if (err) {
				dev_err(data->dev, "Cannot add opp entries.\n");
				return err;
			}
		}
		break;
	default:
		dev_err(data->dev, "Cannot determine the device id %d\n", data->type);
		err = -EINVAL;
	}

	return err;
}

static int scxx30_dmcfreq_pm_notifier(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct dmcfreq_data *data = container_of(this, struct dmcfreq_data,
						 pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&data->lock);
		data->disabled = true;
		/*
		* DMC must be set 200MHz before deep sleep in ES chips
		*/
		emc_clk_set(200, 1);
		mutex_unlock(&data->lock);
		return NOTIFY_OK;
#if 0
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		/* Reactivate */
		mutex_lock(&data->lock);
		data->disabled = false;
		mutex_unlock(&data->lock);
		return NOTIFY_OK;
#endif
	}

	return NOTIFY_DONE;
}

static __devinit int scxx30_dmcfreq_probe(struct platform_device *pdev)
{
	struct dmcfreq_data *data;
	struct opp *opp;
	struct device *dev = &pdev->dev;
	int err = 0;

	data = kzalloc(sizeof(struct dmcfreq_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	data->type = pdev->id_entry->driver_data;
	data->pm_notifier.notifier_call = scxx30_dmcfreq_pm_notifier;
	data->dev = dev;
	mutex_init(&data->lock);

	switch (data->type) {
	case TYPE_DMC_SCXX30:
		err = scxx30_init_tables(data);
		break;
	default:
		dev_err(dev, "Cannot determine the device id %d\n", data->type);
		err = -EINVAL;
	}
	if (err)
		goto err_opp_add;

	opp = opp_find_freq_floor(dev, &scxx30_dmcfreq_profile.initial_freq);
	if (IS_ERR(opp)) {
		dev_err(dev, "Invalid initial frequency %lu kHz.\n",
		       scxx30_dmcfreq_profile.initial_freq);
		err = PTR_ERR(opp);
		goto err_opp_add;
	}
	data->curr_opp = opp;
	data->last_jiffies = jiffies;
	platform_set_drvdata(pdev, data);
	data->devfreq = devfreq_add_device(dev, &scxx30_dmcfreq_profile,
					   &devfreq_ondemand, scxx30_convert_bw_to_freq);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		dev_err(dev, "Failed to add device\n");
		goto err_opp_add;
	}

	devfreq_register_opp_notifier(dev, data->devfreq);

	data->devfreq->min_freq = scxx30_min_freq(data);
	data->devfreq->max_freq = scxx30_max_freq(data);

	err = register_pm_notifier(&data->pm_notifier);
	if (err) {
		dev_err(dev, "Failed to setup pm notifier\n");
		goto err_devfreq_add;
	}
#ifdef CONFIG_BUS_MONITOR
	dmc_mon_cnt_clr( );
	dmc_mon_cnt_start( );
#endif
	boot_done = jiffies + BOOT_TIME;
	pr_info(" %s done,  current freq:%lu \n", __func__, opp_get_freq(data->curr_opp));
	return 0;

err_devfreq_add:
	devfreq_remove_device(data->devfreq);
err_opp_add:
	kfree(data);
	return err;
}

static __devexit int scxx30_dmcfreq_remove(struct platform_device *pdev)
{
	struct dmcfreq_data *data = platform_get_drvdata(pdev);

	unregister_pm_notifier(&data->pm_notifier);
	devfreq_remove_device(data->devfreq);
	kfree(data);

	return 0;
}

static int scxx30_dmcfreq_resume(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	data->disabled = false;
#ifdef CONFIG_BUS_MONITOR
	dmc_mon_cnt_clr( );
	dmc_mon_cnt_start( );
#endif
	return 0;
}

static const struct dev_pm_ops scxx30_dmcfreq_pm = {
	.resume	= scxx30_dmcfreq_resume,
};

static const struct platform_device_id scxx30_dmcfreq_id[] = {
	{ "scxx30-dmcfreq", TYPE_DMC_SCXX30 },
	{ },
};

static struct platform_device scxx30_dmcfreq = {
	.name = "scxx30-dmcfreq",
};

static struct platform_driver scxx30_dmcfreq_driver = {
	.probe	= scxx30_dmcfreq_probe,
	.remove	= __devexit_p(scxx30_dmcfreq_remove),
	.id_table = scxx30_dmcfreq_id,
	.driver = {
		.name	= "scxx30_dmcfreq",
		.owner	= THIS_MODULE,
		.pm	= &scxx30_dmcfreq_pm,
	},
};

static int __init scxx30_dmcfreq_init(void)
{
	int err;
	err = platform_device_register(&scxx30_dmcfreq);
	if(err){
		pr_err(" register scxx30_dmcfreq failed, err:%d\n", err);
	}
	err = platform_driver_register(&scxx30_dmcfreq_driver);
	if(err){
		pr_err(" register scxx30_dmcfreq_driver failed, err:%d\n", err);
	}
	return err;
}
late_initcall(scxx30_dmcfreq_init);

static void __exit scxx30_dmcfreq_exit(void)
{
	platform_driver_unregister(&scxx30_dmcfreq_driver);
}
module_exit(scxx30_dmcfreq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SCxx30 dmcfreq driver with devfreq framework");
