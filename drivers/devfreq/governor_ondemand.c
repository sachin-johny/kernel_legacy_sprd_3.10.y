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

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include "governor.h"

/* Default constants for DevFreq-Ondemand (DFO) */
#define DFO_UPTHRESHOLD		(90)
#define DFO_DOWNDIFFERENCTIAL	(5)

/*
* TODO: add kernel space requests
*/

struct dfs_request_state{
	int req_sum;  /* in KHz */
	u32 ddr_freq_after_req;  /* in KHz */
};
static struct dfs_request_state user_requests;
static struct devfreq *g_devfreq; /* for requests from kernel */

/************ userspace interface *****************/
struct userspace_data {
	int req_bw;
	unsigned long set_freq;
	unsigned long upthreshold;
	unsigned long downdifferential;
	unsigned long (*convert_bw_to_freq)(u32 req_bw);
	bool enable;
};

static ssize_t store_upthreshold(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned long wanted;


	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	sscanf(buf, "%lu", &wanted);
	if(data)
		data->upthreshold = wanted;
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t show_upthreshold(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if(data){
		err = sprintf(buf, "%lu\n", data->upthreshold);
	}else
		err = sprintf(buf, "%d\n", DFO_UPTHRESHOLD);
	mutex_unlock(&devfreq->lock);
	return err;
}

static ssize_t store_downdifferential(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned long wanted;


	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	sscanf(buf, "%lu", &wanted);
	if(data)
		data->downdifferential = wanted;
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t show_downdifferential(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if(data){
		err = sprintf(buf, "%lu\n", data->downdifferential);
	}else{
		err = sprintf(buf, "%d\n", DFO_DOWNDIFFERENCTIAL);
	}
	mutex_unlock(&devfreq->lock);
	return err;
}


static ssize_t store_request(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int wanted;
	unsigned long req_freq;
	int err = 0;

	req_freq = 0;
	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	sscanf(buf, "%d", &wanted);
	if(data){
		data->req_bw += wanted;
		pr_debug("*** %s, request:%d, total request:%d ***\n",
				__func__, wanted, data->req_bw);
		if(data->req_bw < 0)
			data->req_bw = 0;
		if(data->convert_bw_to_freq)
			req_freq = data->convert_bw_to_freq(data->req_bw);
	}
	user_requests.req_sum += req_freq;
	err = update_devfreq(devfreq);
	if (err == 0)
		err = count;
	mutex_unlock(&devfreq->lock);
	return err;
}

static ssize_t show_request(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if(data)
		err = sprintf(buf, "%d KB\n", data->req_bw);
	mutex_unlock(&devfreq->lock);
	return err;
}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned long wanted;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	sscanf(buf, "%lu", &wanted);
	if(data){
		data->enable = wanted;
	}
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if(data)
		err = sprintf(buf, "%d \n", data->enable);
	mutex_unlock(&devfreq->lock);
	return err;
}

static ssize_t store_freq(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned long wanted;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	sscanf(buf, "%lu", &wanted);
	if(data){
		data->set_freq = wanted;
		pr_debug("*** %s, set freq:%lu KHz***\n", __func__, wanted);
	}
	err = update_devfreq(devfreq);
	if (err == 0)
		err = count;
	mutex_unlock(&devfreq->lock);
	return err;
}

static ssize_t show_freq(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if(data)
		err = sprintf(buf, "%lu KHz\n", data->set_freq);
	mutex_unlock(&devfreq->lock);
	return err;
}

static DEVICE_ATTR(set_freq, 0644, show_freq, store_freq);
static DEVICE_ATTR(set_enable, 0644, show_enable, store_enable);
static DEVICE_ATTR(set_request, 0644, show_request, store_request);
static DEVICE_ATTR(set_upthreshold, 0644, show_upthreshold, store_upthreshold);
static DEVICE_ATTR(set_downdifferential, 0644, show_downdifferential, store_downdifferential);
static struct attribute *dev_entries[] = {
	&dev_attr_set_freq.attr,
	&dev_attr_set_enable.attr,
	&dev_attr_set_request.attr,
	&dev_attr_set_upthreshold.attr,
	&dev_attr_set_downdifferential.attr,
	NULL,
};
static struct attribute_group dev_attr_group = {
	.name	= "ondemand",
	.attrs	= dev_entries,
};

static int devfreq_ondemand_init(struct devfreq *devfreq)
{
	int err = 0;
	struct userspace_data *data = kzalloc(sizeof(struct userspace_data),
			GFP_KERNEL);

	if (!data) {
		err = -ENOMEM;
		goto out;
	}
	data->req_bw = 0;
	data->set_freq = 0;
	data->upthreshold = DFO_UPTHRESHOLD;
	data->downdifferential = DFO_DOWNDIFFERENCTIAL;
	data->enable = false;
	if(devfreq->data){
		data->convert_bw_to_freq = devfreq->data;
		pr_info("*** %s, data->convert_bw_to_freq:%pf ***\n", __func__, data->convert_bw_to_freq);
	}
	devfreq->data = data;
	g_devfreq = devfreq;
	err = sysfs_create_group(&devfreq->dev.kobj, &dev_attr_group);
out:
	return err;
}
/************ userspace interface *****************/

static int devfreq_ondemand_func(struct devfreq *df,
					unsigned long *freq)
{
	struct devfreq_dev_status stat;
	int err = df->profile->get_dev_status(df->dev.parent, &stat);
	unsigned long long a, b;
	unsigned int dfso_upthreshold = DFO_UPTHRESHOLD;
	unsigned int dfso_downdifferential = DFO_DOWNDIFFERENCTIAL;
	struct userspace_data *data = df->data;
	unsigned long max = (df->max_freq) ? df->max_freq : UINT_MAX;
	unsigned long req_freq;

	if (err)
		return err;

	if (data) {
		if (data->enable==false || data->set_freq){
			*freq = data->set_freq ? data->set_freq : max;
			return 0;
		}
		if (data->upthreshold)
			dfso_upthreshold = data->upthreshold;
		if (data->downdifferential)
			dfso_downdifferential = data->downdifferential;
	}

	if (dfso_upthreshold > 100 ||
	    dfso_upthreshold < dfso_downdifferential)
		return -EINVAL;

	/* Assume MAX if it is going to be divided by zero */
	if (stat.total_time == 0) {
		*freq = max;
		pr_debug("*** %s, stat.total_time == 0, freq:%lu ***\n", __func__, *freq);
		return 0;
	}

	/* Prevent overflow */
	if (stat.busy_time >= (1 << 24) || stat.total_time >= (1 << 24)) {
		stat.busy_time >>= 7;
		stat.total_time >>= 7;
	}

	/* Set MAX if it's busy enough */
	if (stat.busy_time * 100 >
	    stat.total_time * dfso_upthreshold) {
		*freq = max;
		pr_debug("*** %s, set max freq:%lu ***\n", __func__, *freq);
		return 0;
	}

	/* Set MAX if we do not know the initial frequency */
	if (stat.current_frequency == 0) {
		*freq = max;
		pr_debug("*** %s, stat.current_frequency == 0, freq:%lu ***\n", __func__, *freq);
		return 0;
	}

	/*
	* TODO: add request frequency
	*/
	req_freq = user_requests.req_sum;

	/* Keep the current frequency */
	if (stat.busy_time * 100 >
	    stat.total_time * (dfso_upthreshold - dfso_downdifferential)) {
		*freq = stat.current_frequency + req_freq;
		pr_debug("*** %s, Keep the current frequency %lu, req_freq:%lu ***\n",
				__func__, stat.current_frequency, req_freq);
		return 0;
	}

	/* Set the desired frequency based on the load */
	a = stat.busy_time;
	a *= stat.current_frequency;
	b = div_u64(a, stat.total_time);
	b *= 100;
	b = div_u64(b, (dfso_upthreshold - dfso_downdifferential / 2));
	*freq = (unsigned long) b + req_freq;
	pr_debug("*** %s, calculate freq:%lu, req_freq:%lu ***\n",
				__func__, (unsigned long)b, req_freq);

	if (df->min_freq && *freq < df->min_freq)
		*freq = df->min_freq;
	if (df->max_freq && *freq > df->max_freq)
		*freq = df->max_freq;

	return 0;
}

const struct devfreq_governor devfreq_ondemand = {
	.name = "ondemand",
	.init = devfreq_ondemand_init,
	.get_target_freq = devfreq_ondemand_func,
};
