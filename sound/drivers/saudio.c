/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <linux/sipc.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/memory.h>
#include <linux/io.h>
#include <sound/saudio.h>

#define ETRACE(x...)			printk(KERN_ERR "Error: " x)

#define ADEBUG()			pr_debug("saudio.c:line %d\n",__LINE__)

#define CMD_BLOCK_SIZE			80
#define TX_DATA_BLOCK_SIZE		80
#define RX_DATA_BLOCK_SIZE		0
#define MAX_BUFFER_SIZE			(64*1024)

#define MAX_PERIOD_SIZE			MAX_BUFFER_SIZE

#define USE_FORMATS			(SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)

#define USE_RATE			SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000
#define USE_RATE_MIN			5500
#define USE_RATE_MAX			48000

#define USE_CHANNELS_MIN		1

#define USE_CHANNELS_MAX		2

#define USE_PERIODS_MIN			1

#define USE_PERIODS_MAX			1024

#define CMD_TIMEOUT			-1

#define SAUDIO_CMD_NONE			0x00000000
#define SAUDIO_CMD_OPEN			0x00000001
#define SAUDIO_CMD_CLOSE		0x00000002
#define SAUDIO_CMD_START		0x00000004
#define SAUDIO_CMD_STOP			0x00000008
#define SAUDIO_CMD_PREPARE		0x00000010
#define SAUDIO_CMD_TRIGGER              0x00000020

#define SAUDIO_CMD_OPEN_RET		0x00010000
#define SAUDIO_CMD_CLOSE_RET		0x00020000
#define SAUDIO_CMD_START_RET		0x00040000
#define SAUDIO_CMD_STOP_RET             0x00080000
#define SAUDIO_CMD_PREPARE_RET		0x00100000
#define SAUDIO_CMD_TRIGGER_RET		0x00200000

#define SAUDIO_DATA_PCM			0x00000040
#define SAUDIO_DATA_SILENCE		0x00000080

#define PLAYBACK_DATA_ABORT		0x00000001
#define PLAYBACK_DATA_BREAK		0x00000002

#define  SAUDIO_DEV_CTRL_ABORT		0x00000001
#define SAUDIO_DEV_CTRL_BREAK		0x00000002

#define SAUDIO_SUBCMD_CAPTURE		0x0
#define SAUDIO_SUBCMD_PLAYBACK		0x1

#define SAUDIO_DEV_MAX			1
#define SAUDIO_STREAM_MAX		2
#define SAUDIO_CARD_NAME_LEN_MAX	32

#define  SAUDIO_STREAM_BLOCK_COUNT	4
#define  SAUDIO_CMD_BLOCK_COUNT		4

struct cmd_common {
	unsigned int command;
	unsigned int sub_cmd;
	unsigned int reserved1;
	unsigned int reserved2;
};

struct cmd_prepare {
	struct cmd_common common;
	unsigned int rate;	/* rate in Hz */
	unsigned char channels;	/* channels */
	unsigned char format;
	unsigned char reserved1;
	unsigned char reserved2;
	unsigned int period;	/* period size */
	unsigned int periods;	/* periods */
};

struct cmd_open {
	struct cmd_common common;
	uint32_t stream_type;
};

struct saudio_msg {
	uint32_t command;
	uint32_t stream_id;
	uint32_t reserved1;
	uint32_t reserved2;
	void *param;
};

enum snd_status {
	SAUDIO_IDLE,
	SAUDIO_OPENNED,
	SAUDIO_CLOSED,
	SAUDIO_STOPPED,
	SAUDIO_PREPARED,
	SAUDIO_TRIGGERED,
	SAUDIO_PARAMIZED,
	SAUDIO_ABORT,
};


struct saudio_dev_ctrl;
struct snd_saudio;

struct saudio_stream {

	struct snd_saudio *saudio;
	struct snd_pcm_substream *substream;
	struct saudio_dev_ctrl *dev_ctrl;
	int stream_id;		/* numeric identification */

	uint32_t stream_state;

	uint32_t dst;
	uint32_t channel;

	int32_t period;
	int32_t periods_avail;
	int32_t periods_tosend;

	uint32_t hwptr_done;

	uint32_t last_elapsed_count;
	uint32_t last_getblk_count;
	uint32_t blk_count;

};

struct saudio_dev_ctrl {
	uint32_t dev_state;
	struct mutex mutex;
	uint32_t dst;
	uint32_t channel;
	uint8_t name[SAUDIO_CARD_NAME_LEN_MAX];
	struct saudio_stream stream[SAUDIO_STREAM_MAX];
};

struct snd_saudio {
	struct snd_card *card;
	struct snd_pcm *pcm[SAUDIO_DEV_MAX];
	struct saudio_dev_ctrl dev_ctrl[SAUDIO_DEV_MAX];
	struct platform_device *pdev;
};
static int saudio_send_common_cmd(uint32_t dst, uint32_t channel,
				      uint32_t cmd, uint32_t subcmd)
{
	int result = 0;
	struct sblock blk = { 0 };
	ADEBUG();
	pr_debug(" dst is %d, channel %d, cmd %x, subcmd %x\n", dst, channel,
		 cmd, subcmd);
	result = sblock_get(dst, channel, (struct sblock *)&blk, CMD_TIMEOUT);
	if (!result) {
		struct cmd_common *common = (struct cmd_common *)blk.addr;
		common->command = cmd;
		common->sub_cmd = subcmd;
		blk.length = sizeof(struct cmd_common);
		pr_debug(" dst is %d, channel %d, cmd %x, subcmd %x send ok\n",
			 dst, channel, cmd, subcmd);
		result = sblock_send(dst, channel, (struct sblock *)&blk);
	}
	return result;
}

static int saudio_wait_common_cmd(uint32_t dst, uint32_t channel,
				      uint32_t cmd, uint32_t subcmd)
{
	int result = 0;
	struct sblock blk = { 0 };
	struct cmd_common *common = NULL;
	ADEBUG();
	result =
	    sblock_receive(dst, channel, (struct sblock *)&blk, CMD_TIMEOUT);
	if (result) {
		ETRACE("sblock_receive dst %d, channel %d result is %d \n", dst,
		       channel, result);
		return result;
	}

	common = (struct cmd_common *)blk.addr;
	pr_debug("dst is %d, channel %d, common->command is %x ,sub cmd %x,\n",
		 dst, channel, common->command, common->sub_cmd);
	if (subcmd) {
		if ((common->command == cmd) && (common->sub_cmd == subcmd)) {
			result = 0;
		} else {
			result = -1;
		}
	} else {
		if (common->command == cmd) {
			result = 0;
		} else {
			result = -1;
		}
	}
	sblock_release(dst, channel, &blk);
	return result;
}

static int saudio_pcm_lib_malloc_pages(struct snd_pcm_substream *substream,
				       size_t size)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dmab = &substream->dma_buffer;

	/* Use the pre-allocated buffer */
	snd_pcm_set_runtime_buffer(substream, dmab);
	runtime->dma_bytes = size;
	return 1;
}

static int saudio_pcm_lib_free_pages(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int saudio_snd_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	ADEBUG();
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       substream->dma_buffer.addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_hardware snd_card_saudio_playback = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = USE_FORMATS,
	.rates = USE_RATE,
	.rate_min = USE_RATE_MIN,
	.rate_max = USE_RATE_MAX,
	.channels_min = USE_CHANNELS_MIN,
	.channels_max = USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = 64,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = USE_PERIODS_MIN,
	.periods_max = USE_PERIODS_MAX,
	.fifo_size = 0,
};

static struct snd_pcm_hardware snd_card_saudio_capture = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = USE_FORMATS,
	.rates = USE_RATE,
	.rate_min = USE_RATE_MIN,
	.rate_max = USE_RATE_MAX,
	.channels_min = USE_CHANNELS_MIN,
	.channels_max = USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = 64,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = USE_PERIODS_MIN,
	.periods_max = USE_PERIODS_MAX,
	.fifo_size = 0,
};

static int snd_card_saudio_pcm_open(struct snd_pcm_substream *substream)
{
	const struct snd_saudio *saudio = snd_pcm_substream_chip(substream);
	const int stream_id = substream->pstr->stream;
	const int dev = substream->pcm->device;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct saudio_stream *stream = NULL;
	int result = 0;
	struct saudio_dev_ctrl *dev_ctrl = NULL;
	ADEBUG();
	dev_ctrl = (struct saudio_dev_ctrl *)&(saudio->dev_ctrl[dev]);
	stream = (struct saudio_stream *)&(dev_ctrl->stream[stream_id]);
	stream->substream = substream;
	stream->stream_id = stream_id;

	stream->period = 0;
	stream->periods_tosend = 0;
	stream->periods_avail = 0;
	stream->hwptr_done = 0;
	stream->last_getblk_count = 0;
	stream->last_elapsed_count = 0;
	stream->blk_count = sblock_get_free_count(stream->dst, stream->channel);

	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = snd_card_saudio_playback;
	} else {
		runtime->hw = snd_card_saudio_capture;
	}
	mutex_lock(&dev_ctrl->mutex);
	saudio_send_common_cmd(dev_ctrl->dst, dev_ctrl->channel,
			       SAUDIO_CMD_OPEN, stream_id);
	result = saudio_wait_common_cmd(dev_ctrl->dst,
					dev_ctrl->channel,
					SAUDIO_CMD_OPEN_RET, 0);
	mutex_unlock(&dev_ctrl->mutex);

	return result;
}

static int snd_card_saudio_pcm_close(struct snd_pcm_substream *substream)
{
	const struct snd_saudio *saudio = snd_pcm_substream_chip(substream);
	const int stream_id = substream->pstr->stream;
	const int dev = substream->pcm->device;
	struct saudio_dev_ctrl *dev_ctrl = NULL;
	int result = 0;
	ADEBUG();
	dev_ctrl = (struct saudio_dev_ctrl *)&(saudio->dev_ctrl[dev]);
	mutex_lock(&dev_ctrl->mutex);
	saudio_send_common_cmd(dev_ctrl->dst, dev_ctrl->channel,
			       SAUDIO_CMD_CLOSE, stream_id);
	result =
	    saudio_wait_common_cmd(dev_ctrl->dst,
				   dev_ctrl->channel,
				   SAUDIO_CMD_CLOSE_RET, 0);
	mutex_unlock(&dev_ctrl->mutex);
	return result;

}

static int saudio_data_trigger_process(struct saudio_stream *stream,
					   struct saudio_msg *msg);

static int snd_card_saudio_pcm_trigger(struct snd_pcm_substream *substream,
				       int cmd)
{
	const struct snd_saudio *saudio = snd_pcm_substream_chip(substream);
	const int stream_id = substream->pstr->stream;
	const int dev = substream->pcm->device;
	struct saudio_dev_ctrl *dev_ctrl = NULL;
	struct saudio_stream *stream = NULL;
	struct saudio_msg msg = { 0 };
	int err = 0;
	int result = 0;
	ADEBUG();
	dev_ctrl = (struct saudio_dev_ctrl *)&(saudio->dev_ctrl[dev]);
	stream = (struct saudio_stream *)&(dev_ctrl->stream[stream_id]);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		msg.stream_id = stream_id;
		stream->stream_state = SAUDIO_TRIGGERED;
		result = saudio_data_trigger_process(stream, &msg);
		mutex_lock(&dev_ctrl->mutex);
		saudio_send_common_cmd(dev_ctrl->dst, dev_ctrl->channel,
				       SAUDIO_CMD_START, stream->stream_id);
		mutex_unlock(&dev_ctrl->mutex);

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		mutex_lock(&dev_ctrl->mutex);
		stream->stream_state = SAUDIO_STOPPED;
		saudio_send_common_cmd(dev_ctrl->dst, dev_ctrl->channel,
				       SAUDIO_CMD_STOP, stream->stream_id);
		mutex_unlock(&dev_ctrl->mutex);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return 0;
}

static int saudio_cmd_prepare_process(struct saudio_dev_ctrl *dev_ctrl,
				      struct saudio_msg *msg);
static int snd_card_saudio_pcm_prepare(struct snd_pcm_substream *substream)
{
	const struct snd_saudio *saudio = snd_pcm_substream_chip(substream);
	const int stream_id = substream->pstr->stream;
	const int dev = substream->pcm->device;
	struct saudio_dev_ctrl *dev_ctrl = NULL;
	struct saudio_msg msg = { 0 };
	int result = 0;

	ADEBUG();
	dev_ctrl = (struct saudio_dev_ctrl *)&(saudio->dev_ctrl[dev]);
	msg.command = SAUDIO_CMD_PREPARE;
	msg.stream_id = stream_id;

	result = saudio_cmd_prepare_process(dev_ctrl, &msg);
	return 0;
}

static int snd_card_saudio_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *hw_params)
{
	int32_t result = 0;
	ADEBUG();
	result =
	    saudio_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
	pr_debug("saudio.c: saudio.c: hw_params result is %d", result);
	return result;
}

static int snd_card_saudio_hw_free(struct snd_pcm_substream *substream)
{
	ADEBUG();
	return saudio_pcm_lib_free_pages(substream);
}

static snd_pcm_uframes_t snd_card_saudio_pcm_pointer(struct snd_pcm_substream
						     *substream)
{
	const struct snd_saudio *saudio = snd_pcm_substream_chip(substream);
	const int stream_id = substream->pstr->stream;
	const int dev = substream->pcm->device;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct saudio_stream *stream =
	    (struct saudio_stream *)&(saudio->dev_ctrl[dev].stream[stream_id]);
	unsigned int offset;
	offset =
	    stream->hwptr_done * frames_to_bytes(runtime, runtime->period_size);
	pr_debug("saudio_pcm_pointer: offset is %d,stream->hwptr_done is %d\n",
		 offset, stream->hwptr_done);
	return bytes_to_frames(runtime, offset);
}

static struct snd_pcm_ops snd_card_saudio_playback_ops = {
	.open = snd_card_saudio_pcm_open,
	.close = snd_card_saudio_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_card_saudio_hw_params,
	.hw_free = snd_card_saudio_hw_free,
	.prepare = snd_card_saudio_pcm_prepare,
	.trigger = snd_card_saudio_pcm_trigger,
	.pointer = snd_card_saudio_pcm_pointer,
	.mmap = saudio_snd_mmap,
};

static struct snd_pcm_ops snd_card_saudio_capture_ops = {
	.open = snd_card_saudio_pcm_open,
	.close = snd_card_saudio_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_card_saudio_hw_params,
	.hw_free = snd_card_saudio_hw_free,
	.prepare = snd_card_saudio_pcm_prepare,
	.trigger = snd_card_saudio_pcm_trigger,
	.pointer = snd_card_saudio_pcm_pointer,
	.mmap = saudio_snd_mmap,
};

void saudio_pcm_lib_preallocate_free_for_all(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int stream;
	pr_debug("saudio.c:saudio_pcm_lib_preallocate_free_for_all");
	for (stream = 0; stream < 2; stream++)
		for (substream = pcm->streams[stream].substream; substream;
		     substream = substream->next) {
			iounmap(substream->dma_buffer.area);
			smem_free(substream->dma_buffer.addr,
				  substream->dma_buffer.bytes);
			substream->dma_buffer.addr = (dma_addr_t) NULL;
			substream->dma_buffer.area = (int8_t *) NULL;
			substream->dma_buffer.bytes = 0;
		}
}

int saudio_pcm_lib_preallocate_pages_for_all(struct snd_pcm *pcm,
					     int type, void *data,
					     size_t size, size_t max)
{

	struct snd_pcm_substream *substream;
	int stream;
	pr_debug("saudio.c:saudio_pcm_lib_preallocate_pages_for_all in");
	(void)size;
	for (stream = 0; stream < SAUDIO_STREAM_MAX; stream++) {
		for (substream = pcm->streams[stream].substream; substream;
		     substream = substream->next) {
			struct snd_dma_buffer *dmab = &substream->dma_buffer;

			int addr = smem_alloc(size);

			dmab->dev.type = type;
			dmab->dev.dev = data;
			dmab->area = ioremap(addr, size);
			dmab->addr = addr;
			dmab->bytes = size;
			memset(dmab->area, 0x5a, size);
			pr_debug
			    ("saudio_pcm_lib_preallocate_pages_for_all:saudio.c: dmab addr is %x, area is %x,size is %d",
			     (uint32_t) dmab->addr, (uint32_t) dmab->area,
			     size);
			if (substream->dma_buffer.bytes > 0)
				substream->buffer_bytes_max =
				    substream->dma_buffer.bytes;
			substream->dma_max = max;
		}
	}
	return 0;
}

static int __devinit snd_card_saudio_pcm(struct snd_saudio *saudio, int device,
					 int substreams)
{
	struct snd_pcm *pcm;
	int err;
	ADEBUG();
	err = snd_pcm_new(saudio->card, "SAUDIO PCM", device,
			  substreams, substreams, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = saudio;
	saudio->pcm[device] = pcm;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_saudio_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_saudio_capture_ops);
	pcm->private_data = saudio;
	pcm->info_flags = 0;
	strcpy(pcm->name, "SAUDIO PCM");
	pcm->private_free = saudio_pcm_lib_preallocate_free_for_all;
	saudio_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
						 snd_dma_continuous_data
						 (GFP_KERNEL), MAX_BUFFER_SIZE,
						 MAX_BUFFER_SIZE);
	return 0;
}

static struct snd_saudio *saudio_card_probe(struct saudio_init_data *init_data)
{
	struct snd_saudio *saudio = NULL;
	struct snd_card *saudio_card;
	int32_t result = 0;
	result = snd_card_create(SNDRV_DEFAULT_IDX1, "VAUDIO", THIS_MODULE,
				 sizeof(struct snd_saudio), &saudio_card);

	if (result < 0)
		return NULL;
	saudio = saudio_card->private_data;
	saudio->card = saudio_card;
	saudio->dev_ctrl[0].channel = init_data->ctrl_channel;
	saudio->dev_ctrl[0].dst = init_data->dst;
	memcpy(saudio->dev_ctrl[0].name, init_data->name,
	       SAUDIO_CARD_NAME_LEN_MAX);
	saudio->dev_ctrl[0].stream[SNDRV_PCM_STREAM_PLAYBACK].channel =
	    init_data->playback_channel;
	saudio->dev_ctrl[0].stream[SNDRV_PCM_STREAM_PLAYBACK].dst =
	    init_data->dst;
	saudio->dev_ctrl[0].stream[SNDRV_PCM_STREAM_CAPTURE].channel =
	    init_data->capture_channel;
	saudio->dev_ctrl[0].stream[SNDRV_PCM_STREAM_CAPTURE].dst =
	    init_data->dst;
	return saudio;
}

static int saudio_data_trigger_process(struct saudio_stream *stream,
					   struct saudio_msg *msg)
{
	int32_t result = 0;
	struct sblock blk = { 0 };
	struct cmd_common *common = NULL;
	struct snd_pcm_runtime *runtime = stream->substream->runtime;
	ADEBUG();

	if (stream->stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		stream->periods_avail = snd_pcm_playback_avail(runtime) /
		    runtime->period_size;
	} else {
		stream->periods_avail = snd_pcm_capture_avail(runtime) /
		    runtime->period_size;
	}

	pr_debug("saudio.c:stream->periods_avail is %d,block count is %d",
		 stream->periods_avail, sblock_get_free_count(stream->dst,
							      stream->channel));

	stream->periods_tosend = runtime->periods - stream->periods_avail;

	ADEBUG();

	while (stream->periods_tosend) {

		result = sblock_get(stream->dst, stream->channel, &blk, 0);
		if (result) {
			break;
		}
		stream->last_getblk_count++;
		common = (struct cmd_common *)blk.addr;
		blk.length = frames_to_bytes(runtime, runtime->period_size);
		common->command = SAUDIO_DATA_PCM;
		common->sub_cmd = stream->stream_id;
		common->reserved1 =
		    stream->substream->dma_buffer.addr +
		    stream->period * blk.length;

		sblock_send(stream->dst, stream->channel, &blk);

		stream->period++;
		stream->period = stream->period % runtime->periods;
		stream->periods_tosend--;
	}

	pr_debug(":sblock_getblock_count trigger is %d \n",
		 stream->last_getblk_count);

	return result;
}

static int saudio_data_transfer_process(struct saudio_stream *stream,
					    struct saudio_msg *msg)
{
	struct snd_pcm_runtime *runtime = stream->substream->runtime;
	struct sblock blk = { 0 };
	int32_t result = 0;
	struct cmd_common *common = NULL;

	int32_t elapsed_blks = 0;
	int32_t periods_avail;
	int32_t periods_tosend;
	int32_t cur_blk_count = 0;

	cur_blk_count = sblock_get_free_count(stream->dst, stream->channel);

	elapsed_blks =
	    (cur_blk_count + stream->last_getblk_count - stream->blk_count) -
	    stream->last_elapsed_count;

	if (stream->stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		periods_avail = snd_pcm_playback_avail(runtime) /
		    runtime->period_size;
	} else {
		periods_avail = snd_pcm_capture_avail(runtime) /
		    runtime->period_size;
	}

	periods_tosend = stream->periods_avail - periods_avail;
	if (periods_tosend > 0) {
		stream->periods_tosend += periods_tosend;
	}

	if (stream->periods_tosend) {
		while (stream->periods_tosend) {
			result =
			    sblock_get(stream->dst, stream->channel, &blk, 0);
			if (result) {
				ETRACE
				    ("saudio.c: sblock get failed result is %d,dst is %d,channel is %d,periods_tosend is %d \n",
				     result, stream->dst, stream->channel,
				     stream->periods_tosend);
				break;
			}
			stream->last_getblk_count++;
			common = (struct cmd_common *)blk.addr;
			blk.length =
			    frames_to_bytes(runtime, runtime->period_size);
			common->command = SAUDIO_DATA_PCM;
			common->sub_cmd = stream->stream_id;
			common->reserved1 =
			    stream->substream->dma_buffer.addr +
			    stream->period * blk.length;

			sblock_send(stream->dst, stream->channel, &blk);

			stream->periods_tosend--;
			stream->period++;
			stream->period = stream->period % runtime->periods;
		}

	} else {
		pr_debug("saudio.c: saudio no data to send ");
		if (sblock_get_free_count(stream->dst, stream->channel) ==
		    SAUDIO_STREAM_BLOCK_COUNT) {
			pr_debug
			    ("saudio.c: saudio no data to send and  is empty ");
			result =
			    sblock_get(stream->dst, stream->channel, &blk, 0);
			if (result) {
				ETRACE("saudio.c: no data and no blk\n");
			} else {
				stream->last_getblk_count++;
				common = (struct cmd_common *)blk.addr;
				common->command = SAUDIO_DATA_SILENCE;
				common->sub_cmd = stream->stream_id;

				sblock_send(stream->dst, stream->channel, &blk);
				stream->last_elapsed_count++;
				schedule_timeout(msecs_to_jiffies(1));
			}
		}
	}

	while (elapsed_blks) {
		elapsed_blks--;
		stream->hwptr_done++;
		stream->hwptr_done %= runtime->periods;
		snd_pcm_period_elapsed(stream->substream);
		stream->periods_avail++;
		stream->last_elapsed_count++;
	}

	return 0;
}

static int saudio_cmd_prepare_process(struct saudio_dev_ctrl *dev_ctrl,
				      struct saudio_msg *msg)
{
	struct sblock blk;
	int32_t result = 0;
	struct snd_pcm_runtime *runtime =
	    dev_ctrl->stream[msg->stream_id].substream->runtime;
	ADEBUG();
	result =
	    sblock_get(dev_ctrl->dst, dev_ctrl->channel, (struct sblock *)&blk,
		       CMD_TIMEOUT);
	if (!result) {
		struct cmd_prepare *prepare = (struct cmd_prepare *)blk.addr;
		prepare->common.command = SAUDIO_CMD_PREPARE;
		prepare->common.sub_cmd=msg->stream_id;
		prepare->rate = runtime->rate;
		prepare->channels = runtime->channels;
		prepare->format = runtime->format;
		prepare->period =
		    frames_to_bytes(runtime, runtime->period_size);
		prepare->periods = runtime->periods;
		blk.length = sizeof(struct cmd_prepare);

		sblock_send(dev_ctrl->dst, dev_ctrl->channel,
			    (struct sblock *)&blk);
		result =
		    saudio_wait_common_cmd(dev_ctrl->dst, dev_ctrl->channel,
					   SAUDIO_CMD_PREPARE_RET,
					   0);
	}
	pr_debug("saudio_cmd_prepare_process result is %d", result);
	return result;

}

static void sblock_notifier(int event, void *data)
{
	struct saudio_stream *stream = data;
	struct saudio_msg msg = { 0 };
	int result = 0;

	if (event == SBLOCK_NOTIFY_GET) {
		if (stream->stream_state == SAUDIO_TRIGGERED) {
			result = saudio_data_transfer_process(stream, &msg);
		} else {
			pr_debug("\n: saudio is stopped\n");
		}
	}
}

static int saudio_snd_init_card(struct snd_saudio *saudio)
{
	int result = 0;
	int32_t i = 0, j = 0, err = 0;
	struct saudio_stream *stream = NULL;
	struct saudio_dev_ctrl *dev_ctrl = NULL;

	ADEBUG();

	for (i = 0; i < SAUDIO_DEV_MAX; i++) {	/* now only support  one device */
		dev_ctrl = &saudio->dev_ctrl[i];

		result =
		    sblock_create(dev_ctrl->dst, dev_ctrl->channel,
				  SAUDIO_CMD_BLOCK_COUNT, CMD_BLOCK_SIZE,
				  SAUDIO_CMD_BLOCK_COUNT, CMD_BLOCK_SIZE);
		if (result) {
			ETRACE
			    ("saudio_thread sblock create  failed result is %d\n",
			     result);
			goto __nodev;
		}
		pr_debug("saudio_thread sblock create  result is %d\n", result);

		mutex_init(&dev_ctrl->mutex);
		err = snd_card_saudio_pcm(saudio, i, 1);
		if (err < 0)
			goto __nodev;
		for (j = 0; j < SAUDIO_STREAM_MAX; j++) {
			stream = &dev_ctrl->stream[j];
			stream->dev_ctrl = dev_ctrl;

			stream->stream_state = SAUDIO_IDLE;
			stream->stream_id = j;

			result =
			    sblock_create(stream->dst, stream->channel,
					  SAUDIO_STREAM_BLOCK_COUNT,
					  TX_DATA_BLOCK_SIZE,
					  SAUDIO_STREAM_BLOCK_COUNT,
					  RX_DATA_BLOCK_SIZE);
			if (result) {
				ETRACE
				    ("saudio_thread sblock create  failed result is %d\n",
				     result);
				goto __nodev;
			}
			sblock_register_notifier(stream->dst, stream->channel,
						 sblock_notifier, stream);
			pr_debug("saudio_thread sblock create  result is %d\n",
				 result);

		}
	}
	ADEBUG();

	memcpy(saudio->card->driver, dev_ctrl->name, SAUDIO_CARD_NAME_LEN_MAX);
	memcpy(saudio->card->shortname, dev_ctrl->name,
	       SAUDIO_CARD_NAME_LEN_MAX);
	memcpy(saudio->card->longname, dev_ctrl->name,
	       SAUDIO_CARD_NAME_LEN_MAX);

	err = snd_card_register(saudio->card);
	if (err == 0) {
		ETRACE("snd_card create ok\n");
		return 0;
	}
__nodev:
	ETRACE("initialization failed\n");
	return err;
}

static int saudio_ctrl_thread(void *data)
{
	struct snd_saudio *saudio = (struct snd_saudio *)data;
	ADEBUG();
	daemonize("saudio");

	saudio_snd_init_card(saudio);

	ETRACE("saudio_ctrl_thread  create  ok\n");

	return 0;
}

static int __devinit snd_saudio_probe(struct platform_device *devptr)
{
	pid_t thread_id = (pid_t) NULL;
	struct snd_saudio *saudio = NULL;
	struct saudio_init_data *init_data = devptr->dev.platform_data;
	ADEBUG();
	if (!(saudio = saudio_card_probe(init_data))) {
		return -1;
	}
	saudio->pdev = devptr;
	platform_set_drvdata(devptr, saudio);

	thread_id = kernel_thread(saudio_ctrl_thread, saudio, 0);
	if (thread_id < 0) {
		ETRACE("virtual audio cmd kernel thread creation failure \n");
		return thread_id;
	}
	return 0;
}

static int __devexit snd_saudio_remove(struct platform_device *devptr)
{
	struct snd_saudio *saudio = platform_get_drvdata(devptr);

	if (saudio) {
		if (saudio->pdev) {
			platform_device_unregister(saudio->pdev);
		}
		if (saudio->card)
			snd_card_free(saudio->card);
	}
	return 0;
}

#define SND_SAUDIO_DRIVER	"saudio"

static struct platform_driver snd_saudio_driver = {
	.probe = snd_saudio_probe,
	.remove = __devexit_p(snd_saudio_remove),
	.driver = {
		   .name = SND_SAUDIO_DRIVER},
};

static int __init alsa_card_saudio_init(void)
{
	int err;
	err = platform_driver_register(&snd_saudio_driver);
	if (err < 0)
		return err;

	return 0;
}

static void __exit alsa_card_saudio_exit(void)
{
	platform_driver_unregister(&snd_saudio_driver);
	return;
}

module_init(alsa_card_saudio_init)
module_exit(alsa_card_saudio_exit)
