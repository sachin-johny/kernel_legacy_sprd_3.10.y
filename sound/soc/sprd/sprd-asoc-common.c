/*
 * sound/soc/sprd/sprd-asoc-common.c
 *
 * SPRD ASoC Common implement -- SpreadTrum ASOC Common.
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
#include "sprd-asoc-debug.h"
#define pr_fmt(fmt) pr_sprd_fmt(" COM ") fmt
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <sound/soc.h>
#include <sound/info.h>
#include <sound/sprd-audio-hook.h>

#include "sprd-asoc-common.h"

int sprd_ext_speaker_ctrl(int id, int on)
    __attribute__ ((weak, alias("__sprd_ext_speaker_ctrl")));
EXPORT_SYMBOL(sprd_ext_speaker_ctrl);

static int __sprd_ext_speaker_ctrl(int id, int on)
{
	sp_asoc_pr_dbg("No external Speaker(%d) Hook; on=%d\n", id, on);
	return NO_HOOK;
}

int sprd_ext_headphone_ctrl(int id, int on)
    __attribute__ ((weak, alias("__sprd_ext_headphone_ctrl")));
EXPORT_SYMBOL(sprd_ext_headphone_ctrl);

static int __sprd_ext_headphone_ctrl(int id, int on)
{
	sp_asoc_pr_dbg("No external headphone(%d) Hook; on=%d\n", id, on);
	return NO_HOOK;
}

int sprd_ext_earpiece_ctrl(int id, int on)
    __attribute__ ((weak, alias("__sprd_ext_earpiece_ctrl")));
EXPORT_SYMBOL(sprd_ext_earpiece_ctrl);

static int __sprd_ext_earpiece_ctrl(int id, int on)
{
	sp_asoc_pr_dbg("No external earpiece(%d) Hook; on=%d\n", id, on);
	return NO_HOOK;
}

int sprd_ext_mic_ctrl(int id, int on)
    __attribute__ ((weak, alias("__sprd_ext_mic_ctrl")));
EXPORT_SYMBOL(sprd_ext_mic_ctrl);

static int __sprd_ext_mic_ctrl(int id, int on)
{
	sp_asoc_pr_dbg("No external mic(%d) Hook; on=%d\n", id, on);
	return NO_HOOK;
}

int sprd_ext_fm_ctrl(int id, int on)
    __attribute__ ((weak, alias("__sprd_ext_fm_ctrl")));
EXPORT_SYMBOL(sprd_ext_fm_ctrl);

static int __sprd_ext_fm_ctrl(int id, int on)
{
	sp_asoc_pr_dbg("No external fm(%d) Hook; on=%d\n", id, on);
	return NO_HOOK;
}

/* spreadtrum audio debug */

static int sp_audio_debug_flag = SP_AUDIO_DEBUG_DEFAULT;

inline int get_sp_audio_debug_flag(void)
{
	return sp_audio_debug_flag;
}

EXPORT_SYMBOL(get_sp_audio_debug_flag);

static void snd_pcm_sprd_debug_read(struct snd_info_entry *entry,
				    struct snd_info_buffer *buffer)
{
	int *p_sp_audio_debug_flag = entry->private_data;
	snd_iprintf(buffer, "0x%08x\n", *p_sp_audio_debug_flag);
}

static void snd_pcm_sprd_debug_write(struct snd_info_entry *entry,
				     struct snd_info_buffer *buffer)
{
	int *p_sp_audio_debug_flag = entry->private_data;
	char line[64];
	if (!snd_info_get_line(buffer, line, sizeof(line)))
		*p_sp_audio_debug_flag = simple_strtoul(line, NULL, 16);
}

int sprd_audio_debug_init(struct snd_card *card)
{
	struct snd_info_entry *entry;
	if ((entry = snd_info_create_card_entry(card, "asoc-sprd-debug",
						card->proc_root)) != NULL) {
		entry->c.text.read = snd_pcm_sprd_debug_read;
		entry->c.text.write = snd_pcm_sprd_debug_write;
		entry->mode |= S_IWUSR;
		entry->private_data = &sp_audio_debug_flag;
		if (snd_info_register(entry) < 0) {
			snd_info_free_entry(entry);
			entry = NULL;
		}
	}
	return 0;
}

EXPORT_SYMBOL(sprd_audio_debug_init);
