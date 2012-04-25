/*
 * audio_pa.h
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#ifndef __SOUND_AUDIO_PA
#define __SOUND_AUDIO_PA
#include <linux/platform_device.h>
typedef int (*audio_pa_callback)(u32, void *);
extern audio_pa_callback audio_pa_amplifier;
#endif
