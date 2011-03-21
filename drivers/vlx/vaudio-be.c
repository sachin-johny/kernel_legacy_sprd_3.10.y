/*
 ****************************************************************
 *
 * Copyright (C) 2006-2010, VirtualLogix. All Rights Reserved.
 *
 * Use of this product is contingent on the existence of an executed license
 * agreement between VirtualLogix, or one of its sublicensee, and your
 * organization, which specifies this software's terms of use. This software
 * is here defined as VirtualLogix Intellectual Property for the purposes
 * of determining terms of use as defined within the license agreement.
 *
 * Component = ALSA audio driver on top of the nkdki
 *	       communication layer.
 *
 * Contributor(s):
 *   Pascal Piovesan (pascal.piovesan@virtuallogix.com) VirtualLogix
 *   Adam Mirowski (adam.mirowski@virtuallogix.com)
 *   Chi Dat Truong (chidat.truong@virtuallogix.com) VirtualLogix
 *
 ****************************************************************
 * #ident  "@(#)vaudio-be.c 1.9     08/03/28 VirtualLogix"
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,27)
#include <sound/driver.h>
#endif
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>

#if 0
#define VAUDIO_DEBUG
#endif

#if 1
#define VAUDIO_USE_RAUDIO
#endif

#include "vaudio.h"
#define VAUDIO_PREFIX	"VAUDIO-BE: "
#include "vaudio.c"

MODULE_DESCRIPTION("ALSA audio driver on top of VirtualLogix VLX");
MODULE_AUTHOR("Pascal Piovesan <pascal.piovesan@virtuallogix.com>");
MODULE_LICENSE("Proprietary");

/******************************************************************/

typedef enum {
    PCM_FORMAT_S8 = 0,
    PCM_FORMAT_U8,
    PCM_FORMAT_S16_LE,
    PCM_FORMAT_S16_BE,
    PCM_FORMAT_U16_LE,
    PCM_FORMAT_U16_BE,
    PCM_FORMAT_S24_LE,
    PCM_FORMAT_S24_BE,
    PCM_FORMAT_U24_LE,
    PCM_FORMAT_U24_BE,
    PCM_FORMAT_S32_LE,
    PCM_FORMAT_S32_BE,
    PCM_FORMAT_U32_LE,
    PCM_FORMAT_U32_BE
} AudioFormat;

typedef enum AudioStatus {
    AUDIO_STATUS_OK = 0,
    AUDIO_STATUS_ERROR
} AudioStatus;

    /*
     * Audio DMA channel interrupt handler.
     *
     * This handler is called back by the audio driver in the context of
     * a LISR. It must therefore limits its calls to what is allowed
     * in such a context.
     */

typedef void (*AudioCallback) (AudioStatus status, void* cookie);

    /*
     * Real Audio operations.
     */
typedef struct {
    void* (*open) (int substream, void* dma_cb_cookie);
    void  (*close)(void* cookie);

    int   (*set_sample)(void* cookie, unsigned int channels,
			AudioFormat  format, unsigned int rate,
                        unsigned int period, unsigned int periods,
                        unsigned int dma_addr);
    int   (*start_dma) (void* cookie, char* buf, unsigned int size);
    void  (*stop_dma)  (void* cookie);

    int   (*mixer_info)(int idx, struct snd_ctl_elem_info*  info);
    int   (*mixer_get) (int idx, struct snd_ctl_elem_value* val);
    int   (*mixer_put) (int idx, struct snd_ctl_elem_value* val);
} RaudioOps;

/*****************************************************************/

static const unsigned char formats[HW_PCM_FORMAT_U32_BE + 1] = {
    1, 1,
    2, 2, 2, 2,
    3, 3, 3, 3,
    4, 4, 4, 4
};

    /*
     * Audio switch driver data.
     */

#define OS_MAX      5
#define DEV_MAX     NK_VAUDIO_DEV_MAX
#define STREAM_MAX  NK_VAUDIO_STREAM_MAX
#define MIX_MAX     NK_VAUDIO_MIXER_MAX

typedef struct {
    bool              opened;	        /* open was called */
    bool              started;	        /* start was called */
    int               dev;	        /* device */
    int               stream_id;        /* playback or capture id */
    void*             session;          /* playback or capture handle */
    NkStream          stream;           /* vaudio call-back handle */
    NkEventSetRate    set_rate;         /* sample config. per OS */
    unsigned int      pending_dmas;     /* count of pending DMAs */
    struct timer_list timer_play;
} AudioStream;

static struct {
    NkOsId                owner;           /* OS currently owning the audio */
    spinlock_t            lock;
    unsigned int          prefetch;	   /* DMA needs prefetch */
    unsigned int          dev_nb;	   /* device max */
    unsigned int          stream_nb;	   /* stream_max */
    RaudioOps             audio;           /* Real audio device operations */

    nku32_f               pending_open;      /* bit mask */
    nku32_f               pending_close;     /* bit mask */
    nku32_f               pending_set_rate;  /* bit mask */
    nku32_f               pending_start;     /* bit mask */
    nku32_f               pending_stop;      /* bit mask */
    nku32_f               pending_mixer_put; /* bit mask */

    unsigned int          mix_nb;             /* mixer controls number */
    struct snd_ctl_elem_value
			 mix_val[OS_MAX][MIX_MAX]; /* mixer controls per OS */
    int                  mix_idx[OS_MAX];

    NkVaudio             vaudio[OS_MAX]; /* Virtual audio device descriptors */
    AudioStream          s[OS_MAX][DEV_MAX][STREAM_MAX];
    AudioStream*         dma_callbacks[DEV_MAX][STREAM_MAX];
    void*                hw_sessions[DEV_MAX][STREAM_MAX];
} vaudio_sw;

static struct completion vaudio_thread_completion[OS_MAX];
static struct semaphore vaudio_thread_sem[OS_MAX];
static pid_t            vaudio_thread_id[OS_MAX];
static bool		vaudio_thread_aborted;

    /*
     * Compute number of ticks to wait for in order to simulate
     * audio buffer processing.
     * <size> is the buffer size in bytes.
     */
    static unsigned int
vaudio_size_to_ticks (AudioStream* s, int size)
{
    unsigned int ticks;
    unsigned int samples;

    samples = size / (s->set_rate.channels * formats[s->set_rate.format]);
    ticks   = ((samples * HZ) / s->set_rate.rate);
    if (!ticks) {
	ticks = 1;
    }
    return ticks;
}

    /*
     * System timer handler is used to simulate audio processing
     * for the OSes which are _not owner_ of the audio device.
     */
    static void
vaudio_timer_HISR (unsigned long index)
{
    NkPhAddr     addr;
    nku32_f      size;
    int          delay;
    AudioStream* s = (AudioStream*)index;

    OTRACE ("-t-");
    if (!s->pending_dmas) {
        return;
    }
	/*
	 * Acknowledge processing of previous DATA buffer.
	 */
    vaudio_ring_put(s->stream, NK_VAUDIO_STATUS_OK);
    s->pending_dmas--;
	/*
	 * Try to process next buffer (re-arming timeout).
	 */
    if (vaudio_ring_get(s->stream, &addr, &size)) {
	s->pending_dmas++;
        delay = vaudio_size_to_ticks(s, size);
        s->timer_play.expires = jiffies + delay;
	add_timer(&s->timer_play);
    } else {
	while (s->pending_dmas) {
	    vaudio_ring_put(s->stream, NK_VAUDIO_STATUS_OK);
	    s->pending_dmas--;
	}
    }
}

#ifdef VAUDIO_USE_RAUDIO
    /*
     * Audio device DMA transfer call-back.
     * This is called when the audio buffer is processed.
     */
    static void
vaudio_LISR (AudioStatus status, void* cookie)
{
    AudioStream* s = *((AudioStream**)cookie);
    NkPhAddr     addr;
    nku32_f	 size;

    (void) status;
    OTRACE ("<d>");
    if (!s->pending_dmas) {
        return;
    }
	/*
	 * Acknowledge DATA event.
	 */
    vaudio_ring_put(s->stream, NK_VAUDIO_STATUS_OK);
    s->pending_dmas--;
	/*
	 * Try to process next buffer.
	 */
    if (vaudio_ring_get(s->stream, &addr, &size)) {
	s->pending_dmas++;
	vaudio_sw.audio.start_dma(s->session, (char*)addr, size);
    } else {
        while (s->pending_dmas) {
            vaudio_ring_put(s->stream, NK_VAUDIO_STATUS_OK);
            s->pending_dmas--;
        }
    }
}
#endif

    /*
     * Virtual audio event handler (called in HISR context).
     */
    static void
vaudio_HISR (void*         stream,
	     NkVaudioEvent event,
	     void*         params,
	     void*         cookie)
{
    const NkOsId osid = (NkOsId)cookie;
    NkVaudio     vaudio = vaudio_sw.vaudio[osid];
    NkPhAddr     addr = 0;
    nku32_f      size = 0;
    AudioStream* s = (AudioStream*)stream;

    DTRACE ("osid=%d (owner=%d)\n", osid, vaudio_sw.owner);
    switch (event) {
    case NK_VAUDIO_STREAM_OPEN: {
	NkEventOpen* arg = (NkEventOpen*)params;

	DTRACE ("open event\n");
	if (s->opened) {
	    vaudio_event_ack(vaudio, s->stream, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	    break;
	}
	if (arg->stream_type != NK_VAUDIO_ST_TYPE_PCM) {
	    vaudio_event_ack(vaudio, s->stream, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	    break;
	}
        if (arg->session_type == NK_VAUDIO_SS_TYPE_PLAYBACK) {
	    s->stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	} else if (arg->session_type == NK_VAUDIO_SS_TYPE_CAPTURE) {
	    s->stream_id = SNDRV_PCM_STREAM_CAPTURE;
	} else {
	    vaudio_event_ack(vaudio, s->stream, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	    break;
	}
	nkops.nk_atomic_set (&vaudio_sw.pending_open,
			     1 << (s->dev * STREAM_MAX + s->stream_id));
        up(&vaudio_thread_sem[osid]);
	break;
    }
    case NK_VAUDIO_STREAM_CLOSE:
	DTRACE ("close event\n");
	if (!s->opened) {
	    vaudio_event_ack(vaudio, s->stream, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	} else {
	    nkops.nk_atomic_set (&vaudio_sw.pending_close,
				 1 << (s->dev * STREAM_MAX + s->stream_id));
            up(&vaudio_thread_sem[osid]);
	}
	break;

    case NK_VAUDIO_STREAM_SET_RATE: {
	NkEventSetRate* dst = &s->set_rate;
	NkEventSetRate* evt = (NkEventSetRate*)params;

	DTRACE ("set_rate event\n");
	    /*
	     * Save a copy of the sample configuration.
	     */
	dst->channels = evt->channels;
	dst->format   = evt->format;
	dst->rate     = evt->rate;
	dst->period   = evt->period;
	dst->periods  = evt->periods;
	dst->dma_addr = evt->dma_addr;

	nkops.nk_atomic_set (&vaudio_sw.pending_set_rate,
			     1 << (s->dev * STREAM_MAX + s->stream_id));
        up(&vaudio_thread_sem[osid]);
	break;
    }
    case NK_VAUDIO_STREAM_START:
	DTRACE ("start event\n");
	if (!s->opened) {
	    vaudio_event_ack(vaudio, s->stream, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	} else {
	    nkops.nk_atomic_set (&vaudio_sw.pending_start,
				 1 << (s->dev * STREAM_MAX + s->stream_id));
            up(&vaudio_thread_sem[osid]);
	}
	break;

    case NK_VAUDIO_STREAM_DATA: {
	int      res;

	DTRACE ("data event\n");
	if (!s->started) {
	    break;
	}
	res = vaudio_ring_get(s->stream, &addr, &size);
	if (!res) {
#ifdef VAUDIO_DEBUG
	    ETRACE("DATA event and no buffer!\n");
#endif
	    break;
	}
	s->pending_dmas++;
	if (osid == vaudio_sw.owner) {
	    int j = vaudio_sw.prefetch;

	    vaudio_sw.audio.start_dma(s->session, (char*)addr, size);
	    while (j--) {
                if (vaudio_ring_get(s->stream, &addr, &size)) {
	            s->pending_dmas++;
	            vaudio_sw.audio.start_dma(s->session, (char*)addr, size);
                }
	    }
	} else {
            const int delay = vaudio_size_to_ticks(s, size);
            s->timer_play.expires = jiffies + delay;
	    add_timer(&s->timer_play);
	}
	break;
    }
    case NK_VAUDIO_STREAM_STOP:
	DTRACE ("stop event\n");
	nkops.nk_atomic_set (&vaudio_sw.pending_stop,
			     1 << (s->dev * STREAM_MAX + s->stream_id));
        up(&vaudio_thread_sem[osid]);
	break;

    case NK_VAUDIO_STREAM_MIXER: {
	NkEventMixer* evt = (NkEventMixer*)params;
	int           res = NK_VAUDIO_STATUS_OK;

	DTRACE ("mixer event idx %d cmd %d\n", evt->mix_idx, evt->mix_cmd);
        if (evt->mix_idx >= vaudio_sw.mix_nb) {
	    vaudio_event_ack(vaudio, 0, event, 0,
			     (nku32_f) NK_VAUDIO_STATUS_ERROR);
	    break;
	}
	switch (evt->mix_cmd) {
	case NK_VAUDIO_MIXER_INFO: {
	    struct snd_ctl_elem_info info;
	    int			     reserved;

	    info.value.enumerated.item = evt->mix_info.value.enumerated.item;
	    res = vaudio_sw.audio.mixer_info(evt->mix_idx, &info);

	    memcpy(evt->mix_info.name, info.id.name,
		   sizeof(evt->mix_info.name));
	    reserved = *(int*)info.reserved;
	    if (reserved) {
		memcpy(evt->mix_info.reserved, info.reserved,
		       sizeof(evt->mix_info.reserved));
	    }
	    switch (info.type) {
	    case SNDRV_CTL_ELEM_TYPE_BOOLEAN:
		evt->mix_info.type  = NK_CTL_ELEM_TYPE_BOOLEAN;
		evt->mix_info.count = info.count;
		evt->mix_info.value.integer.min = info.value.integer.min;
		evt->mix_info.value.integer.max = info.value.integer.max;
		break;

	    case SNDRV_CTL_ELEM_TYPE_INTEGER:
		evt->mix_info.type  = NK_CTL_ELEM_TYPE_INTEGER;
		evt->mix_info.count = info.count;
		evt->mix_info.value.integer.min = info.value.integer.min;
		evt->mix_info.value.integer.max = info.value.integer.max;
		break;

	    case SNDRV_CTL_ELEM_TYPE_ENUMERATED:
		evt->mix_info.type  = NK_CTL_ELEM_TYPE_ENUMERATED;
		evt->mix_info.count = info.count;
		evt->mix_info.value.enumerated.items =
			info.value.enumerated.items;
		memcpy(evt->mix_info.value.enumerated.name,
		       info.value.enumerated.name,
		       sizeof(info.value.enumerated.name));
		break;

	    default:
		break;
	    }
	    vaudio_event_ack(vaudio, 0, event, evt, res);
	    break;
	}
	case NK_VAUDIO_MIXER_GET: {
	    struct snd_ctl_elem_value* val =
		&vaudio_sw.mix_val[osid][evt->mix_idx];

	    switch (evt->mix_type) {
	    case NK_CTL_ELEM_TYPE_BOOLEAN:
	    case NK_CTL_ELEM_TYPE_INTEGER:
		memcpy(&evt->mix_val.value.integer, &val->value.integer,
		       sizeof(evt->mix_val.value.integer));
		break;

	    case NK_CTL_ELEM_TYPE_ENUMERATED:
		memcpy(&evt->mix_val.value.enumerated, &val->value.enumerated,
		       sizeof(evt->mix_val.value.enumerated));
		break;
	    }
	    vaudio_event_ack(vaudio, 0, event, evt, res);
	    break;
	}
	case NK_VAUDIO_MIXER_PUT: {
	    struct snd_ctl_elem_value* val =
		&vaudio_sw.mix_val[osid][evt->mix_idx];

	    switch (evt->mix_type) {
	    case NK_CTL_ELEM_TYPE_BOOLEAN:
	    case NK_CTL_ELEM_TYPE_INTEGER:
		memcpy(&val->value.integer, &evt->mix_val.value.integer,
		       sizeof(evt->mix_val.value.integer));
		break;

	    case NK_CTL_ELEM_TYPE_ENUMERATED:
		memcpy(&val->value.enumerated, &evt->mix_val.value.enumerated,
		       sizeof(evt->mix_val.value.enumerated));
		break;
	    }
	    if (osid == vaudio_sw.owner) {
		nkops.nk_atomic_set (&vaudio_sw.pending_mixer_put, 1 << osid);
		vaudio_sw.mix_idx[osid] = evt->mix_idx;
		up(&vaudio_thread_sem[osid]);
	    } else {
		vaudio_event_ack(vaudio, 0, event, evt, res);
	    }
	    break;
	}
	}
	break;
    }
    default:
	WTRACE ("unknown event %d\n", event);
	break;
    }
    DTRACE ("end\n");
}

    static inline void
vaudio_set_hardware_session (const int dev_id, const int stream_id,
			     void* session)
{
    vaudio_sw.hw_sessions[dev_id][stream_id] = session;
}

    static inline void*
vaudio_get_hardware_session (const int dev_id, const int stream_id)
{
    return vaudio_sw.hw_sessions[dev_id][stream_id];
}

static DEFINE_MUTEX(vaudio_hardware_stream_lock);

    static AudioStatus
vaudio_stream_open (AudioStream* audio_stream, const bool focused)
{
    const int did = audio_stream->dev;
    const int sid = audio_stream->stream_id;
    AudioStatus status = NK_VAUDIO_STATUS_OK;
    void* hw_session;

    mutex_lock(&vaudio_hardware_stream_lock);
    audio_stream->opened = 1;
    audio_stream->session = NULL;
	/*
	 * Try to open a hardware session.
	 */
    hw_session = vaudio_get_hardware_session(did, sid);
    if (hw_session == NULL) {
	    /*
	     * Create a unique hardware session.
	     */
	hw_session = vaudio_sw.audio.open(sid,
					  &vaudio_sw.dma_callbacks[did][sid]);
	vaudio_set_hardware_session(did, sid, hw_session);
    }
    if (hw_session == NULL) {
	audio_stream->opened = 0;
	status = NK_VAUDIO_STATUS_ERROR;
    } else if (focused) {
	vaudio_sw.dma_callbacks[did][sid] = audio_stream;
	audio_stream->session = hw_session;
    }
    mutex_unlock(&vaudio_hardware_stream_lock);
    return status;
}

    static void
vaudio_stream_close (AudioStream* audio_stream)
{
    int k;
    bool can_close_hw_session = 1;
    const int did = audio_stream->dev;
    const int sid = audio_stream->stream_id;

    mutex_lock(&vaudio_hardware_stream_lock);
    audio_stream->session = NULL;
    audio_stream->opened = 0;
    for (k = NK_OS_PRIM; k < OS_MAX; k++) {
	if (vaudio_sw.s[k][did][sid].opened) {
		/*
		 * We still have opened audio streams
		 * so we don't close the hardware session.
		 */
	    can_close_hw_session = 0;
	    break;
	}
    }
    if (can_close_hw_session) {
	void* hw_session = vaudio_get_hardware_session(did, sid);
	    /* We suppose here that DMA transfers are correctly stopped */
	if (hw_session) {
	    vaudio_sw.audio.close(hw_session);
	}
    }
    mutex_unlock(&vaudio_hardware_stream_lock);
}

    /*
     * Processing thread.
     */
    static int
vaudio_thread (void* data)
{
    const NkOsId osid = (NkOsId)data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    daemonize();
#else
    daemonize("vaudio-be");
#endif
    while (!vaudio_thread_aborted) {
        down(&vaudio_thread_sem[osid]);

	while (vaudio_sw.pending_open) {
	    const nku32_f j = nkops.nk_mask2bit (vaudio_sw.pending_open);
	    int res = NK_VAUDIO_STATUS_OK;
	    const int did = j / STREAM_MAX;
	    const int sid = j % STREAM_MAX;
	    AudioStream* s = &vaudio_sw.s[osid][did][sid];

	    if (!s->opened) {
		res = vaudio_stream_open(s, (osid == vaudio_sw.owner));
		if (res == NK_VAUDIO_STATUS_OK) {
		    DTRACE ("open session=%x type=%d\n",
			    (unsigned int)s->session, s->stream_id);
		}
	    }
	    nkops.nk_atomic_clear (&vaudio_sw.pending_open, 1 << j);
	    vaudio_event_ack(vaudio_sw.vaudio[osid], s->stream,
			     NK_VAUDIO_STREAM_OPEN, 0, res);
	}
	while (vaudio_sw.pending_close) {
	    const nku32_f j = nkops.nk_mask2bit (vaudio_sw.pending_close);
	    const int did = j / STREAM_MAX;
	    const int sid = j % STREAM_MAX;
	    AudioStream* s = &vaudio_sw.s[osid][did][sid];

	    if (s->opened) {
		vaudio_stream_close(s);
	    }
	    nkops.nk_atomic_clear (&vaudio_sw.pending_close, 1 << j);
	    vaudio_event_ack(vaudio_sw.vaudio[osid], s->stream,
			     NK_VAUDIO_STREAM_CLOSE, 0,
			     NK_VAUDIO_STATUS_OK);
	}
	while (vaudio_sw.pending_set_rate) {
	    const nku32_f j = nkops.nk_mask2bit (vaudio_sw.pending_set_rate);
	    int res = NK_VAUDIO_STATUS_OK;
	    const int did = j / STREAM_MAX;
	    const int sid = j % STREAM_MAX;
	    AudioStream* s = &vaudio_sw.s[osid][did][sid];
	    NkEventSetRate* evt = &s->set_rate;

	    if (osid == vaudio_sw.owner) {
		res = vaudio_sw.audio.set_sample(s->session,
			     evt->channels, evt->format,
			     evt->rate, evt->period, evt->periods,
			     evt->dma_addr);
	    }
	    nkops.nk_atomic_clear (&vaudio_sw.pending_set_rate, 1 << j);
	    vaudio_event_ack(vaudio_sw.vaudio[osid], s->stream,
			     NK_VAUDIO_STREAM_SET_RATE, 0, res);
	}
        if (vaudio_sw.pending_mixer_put) {
            const int                  idx = vaudio_sw.mix_idx[osid];
	    struct snd_ctl_elem_value* val = &vaudio_sw.mix_val[osid][idx];
            int res = vaudio_sw.audio.mixer_put(idx, val);

	    if (res >= 0) {
		res = NK_VAUDIO_STATUS_OK;
	    }
            vaudio_sw.pending_mixer_put = 0;
	    vaudio_event_ack(vaudio_sw.vaudio[osid], 0, NK_VAUDIO_STREAM_MIXER,
			     0, res);
	}
	while (vaudio_sw.pending_start) {
	    const nku32_f j = nkops.nk_mask2bit (vaudio_sw.pending_start);
	    const int did = j / STREAM_MAX;
	    const int sid = j % STREAM_MAX;
	    AudioStream* s = &vaudio_sw.s[osid][did][sid];
	    NkPhAddr addr;
	    nku32_f  size;

	    if (osid == vaudio_sw.owner) {
		int k = vaudio_sw.prefetch + 1;

		while (k--) {
		    if (vaudio_ring_get(s->stream,
					&addr, &size)){
			s->pending_dmas++;
			vaudio_sw.audio.start_dma(s->session,
						  (char*)addr, size);
		    }
		}
	    } else {
		if (vaudio_ring_get(s->stream, &addr, &size)) {
		    int delay = vaudio_size_to_ticks(s, size);
		    s->pending_dmas++;
		    s->timer_play.expires = jiffies + delay;
		    add_timer(&s->timer_play);
		}
	    }
	    s->started = 1;
	    nkops.nk_atomic_clear (&vaudio_sw.pending_start, 1 << j);
	    vaudio_event_ack(vaudio_sw.vaudio[osid], s->stream,
			     NK_VAUDIO_STREAM_START, 0,
			     NK_VAUDIO_STATUS_OK);
	}
	while (vaudio_sw.pending_stop) {
	    const nku32_f j = nkops.nk_mask2bit (vaudio_sw.pending_stop);
	    const int did = j / STREAM_MAX;
	    const int sid = j % STREAM_MAX;
	    AudioStream* s = &vaudio_sw.s[osid][did][sid];

	    if (s->started) {
		if (osid == vaudio_sw.owner) {
		    vaudio_sw.audio.stop_dma(s->session);
		} else {
		    del_timer(&s->timer_play);
		}
		while (s->pending_dmas) {
		    vaudio_ring_put(s->stream,
				    NK_VAUDIO_STATUS_OK);
		    s->pending_dmas--;
		}
		s->started = 0;
	    }
	    nkops.nk_atomic_clear (&vaudio_sw.pending_stop, 1 << j);
	}
    }
    complete_and_exit(&vaudio_thread_completion [osid], 0);
    /*NOTREACHED*/
    return 0;
}

    static void
vaudio_switch_focused_streams (const NkOsId from, const NkOsId to)
{
    NkEventSetRate* evt;
    unsigned long   flags;
    unsigned int    i, j, k;
    NkPhAddr        addr;
    nku32_f         size;
    int             delay;

    for (j = 0; j < DEV_MAX; j++) {
        for (k = 0; k < STREAM_MAX; k++) {
	    AudioStream* sfrom = &vaudio_sw.s[from][j][k];
	    AudioStream* sto = &vaudio_sw.s[to][j][k];

	    spin_lock_irqsave(&vaudio_sw.lock, flags);
		/*
		 * Stop "owner's" DMA transfer, if any.
		 */
	    if (sfrom->started) {
		vaudio_sw.audio.stop_dma(sfrom->session);
		while (sfrom->pending_dmas) {
		    vaudio_ring_put(sfrom->stream, NK_VAUDIO_STATUS_OK);
		    sfrom->pending_dmas--;
		}
	    }
		/*
		 * Stop "to's" timers, if any.
		 */
	    if (sto->started) {
		del_timer(&sto->timer_play);
		while (sto->pending_dmas) {
		    vaudio_ring_put(sto->stream, NK_VAUDIO_STATUS_OK);
		    sto->pending_dmas--;
		}
	    }
	    spin_unlock_irqrestore(&vaudio_sw.lock, flags);
		/*
		 * Switch hardware sessions.
		 */
	    vaudio_sw.owner = to;
	    vaudio_sw.dma_callbacks[j][k] = sto;
	    sto->session = vaudio_get_hardware_session(j, k);
	    sfrom->session = NULL;
		/*
		 * Apply new owner's configuration (sample and volume)
		 * then switch to new owner.
		 */
	    for (i = 0; i < vaudio_sw.mix_nb; i++) {
	        vaudio_sw.audio.mixer_put(i, &vaudio_sw.mix_val[to][i]);
	    }
	    evt = &sto->set_rate;
	    if (evt->channels) {
		(void)vaudio_sw.audio.set_sample(sto->session, evt->channels,
						 evt->format, evt->rate,
						 evt->period, evt->periods,
						 evt->dma_addr);
	    }
	    spin_lock_irqsave(&vaudio_sw.lock, flags);
		/*
		 * Notify "from", if required ... using timers.
		 */
	    if (sfrom->started) {
	        if (vaudio_ring_get(sfrom->stream, &addr, &size)) {
	            delay = vaudio_size_to_ticks(sfrom, size);
		    sfrom->pending_dmas++;
	            sfrom->timer_play.expires = jiffies + delay;
		    add_timer(&sfrom->timer_play);
		}
	    }
		/*
		 * Notify "to", if required ... using real audio device.
		 */
	    if (sto->started) {
	        i = vaudio_sw.prefetch + 1;
		while (i--) {
	            if (vaudio_ring_get(sto->stream, &addr, &size)) {
		        sto->pending_dmas++;
		        vaudio_sw.audio.start_dma(sto->session, (char*)addr,
						  size);
	            }
		}
	    }
	    spin_unlock_irqrestore(&vaudio_sw.lock, flags);
	}
    }
}

extern int focus_register_client  (struct notifier_block *nb);
extern int focus_unregister_client(struct notifier_block *nb);

static struct notifier_block focus_nb;

    /*
     * Audio device switch routine.
     */
    static int
vaudio_notify_focus (struct notifier_block* self,
		     unsigned long          event,
		     void*                  data)
{
    const NkOsId    to    = (NkOsId) event;
    const NkOsId    from  = vaudio_sw.owner;

    (void) self;
    (void) data;
    if (to != from) {
	vaudio_switch_focused_streams(from, to);
    }
    return NOTIFY_DONE;
}

    /*
     * Driver initialization.
     */

#ifdef VAUDIO_USE_RAUDIO
extern int raudio_be_callback(int, void*, void*, unsigned int*,
                              struct snd_pcm_hardware* pcm_hw);
extern int vaudio_be_register_client  (struct notifier_block *nb);
extern int vaudio_be_unregister_client(struct notifier_block *nb);

#pragma weak raudio_be_callback
#pragma weak vaudio_be_register_client
#pragma weak vaudio_be_unregister_client

static struct notifier_block vaudio_be_nb;
static int vaudio_init(void);

    static int
vaudio_be_audio_init (struct notifier_block *nb, unsigned long p1, void *p2)
{
    (void) nb;
    (void) p1;
    (void) p2;
    return vaudio_init();
}
#endif /* VAUDIO_USE_RAUDIO */

    static void
vaudio_cleanup (void)
{
    NkOsId osid;

    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_sw.vaudio[osid]) continue;
	vaudio_destroy (vaudio_sw.vaudio[osid]);
    }
}

    static int
vaudio_init (void)
{
    unsigned                 frontends = 0;
    int                      res;
    int                      i;
    NkOsId                   osid;
    struct snd_pcm_hardware* pcm_hw;
    struct snd_pcm_hardware* cur_pcm_hw;
    const int                pcm_size = sizeof(struct snd_pcm_hardware) *
				DEV_MAX * STREAM_MAX;
    NkVaudioHw*              nk_pcm_hw;
    NkVaudioHw*              cur_nk_pcm_hw;
    const int                nk_pcm_size = sizeof(NkVaudioHw) * DEV_MAX *
				STREAM_MAX;

#ifdef VAUDIO_USE_RAUDIO
    if (!raudio_be_callback || !vaudio_be_register_client ||
	!vaudio_be_unregister_client) {
	ETRACE ("No VLX-adapted real audio driver.\n");
	return -EINVAL;
    }
#endif
        /*
	 * Open real audio driver.
	 */
    DTRACE ("opening real audio driver\n");
    pcm_hw = (struct snd_pcm_hardware*)kmalloc(pcm_size, GFP_KERNEL);
    if (pcm_hw == 0) {
	return -ENOMEM;
    }
    nk_pcm_hw = (NkVaudioHw*)kmalloc(nk_pcm_size, GFP_KERNEL);
    if (nk_pcm_hw == 0) {
	kfree (pcm_hw);
	return -ENOMEM;
    }
    spin_lock_init(&vaudio_sw.lock);
#ifdef VAUDIO_USE_RAUDIO
    res = raudio_be_callback(-1, &vaudio_sw.audio, vaudio_LISR,
			     &vaudio_sw.prefetch, pcm_hw);
    if (res) {
	TRACE ("No AUDIO driver found, waiting ...\n");
        kfree(pcm_hw);
        kfree(nk_pcm_hw);
        vaudio_be_nb.notifier_call = vaudio_be_audio_init;
        res = vaudio_be_register_client(&vaudio_be_nb);
        if (res) {
	    ETRACE ("vaudio_be_register_client() failed (%d)", res);
	    return res;
        }
	return 0;
    }
    if (vaudio_be_nb.notifier_call) {
        vaudio_be_unregister_client(&vaudio_be_nb);
	vaudio_be_nb.notifier_call = NULL;
    }
#endif
        /*
	 * Register as a switchable HID device.
	 */
    DTRACE ("registering HID switch\n");
    focus_nb.notifier_call = vaudio_notify_focus;
    res = focus_register_client(&focus_nb);
    if (res) {
	ETRACE ("focus_register_client() failed (%d)\n", res);
#ifdef VAUDIO_USE_RAUDIO
	raudio_be_callback (-2, NULL, NULL, NULL, NULL); /* Disconnect */
#endif
        kfree(pcm_hw);
        kfree(nk_pcm_hw);
	return res;
    }
        /*
	 * Create virtual audio devices.
	 */
    cur_pcm_hw = pcm_hw;
    cur_nk_pcm_hw = nk_pcm_hw;
    for (i = 0; i < DEV_MAX; i++) {
	int j;

        for (j = 0; j < STREAM_MAX; j++) {
	    cur_nk_pcm_hw->pcm.formats      = cur_pcm_hw->formats;
	    cur_nk_pcm_hw->pcm.rates        = cur_pcm_hw->rates;
	    cur_nk_pcm_hw->pcm.rate_min     = cur_pcm_hw->rate_min;
	    cur_nk_pcm_hw->pcm.rate_max     = cur_pcm_hw->rate_max;
	    cur_nk_pcm_hw->pcm.channels_min = cur_pcm_hw->channels_min;
	    cur_nk_pcm_hw->pcm.channels_max = cur_pcm_hw->channels_max;
	    cur_nk_pcm_hw->pcm.buffer_bytes_max = cur_pcm_hw->buffer_bytes_max;
	    cur_nk_pcm_hw->pcm.period_bytes_min = cur_pcm_hw->period_bytes_min;
	    cur_nk_pcm_hw->pcm.period_bytes_max = cur_pcm_hw->period_bytes_max;
	    cur_nk_pcm_hw->pcm.periods_min = cur_pcm_hw->periods_min;
	    cur_nk_pcm_hw->pcm.periods_max = cur_pcm_hw->periods_max;
	    cur_nk_pcm_hw->pcm.fifo_size = cur_pcm_hw->fifo_size;
	    cur_nk_pcm_hw->stream_cap    = HW_CAP_PCM;
	    cur_nk_pcm_hw++;
	    cur_pcm_hw++;
	}
    }
    vaudio_sw.owner  = nkops.nk_id_get();
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_configured (osid)) continue;
	DTRACE ("creating vaudio (os=%d)\n", osid);
	    /*
	     * vaudio_HISR() can signal this semaphore, so initialize
	     * it before creating the handler.
	     */
	sema_init(&vaudio_thread_sem[osid], 0);
	vaudio_sw.vaudio[osid] = vaudio_create (osid, vaudio_HISR,
						(void*)osid, nk_pcm_hw);
	if (!vaudio_sw.vaudio[osid]) {
	    WTRACE ("vaudio_create(osid=%d) failed\n", osid);
	    continue;
	}
	++frontends;
	for (i = 0; i < DEV_MAX; i++) {
	    int j;

	    for (j = 0; j < STREAM_MAX; j++) {
		vaudio_sw.vaudio[osid]->stream[i][j].cookie =
		    &vaudio_sw.s[osid][i][j];
		vaudio_sw.s[osid][i][j].stream =
		    &vaudio_sw.vaudio[osid]->stream[i][j];
		vaudio_sw.hw_sessions[i][j] = NULL;
		vaudio_sw.dma_callbacks[i][j] = NULL;
	    }
	}
    }
    kfree(pcm_hw);
    kfree(nk_pcm_hw);
        /*
	 * Create timers in disabled state.
	 */
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_sw.vaudio[osid]) {
	    continue;
	}
        for (i = 0; i < DEV_MAX; i++) {
	    int j;

            for (j = 0; j < STREAM_MAX; j++) {
		AudioStream* s = &vaudio_sw.s[osid][i][j];

                init_timer(&s->timer_play);
                s->timer_play.function = vaudio_timer_HISR;
                s->timer_play.data     = (unsigned long)s;
                s->dev = i;
	    }
	}
    }
        /*
	 * Find audio controls.
	 */
    vaudio_sw.mix_nb = 0;
    for (i = 0; i < MIX_MAX; i++) {
	int j;

#ifndef VAUDIO_USE_RAUDIO
        if (!vaudio_sw.audio.mixer_get) {
	    break;
	}
#endif
        if (vaudio_sw.audio.mixer_get(i, &vaudio_sw.mix_val[0][i])) {
	    break;
        }
        for (j = NK_OS_PRIM; j < OS_MAX; j++) {
	    vaudio_sw.mix_val[j][i] = vaudio_sw.mix_val[0][i];
	}
        vaudio_sw.mix_nb++;
    }
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_sw.vaudio[osid]) {
	    continue;
	}
	if (!vaudio_start (vaudio_sw.vaudio[osid])) {
	    ETRACE ("vaudio_start() %d failed\n", osid);
	    vaudio_cleanup();
	    return -ENOMEM;
	}
    }
        /*
	 * Start the vaudio control threads.
	 */
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_sw.vaudio[osid]) {
	    continue;
	}
	init_completion (&vaudio_thread_completion [osid]);
        vaudio_thread_id[osid] = kernel_thread(vaudio_thread, (void*)osid, 0);
	    /* TBD LATER */
	if (vaudio_thread_id[osid] < 0) {
	    WTRACE("thread for guest %d did not start\n", osid);
	}
    }
    TRACE ("module loaded, %u frontend(s)\n", frontends);
    return 0;
}

    static void __exit
vaudio_exit (void)
{
    NkOsId osid;

    vaudio_thread_aborted = 1;
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	if (!vaudio_sw.vaudio[osid]) {
	    continue;
	}
	up (&vaudio_thread_sem [osid]);
	if (vaudio_thread_id[osid] >= 0) {
	    wait_for_completion (&vaudio_thread_completion [osid]);
	}
    }
    for (osid = NK_OS_PRIM; osid < OS_MAX; osid++) {
	int i;

	if (!vaudio_sw.vaudio[osid]) {
	    continue;
	}
        for (i = 0; i < DEV_MAX; i++) {
	    int j;

            for (j = 0; j < STREAM_MAX; j++) {
		del_timer_sync (&vaudio_sw.s[osid][i][j].timer_play);
	    }
	}
    }
    focus_unregister_client (&focus_nb);
#ifdef VAUDIO_USE_RAUDIO
    if (vaudio_be_nb.notifier_call) {
	vaudio_be_unregister_client (&vaudio_be_nb);
    }
    raudio_be_callback (-2, NULL, NULL, NULL, NULL); /* Disconnect */
#endif
    vaudio_cleanup();
    TRACE ("module unloaded\n");
}

module_init(vaudio_init);
module_exit(vaudio_exit);
