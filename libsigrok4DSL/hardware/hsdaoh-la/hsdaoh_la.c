/*
 * hsdaoh-la driver for ALL LOGIC (libsigrok4DSL).
 *
 * Copyright (C) 2026 Do Van Dung <do_van_dzung@yahoo.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * hsdaoh-la: logic analyzer made of a Raspberry Pi Pico 2 (RP2350) running
 * an hsdaoh-rp2350 logic analyzer firmware and an MS2130 USB3 HDMI capture
 * stick. The Pico samples its inputs continuously and streams every sample
 * over HDMI; libhsdaoh receives the stream on the PC. Firmwares:
 *   - la8:  GP0..GP7, up to 140 MS/s (two samples per 16-bit link word)
 *   - la4:  GP0..GP3, up to 280 MS/s (four samples per link word)
 *   - la48: either of the two, switched over the USB serial port with
 *           "ch 8" / "ch 4"; offered here as the device's channel mode
 * Every buffer is unpacked by the stream format the device reports
 * (PIO_8BIT or PIO_4BIT), so the data is right whatever the firmware.
 *
 * Port of the libsigrok hsdaoh-la driver (github.com/dovandung/libsigrok,
 * branch hsdaoh-la) to libsigrok4DSL. Differences from the libsigrok
 * driver, all due to the host:
 *   - discovery: the MS2130 stick is found in ALL LOGIC's own USB device
 *     list (so hotplug add/remove works); its libhsdaoh index is its
 *     position among the known hsdaoh sticks in that list
 *   - trigger: the DSView simple trigger (per channel 0/1/R/F/C) and the
 *     trigger position slider (percent of the samples before the trigger)
 *   - data: sent as LA_CROSS_DATA (64-sample planes per enabled channel)
 *   - the Pico's serial port cannot be chosen in ALL LOGIC: it is found by
 *     its USB IDs (or HSDAOH_LA_PORT, e.g. COM5)
 *   - DSView ignores sample rate META packets, so without the Pico's serial
 *     port the stream is started when the device is opened, to read the
 *     rate from the HDMI metadata before the first capture
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>
#include <hsdaoh.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "../common/capture_core.h"
#include "pico_serial.h"

#undef LOG_PREFIX
#define LOG_PREFIX "hsdaoh-la: "

/* at most GP0..GP7, one byte per sample in the driver */
#define HL_NUM_CHANNELS		8

/* la48 channel modes (SR_CONF_CHANNEL_MODE) */
enum {
	HL_MODE_8CH = 0,
	HL_MODE_4CH = 1,
};

/* the stick's USB IDs (libhsdaoh known_devices) */
static const struct {
	uint16_t vid, pid;
	const char *name;
} sticks[] = {
	{ 0x345f, 0x2130, "MS2130" },
	{ 0x345f, 0x2131, "MS2131" },
	{ 0x345f, 0x2132, "MS2130S" },
	{ 0x345f, 0x2133, "MS2131S" },
};

/* USB IDs of the Pico SDK's USB serial port (stdio_usb) */
#define PICO_USB_VID		0x2e8a
#define PICO_USB_PID_RP2350	0x0009
#define PICO_USB_PID_RP2040	0x000a

/* used until the device has reported its rate in the stream metadata */
#define DEFAULT_SAMPLERATE	SR_MHZ(140)
/* the firmware's divider range */
#define MAX_DIVIDER		65535
/* how long dev_open waits for the stream's sample rate (no serial port) */
#define RATE_WAIT_MS		1500
/*
 * Largest capture, 1 byte per sample in the driver plus ALL LOGIC's own
 * copy. A 32-bit process has 2 GB of address space at most.
 */
#if GLIB_SIZEOF_VOID_P >= 8
#define MAX_SAMPLES		4000000000ULL
#else
#define MAX_SAMPLES		400000000ULL
#endif
/* samples per LA_CROSS_DATA packet sent to the session (multiple of 64) */
#define SEND_CHUNK		(1024 * 1024)
#define DEFAULT_SAMPLES		SR_Mn(1)

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
};

/* la48: the same plus the channel mode (8 or 4 channels) */
static const int32_t hwoptions_la48[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions_la48[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

struct hl_context {
	const struct sr_dev_inst *sdi;
	struct libusb_device *usb_dev;	/* in ALL LOGIC's libusb context */
	uint32_t hsdaoh_index;
	hsdaoh_dev_t *hsdaoh;
	/*
	 * Once started, the HDMI stream keeps running until the device is
	 * closed: no new synchronization (and no lost first frames) for every
	 * Run. Between acquisitions the data callback ignores the data.
	 */
	gboolean streaming;
	volatile gboolean stream_error;	/* libhsdaoh reported a device error */
	int skip_bufs;			/* buffers to ignore after a rate change */

	/* optional Pico USB serial port for setting the sample rate */
	char *port;
	uint64_t sys_clock;		/* Pico system clock in Hz (0 = unknown) */
	unsigned int min_div;		/* smallest divider in the current mode */
	unsigned int cur_div;

	/* channels: la8 8, la4 4, la48 8 or 4 (its channel mode) */
	unsigned int channels;		/* channels of the device (8 for la48) */
	unsigned int fw_channels;	/* channels the firmware samples now */
	gboolean switchable;		/* la48 */
	unsigned int min_div8, min_div4;	/* la48: minimum per mode */
	char mode_name[2][48];
	struct sr_list_item mode_list[3];
	volatile unsigned int stream_ch; /* 8 / 4 from the stream format, 0 unknown */
	uint64_t *samplerate_list;
	unsigned int num_samplerates;

	/* settings */
	uint64_t samplerate;		/* last known actual sample rate */
	uint64_t limit_samples;
	int max_height;

	/* capture: written by the libhsdaoh thread under `mutex`, read by the
	 * session thread once the state is HL_DONE / HL_FAILED */
	GMutex mutex;
	struct hl_capture cap;
	volatile uint32_t stream_srate;	/* sample rate from the stream metadata */
	gboolean running;		/* acquisition started, END not sent yet */
	volatile gboolean stop_req;	/* Stop pressed */
	uint8_t en_bits[HL_NUM_CHANNELS];
	int en_count;
	uint8_t *cross_buf;
};

SR_PRIV struct sr_dev_driver hsdaoh_la_driver_info;
static struct sr_dev_driver *di = &hsdaoh_la_driver_info;

/* ------------------------------------------------------------ helpers */

static int known_stick(uint16_t vid, uint16_t pid)
{
	unsigned int i;

	for (i = 0; i < G_N_ELEMENTS(sticks); i++)
		if (sticks[i].vid == vid && sticks[i].pid == pid)
			return (int)i;
	return -1;
}

/* rate for a divider, rounded like the la8 firmware does (it reports this
 * value in the stream metadata) */
static uint64_t la8_rate(uint64_t sys_clock, uint64_t div)
{
	return (sys_clock + div / 2) / div;
}

/* "140MHz", "93.3MHz", "500kHz" */
static void rate_str(char *buf, size_t size, uint64_t rate)
{
	if (rate >= SR_MHZ(1))
		snprintf(buf, size, rate % SR_MHZ(1) ? "%.1fMHz" : "%.0fMHz",
			 (double)rate / 1e6);
	else
		snprintf(buf, size, "%.0fkHz", (double)rate / 1e3);
}

static int cmp_u64(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
	return (x > y) - (x < y);
}

/*
 * The firmware only has integer dividers: rate = sys_clock / div.
 * Offer the fastest few dividers plus the ones closest to a 1-2-5 series.
 */
static void build_samplerate_list(struct hl_context *devc)
{
	static const unsigned int steps[] = { 1, 2, 5 };
	uint64_t *list;
	unsigned int n = 0, max = 128, u = 0, i;
	uint64_t decade;
	unsigned int d;

	g_free(devc->samplerate_list);
	devc->samplerate_list = NULL;
	devc->num_samplerates = 0;
	if (!devc->sys_clock || !devc->min_div)
		return;

	list = g_malloc0(max * sizeof(uint64_t));
	for (d = devc->min_div; d < devc->min_div + 6 && d <= MAX_DIVIDER; d++)
		list[n++] = la8_rate(devc->sys_clock, d);

	for (decade = 1000; decade <= devc->sys_clock && n < max; decade *= 10) {
		for (i = 0; i < G_N_ELEMENTS(steps) && n < max; i++) {
			uint64_t target = decade * steps[i];
			uint64_t dv = (devc->sys_clock + target / 2) / target;
			if (dv < devc->min_div || dv > MAX_DIVIDER)
				continue;
			list[n++] = la8_rate(devc->sys_clock, dv);
		}
	}

	qsort(list, n, sizeof(uint64_t), cmp_u64);
	for (i = 0; i < n; i++)
		if (!u || list[i] != list[u - 1])
			list[u++] = list[i];

	devc->samplerate_list = list;
	devc->num_samplerates = u;
}

/* ---------------------------------------------------- Pico serial port */

/* Send one command to the la8 firmware; logs the problems. Returns SR_OK,
 * SR_ERR_ARG (refused) or another SR_ERR_*. */
static int pico_command(const char *port, const char *cmd, char *reply,
			size_t reply_size)
{
	int r = pico_la8_command(port, cmd, reply, reply_size);

	switch (r) {
	case PICO_LA8_OK:
		sr_dbg("Pico answer to '%s': %s", cmd, reply);
		return SR_OK;
	case PICO_LA8_REFUSED:
		return SR_ERR_ARG;
	case PICO_LA8_OPEN_ERROR:
		sr_err("Cannot open the Pico's serial port %s (in use by "
		       "another program?).", port);
		return SR_ERR_CALL_STATUS;
	case PICO_LA8_IO_ERROR:
		sr_err("Writing to / reading from %s failed.", port);
		return SR_ERR_CALL_STATUS;
	default:
		if (reply[0])
			sr_err("Incomplete answer from the Pico on %s: %s", port, reply);
		else
			sr_err("No answer from the Pico on %s (is la8, la4 or la48 "
			       "running?).", port);
		return SR_ERR;
	}
}

/*
 * Take over the firmware status: system clock, divider, minimum divider and
 * the current channel count, e.g.
 * "la8: GP0..GP7, 140.000 MS/s (sys 280 MHz / 2), minimum divider 2, ..."
 * "la4: GP0..GP3, 280.000 MS/s (sys 280 MHz / 1), minimum divider 1, ..."
 * la48 prints one of these for its current mode, then
 * "la48 switchable: ch 8 (GP0..GP7, minimum divider 2) or ch 4 (GP0..GP3,
 * minimum divider 1)".
 */
static int pico_parse_status(struct hl_context *devc, const char *reply)
{
	unsigned int sys_mhz, div, min_div, m8, m4;

	if (pico_la8_parse_status(reply, &sys_mhz, &div, &min_div) != 0)
		return SR_ERR;
	devc->sys_clock = (uint64_t)sys_mhz * 1000000;
	devc->cur_div = div;
	devc->min_div = min_div;
	devc->fw_channels = strstr(reply, "la4:") ? 4 : 8;
	if (pico_la8_parse_la48(reply, &m8, &m4) == 0) {
		devc->switchable = TRUE;
		devc->min_div8 = m8;
		devc->min_div4 = m4;
		devc->min_div = devc->fw_channels == 4 ? m4 : m8;
	}
	devc->samplerate = la8_rate(devc->sys_clock, div);
	return SR_OK;
}

/* la48 channel mode names, with the mode's fastest rate */
static void build_mode_list(struct hl_context *devc)
{
	char r8[16], r4[16];

	rate_str(r8, sizeof(r8), la8_rate(devc->sys_clock, devc->min_div8));
	rate_str(r4, sizeof(r4), la8_rate(devc->sys_clock, devc->min_div4));
	snprintf(devc->mode_name[0], sizeof(devc->mode_name[0]),
		 "Use Channels 0~7 (Max %s)", r8);
	snprintf(devc->mode_name[1], sizeof(devc->mode_name[1]),
		 "Use Channels 0~3 (Max %s)", r4);
	devc->mode_list[0].id = HL_MODE_8CH;
	devc->mode_list[0].name = devc->mode_name[0];
	devc->mode_list[1].id = HL_MODE_4CH;
	devc->mode_list[1].name = devc->mode_name[1];
	devc->mode_list[2].id = -1;
	devc->mode_list[2].name = NULL;
}

/* ask the firmware on `port` for its status; TRUE if it answered */
static gboolean probe_pico(struct hl_context *devc, const char *port)
{
	char reply[1024];

	if (pico_command(port, "?", reply, sizeof(reply)) == SR_OK &&
	    (strstr(reply, "la8:") || strstr(reply, "la4:")) &&
	    pico_parse_status(devc, reply) == SR_OK) {
		devc->channels = devc->switchable ? 8 : devc->fw_channels;
		if (devc->switchable) {
			build_mode_list(devc);
			sr_info("la48 on %s: system clock %" PRIu64 " Hz, %u channel "
				"mode, divider %u (minimum %u with 8 channels, %u "
				"with 4).", port, devc->sys_clock, devc->fw_channels,
				devc->cur_div, devc->min_div8, devc->min_div4);
		} else {
			sr_info("la%u on %s: system clock %" PRIu64 " Hz, divider %u "
				"(minimum %u).", devc->fw_channels, port,
				devc->sys_clock, devc->cur_div, devc->min_div);
		}
		g_free(devc->port);
		devc->port = g_strdup(port);
		build_samplerate_list(devc);
		return TRUE;
	}
	return FALSE;
}

static gboolean in_list(GSList *l, const char *name)
{
	for (; l; l = l->next)
		if (!g_ascii_strcasecmp(l->data, name))
			return TRUE;
	return FALSE;
}

/* the Windows name of a generic USB CDC serial port, e.g. a Pico's */
static gboolean generic_usb_cdc(const struct pico_port *p)
{
	return (p->description && strstr(p->description, "USB Serial Device")) ||
	       (p->name && (strstr(p->name, "ttyACM") || strstr(p->name, "usbmodem")));
}

/*
 * Look for the Pico's serial port, so the sample rate can be changed.
 * Tried in this order:
 *  1. HSDAOH_LA_PORT environment variable (e.g. COM21)
 *  2. ports with the Pico SDK's USB IDs
 *  3. other generic USB CDC ports ("USB Serial Device (COMx)" on Windows,
 *     /dev/ttyACMx on Linux), for when the USB IDs cannot be read
 * Only the status command "?" is sent.
 */
static void find_pico(struct hl_context *devc)
{
	GSList *tried = NULL, *ports, *l;
	const char *env;
	int pass;

	env = g_getenv("HSDAOH_LA_PORT");
	if (env && *env) {
		sr_info("Trying %s (HSDAOH_LA_PORT).", env);
		tried = g_slist_append(tried, g_strdup(env));
		if (probe_pico(devc, env))
			goto out;
	}

	ports = pico_port_list();
	for (pass = 0; pass < 2 && !devc->port; pass++) {
		for (l = ports; l && !devc->port; l = l->next) {
			struct pico_port *p = l->data;
			gboolean pico_ids = p->vid == PICO_USB_VID &&
				(p->pid == PICO_USB_PID_RP2350 ||
				 p->pid == PICO_USB_PID_RP2040);
			if (pass == 0 ? !pico_ids : !generic_usb_cdc(p))
				continue;
			if (in_list(tried, p->name))
				continue;
			sr_info("Trying %s port %s (%s).",
				pass == 0 ? "Pico serial" : "USB serial",
				p->name, p->description ? p->description : "");
			tried = g_slist_append(tried, g_strdup(p->name));
			probe_pico(devc, p->name);
		}
	}
	pico_port_list_free(ports);

	if (!devc->port)
		sr_warn("No la8 / la4 / la48 serial port found (tried %u). The sample rate cannot "
			"be changed: close other programs using the Pico's COM port, "
			"or set HSDAOH_LA_PORT.", g_slist_length(tried));
out:
	g_slist_free_full(tried, g_free);
}

/* ------------------------------------------------------- data callback */

/* Called by libhsdaoh from its own thread with each received buffer. */
static void hl_data_cb(hsdaoh_data_info_t *info)
{
	struct hl_context *devc = info->ctx;
	unsigned int spw;
	int r;

	if (info->stream_id != 0)
		return;

	g_mutex_lock(&devc->mutex);

	if (info->srate)
		devc->stream_srate = info->srate;

	if (info->device_error) {
		devc->stream_error = TRUE;
		if (devc->cap.state == HL_ARMED || devc->cap.state == HL_CAPTURING)
			devc->cap.state = HL_FAILED;
		goto out;
	}
	/* just after a sample rate change: these may still hold old-rate samples */
	if (devc->skip_bufs > 0) {
		devc->skip_bufs--;
		hl_invalidate_counter(&devc->cap);
		goto out;
	}

#ifdef HSDAOH_FORMAT_PIO_4BIT
	/* libhsdaoh la >= 25951fc reports the stream format */
	spw = info->format == HSDAOH_FORMAT_PIO_4BIT ? 4 : 2;
#else
	spw = 2;
#endif
	devc->stream_ch = spw == 4 ? 4 : 8;
	r = hl_feed_spw(&devc->cap, info->buf, info->len / 2, info->data_cnt, spw);
	if (r == 1) {
		/* hl_feed restarted the pre-trigger history, and discarded a
		 * capture in progress (it would contain a hole) */
		sr_warn("Data lost in the HDMI stream (%" PRIu64 " words), "
			"waiting for the trigger again.", devc->cap.gap_words);
	} else if (r < 0) {
		sr_err("Out of memory for the HDMI stream buffer.");
	}
out:
	g_mutex_unlock(&devc->mutex);
}

/* ------------------------------------------------------- stream control */

static int stream_start(struct hl_context *devc)
{
	int ret;

	if (devc->streaming && !devc->stream_error)
		return SR_OK;
	if (devc->streaming) {
		sr_warn("The HDMI stream had an error, restarting it.");
		hsdaoh_stop_stream(devc->hsdaoh);
		devc->streaming = FALSE;
	}

	devc->stream_error = FALSE;
	ret = hsdaoh_start_stream(devc->hsdaoh, hl_data_cb, devc, 0);
	if (ret < 0) {
		/*
		 * libhsdaoh before e7509c2 (Oct 2026) cannot start a stream
		 * again on a handle whose stream was stopped (returns -2).
		 * Reopen the device and try once more; this also recovers
		 * from other start errors.
		 */
		sr_warn("Starting the hsdaoh stream failed (%d), reopening the device.", ret);
		hsdaoh_close(devc->hsdaoh);
		devc->hsdaoh = NULL;
		if (hsdaoh_open(&devc->hsdaoh, devc->hsdaoh_index) < 0) {
			devc->hsdaoh = NULL;
			ret = -1;
		} else {
			ret = hsdaoh_start_stream(devc->hsdaoh, hl_data_cb, devc, 0);
		}
	}
	if (ret < 0) {
		sr_err("Failed to start the hsdaoh stream (%d).", ret);
		return SR_ERR;
	}
	devc->streaming = TRUE;
	return SR_OK;
}

static int set_samplerate(struct hl_context *devc, uint64_t rate)
{
	char cmd[32], reply[1024];
	uint64_t div;
	int ret;

	if (!rate)
		return SR_ERR_ARG;

	if (!devc->port || !devc->sys_clock) {
		if (rate != devc->samplerate)
			sr_warn("The sample rate can only be changed with the Pico's "
				"serial port (set HSDAOH_LA_PORT, e.g. COM5); "
				"keeping %" PRIu64 " Hz.", devc->samplerate);
		return SR_OK;
	}

	div = (devc->sys_clock + rate / 2) / rate;
	if (div < devc->min_div)
		div = devc->min_div;
	if (div > MAX_DIVIDER)
		div = MAX_DIVIDER;
	if (div == devc->cur_div)
		return SR_OK;

	snprintf(cmd, sizeof(cmd), "div %u", (unsigned int)div);
	ret = pico_command(devc->port, cmd, reply, sizeof(reply));
	if (ret == SR_ERR_ARG) {
		sr_err("The Pico refused '%s': %s", cmd, reply);
		return SR_ERR_ARG;
	}
	if (ret != SR_OK)
		return ret;
	if (pico_parse_status(devc, reply) != SR_OK) {
		devc->cur_div = (unsigned int)div;
		devc->samplerate = la8_rate(devc->sys_clock, div);
	}
	/* the stream may be running: ignore the next buffers, they can still
	 * contain samples taken at the old rate */
	g_mutex_lock(&devc->mutex);
	devc->skip_bufs = 3;
	g_mutex_unlock(&devc->mutex);
	sr_info("Sample rate %" PRIu64 " Hz (divider %u).", devc->samplerate, devc->cur_div);
	return SR_OK;
}

/* DSView channel enable: the channels the firmware samples now */
static void apply_channel_count(struct hl_context *devc, struct sr_dev_inst *sdi)
{
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type == SR_CHANNEL_LOGIC)
			ch->enabled = ch->index < (int)devc->fw_channels;
	}
}

/*
 * la48: switch the firmware to 8 or 4 channels ("ch 8" / "ch 4"). The
 * firmware raises the divider to the new mode's minimum if needed, and
 * answers with its status. The sample rate list follows the mode.
 */
static int set_channel_mode(struct hl_context *devc, struct sr_dev_inst *sdi,
			    int mode)
{
	char cmd[16], reply[1024];
	unsigned int want;
	int ret;

	if (mode != HL_MODE_8CH && mode != HL_MODE_4CH)
		return SR_ERR_ARG;
	want = mode == HL_MODE_4CH ? 4 : 8;
	if (!devc->switchable || !devc->port)
		return want == devc->fw_channels ? SR_OK : SR_ERR_NA;
	if (want == devc->fw_channels)
		return SR_OK;

	snprintf(cmd, sizeof(cmd), "ch %u", want);
	ret = pico_command(devc->port, cmd, reply, sizeof(reply));
	if (ret == SR_ERR_ARG) {
		sr_err("The Pico refused '%s': %s", cmd, reply);
		return SR_ERR_ARG;
	}
	if (ret != SR_OK)
		return ret;
	if (pico_parse_status(devc, reply) != SR_OK) {
		/* no status in the answer: ask for it */
		if (pico_command(devc->port, "?", reply, sizeof(reply)) != SR_OK ||
		    pico_parse_status(devc, reply) != SR_OK) {
			sr_err("No status from the Pico after '%s'.", cmd);
			return SR_ERR;
		}
	}
	if (devc->fw_channels != want) {
		sr_err("The Pico still runs %u channels after '%s'.",
		       devc->fw_channels, cmd);
		return SR_ERR;
	}
	build_samplerate_list(devc);
	g_mutex_lock(&devc->mutex);
	devc->skip_bufs = 3;
	g_mutex_unlock(&devc->mutex);
	apply_channel_count(devc, sdi);
	sr_info("la48: %u channel mode, %" PRIu64 " Hz (divider %u, minimum %u).",
		devc->fw_channels, devc->samplerate, devc->cur_div, devc->min_div);
	return SR_OK;
}

/* ------------------------------------------------------- send to session */

static void send_cross(struct hl_context *devc, const uint8_t *src, uint64_t n)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	while (n) {
		uint64_t k = n < SEND_CHUNK ? n : SEND_CHUNK;
		uint64_t bytes = k / 64 * 8 * (uint64_t)devc->en_count;

		hl_to_cross(src, k, devc->en_bits, devc->en_count, devc->cross_buf, 1);
		memset(&packet, 0, sizeof(packet));
		memset(&logic, 0, sizeof(logic));
		packet.type = SR_DF_LOGIC;
		packet.status = SR_PKT_OK;
		packet.payload = &logic;
		logic.length = bytes;
		logic.format = LA_CROSS_DATA;
		logic.data_error = 0;
		logic.data = devc->cross_buf;
		ds_data_forward(devc->sdi, &packet);
		src += k;
		n -= k;
	}
}

static void send_trigger(struct hl_context *devc, uint64_t index)
{
	struct sr_datafeed_packet packet;
	struct ds_trigger_pos pos;

	memset(&pos, 0, sizeof(pos));
	pos.status = 1;
	pos.real_pos = (uint32_t)index;
	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_TRIGGER;
	packet.status = SR_PKT_OK;
	packet.payload = &pos;
	ds_data_forward(devc->sdi, &packet);
}

/*
 * Send the collected samples: pre-trigger part, trigger marker, rest.
 * Only whole groups of 64 samples are sent (LA_CROSS_DATA).
 */
static void send_capture(struct hl_context *devc, gboolean complete)
{
	struct hl_capture *c = &devc->cap;
	uint64_t total = c->buf_fill / 64 * 64;
	uint64_t pre = c->trigger_pos / 64 * 64;

	if (!total) {
		sr_info("No samples captured.");
		return;
	}
	if (!complete)
		sr_info("Acquisition stopped, sending %" PRIu64 " samples.", total);
	if (pre > total)
		pre = total;

	send_cross(devc, c->buf, pre);
	if (c->have_trigger)
		send_trigger(devc, c->trigger_pos);
	send_cross(devc, c->buf + pre, total - pre);
}

/*
 * End the acquisition: send what was captured if `send` is set and close
 * the data feed. The HDMI stream keeps running (it is stopped when the
 * device is closed); once the state is HL_IDLE the libhsdaoh thread no
 * longer touches the capture buffers.
 */
static void end_acquisition(struct hl_context *devc, gboolean send)
{
	struct sr_datafeed_packet packet;
	gboolean complete;
	int state;

	if (!devc->running)
		return;

	g_mutex_lock(&devc->mutex);
	state = devc->cap.state;
	devc->cap.state = HL_IDLE;
	g_mutex_unlock(&devc->mutex);

	complete = (state == HL_DONE);
	if (send && (complete || state == HL_CAPTURING))
		send_capture(devc, complete);
	if (devc->cap.gaps)
		sr_info("%" PRIu64 " gap(s) in the HDMI stream during the acquisition.",
			devc->cap.gaps);

	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);
	devc->running = FALSE;

	g_mutex_lock(&devc->mutex);
	hl_free_buffers(&devc->cap);
	g_mutex_unlock(&devc->mutex);
	g_free(devc->cross_buf);
	devc->cross_buf = NULL;
}

/* Session callback (timer): wait for the libhsdaoh thread to finish the
 * capture, or for Stop, then send everything and end the acquisition. */
static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct hl_context *devc;
	int state;

	(void)fd;
	(void)revents;

	if (!sdi || !(devc = sdi->priv))
		return TRUE;

	if (!devc->running) {
		sr_session_source_remove(-1);
		return TRUE;
	}

	g_mutex_lock(&devc->mutex);
	state = devc->cap.state;
	g_mutex_unlock(&devc->mutex);

	if (state == HL_DONE) {
		end_acquisition(devc, TRUE);
	} else if (state == HL_FAILED) {
		sr_err("hsdaoh device error, acquisition aborted.");
		end_acquisition(devc, FALSE);
	} else if (devc->stop_req) {
		/* Stop: send what has been captured after the trigger so far */
		end_acquisition(devc, TRUE);
	} else {
		return TRUE;
	}
	sr_session_source_remove(-1);
	return TRUE;
}

/* ----------------------------------------------------------- driver API */

static int hw_init(struct sr_context *sr_ctx)
{
	return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static int hw_cleanup(void)
{
	safe_free(di->priv);
	return SR_OK;
}

static void setup_channels(struct sr_dev_inst *sdi, unsigned int n)
{
	char name[16];
	int i;

	for (i = 0; i < (int)n; i++) {
		snprintf(name, sizeof(name), "%d", i);
		sdi->channels = g_slist_append(sdi->channels,
			sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE, name));
	}
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *devices = NULL;
	uint32_t index = 0;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		struct hl_context *devc;
		struct sr_dev_inst *sdi;
		struct sr_usb_dev_inst *usb_info;
		uint8_t bus, address;
		int k;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		k = known_stick(des.idVendor, des.idProduct);
		if (k < 0)
			continue;
		/* libhsdaoh numbers the sticks in USB device list order */
		index++;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;
		g_mutex_init(&devc->mutex);
		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->hsdaoh_index = index - 1;
		devc->samplerate = DEFAULT_SAMPLERATE;
		devc->limit_samples = DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->cap.state = HL_IDLE;
		devc->channels = HL_NUM_CHANNELS;
		devc->fw_channels = HL_NUM_CHANNELS;

		/* the serial port belongs to the Pico on the first stick */
		if (devc->hsdaoh_index == 0)
			find_pico(devc);

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE, "hsdaoh",
				      devc->switchable ? "la48 (RP2350 + MS2130)" :
				      (devc->port && devc->fw_channels == 4) ?
				      "la4 (RP2350 + MS2130)" : "la8 (RP2350 + MS2130)",
				      "hsdaoh-la");
		if (!sdi) {
			libusb_unref_device(devc->usb_dev);
			g_mutex_clear(&devc->mutex);
			g_free(devc);
			break;
		}
		sdi->driver = di;
		sdi->dev_type = DEV_TYPE_USB;
		sdi->priv = devc;
		devc->sdi = sdi;
		sdi->handle = (ds_device_handle)devc->usb_dev;
		bus = libusb_get_bus_number(devc->usb_dev);
		address = libusb_get_device_address(devc->usb_dev);
		usb_info = sr_usb_dev_inst_new(bus, address);
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		}
		setup_channels(sdi, devc->channels);
		apply_channel_count(devc, sdi);
		sr_info("Found %s %04x:%04x as hsdaoh #%u%s%s%s",
			sticks[k].name, des.idVendor, des.idProduct,
			devc->hsdaoh_index, !devc->port ? "" : devc->switchable ?
			", la48 on " : devc->fw_channels == 4 ? ", la4 on " :
			", la8 on ", devc->port ? devc->port : "",
			devc->port ? "" : " (firmware unknown without its serial port)");
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 0);
	return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
	GSList *l = NULL;

	(void)sdi;
	l = g_slist_append(l, (gpointer)&sr_mode_list[0]);
	return l;
}

/* Without the Pico's serial port: start the stream and read the rate the
 * device reports in the HDMI metadata, since DSView does not accept a
 * rate change after the capture. */
static void learn_stream_rate(struct hl_context *devc, struct sr_dev_inst *sdi)
{
	gint64 deadline;

	if (stream_start(devc) != SR_OK)
		return;
	deadline = g_get_monotonic_time() + RATE_WAIT_MS * 1000;
	while (!devc->stream_srate && g_get_monotonic_time() < deadline)
		g_usleep(20 * 1000);
	if (devc->stream_srate) {
		devc->samplerate = devc->stream_srate;
		sr_info("Sample rate from the HDMI stream: %" PRIu64 " Hz.",
			devc->samplerate);
		/* la4 (or la48 in 4 channel mode): only D0..D3 carry data */
		if (devc->stream_ch && devc->stream_ch != devc->fw_channels) {
			devc->fw_channels = devc->stream_ch;
			apply_channel_count(devc, sdi);
			sr_info("The HDMI stream carries %u channels.", devc->fw_channels);
		}
	} else {
		sr_warn("No sample rate in the HDMI stream yet, assuming %" PRIu64 " Hz.",
			devc->samplerate);
	}
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct hl_context *devc = sdi->priv;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;

	if (hsdaoh_open(&devc->hsdaoh, devc->hsdaoh_index) < 0) {
		sr_err("Failed to open hsdaoh device #%u (WinUSB driver installed?).",
		       devc->hsdaoh_index);
		devc->hsdaoh = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	devc->sdi = sdi;
	sdi->status = SR_ST_ACTIVE;

	if (!devc->port)
		learn_stream_rate(devc, sdi);
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct hl_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	if (devc->hsdaoh) {
		if (devc->streaming)
			hsdaoh_stop_stream(devc->hsdaoh);
		devc->streaming = FALSE;
		hsdaoh_close(devc->hsdaoh);
		devc->hsdaoh = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct hl_context *devc;

	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		hl_free(&devc->cap);
		g_free(devc->cross_buf);
		g_free(devc->samplerate_list);
		g_free(devc->port);
		g_mutex_clear(&devc->mutex);
		g_free(devc);
		sdi->priv = NULL;
	}
	if (sdi->conn) {
		sr_usb_dev_inst_free(sdi->conn);
		sdi->conn = NULL;
	}
	sr_dev_inst_free(sdi);
	return SR_OK;
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
			     struct sr_status *status, gboolean prg)
{
	(void)sdi;
	(void)prg;
	if (!status)
		return SR_ERR;
	memset(status, 0, sizeof(*status));
	return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
		      const struct sr_channel *ch,
		      const struct sr_channel_group *cg)
{
	struct hl_context *devc;

	(void)ch;
	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_int16(sdi->mode);
		break;
	case SR_CONF_UNIT_BITS:
		*data = g_variant_new_byte(1);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(maxHeights[devc->max_height]);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		*data = g_variant_new_byte(devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(MAX_SAMPLES);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16((int16_t)devc->channels);
		break;
	case SR_CONF_VLD_CH_NUM:
		/* what the firmware samples now (4 in la48's 4 channel mode) */
		*data = g_variant_new_int16((int16_t)devc->fw_channels);
		break;
	case SR_CONF_CHANNEL_MODE:
		if (!devc->switchable)
			return SR_ERR_NA;
		*data = g_variant_new_int16(devc->fw_channels == 4 ?
					    HL_MODE_4CH : HL_MODE_8CH);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_CAPTURE_RATIO:
		/* The pre-trigger samples are kept in host memory, so the
		 * trigger position is free (DSView enables its slider on this). */
		*data = g_variant_new_uint64(trigger ? ds_trigger_get_pos() : 0);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(LO_OP_STREAM);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_USB_SPEED:
		/* Actual link speed; without it ALL LOGIC shows USB 2.0. */
		if (!devc || !devc->usb_dev)
			return SR_ERR;
		*data = g_variant_new_int32(
			(int32_t)libusb_get_device_speed(devc->usb_dev));
		break;
	case SR_CONF_RLE:
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(FALSE);
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
		      struct sr_channel *ch, struct sr_channel_group *cg)
{
	struct hl_context *devc;
	uint64_t v;

	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_DEVICE_MODE:
		if (g_variant_get_int16(data) != LOGIC)
			return SR_ERR_ARG;
		sdi->mode = LOGIC;
		break;
	case SR_CONF_SAMPLERATE:
		return set_samplerate(devc, g_variant_get_uint64(data));
	case SR_CONF_CHANNEL_MODE:
		return set_channel_mode(devc, sdi, g_variant_get_int16(data));
	case SR_CONF_LIMIT_SAMPLES:
		v = g_variant_get_uint64(data);
		if (!v)
			return SR_ERR_ARG;
		if (v > MAX_SAMPLES) {
			sr_warn("Sample count %" PRIu64 " limited to %" PRIu64 ".",
				v, (uint64_t)MAX_SAMPLES);
			v = MAX_SAMPLES;
		}
		devc->limit_samples = v;
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		break;
	case SR_CONF_MAX_HEIGHT: {
		const char *s = g_variant_get_string(data, NULL);
		int i;
		for (i = 0; i < (int)G_N_ELEMENTS(maxHeights); i++) {
			if (s && strcmp(s, maxHeights[i]) == 0) {
				devc->max_height = i;
				break;
			}
		}
		break;
	}
	case SR_CONF_MAX_HEIGHT_VALUE:
		devc->max_height = g_variant_get_byte(data);
		if (devc->max_height > 4)
			devc->max_height = 4;
		break;
	case SR_CONF_OPERATION_MODE:
	case SR_CONF_LOOP_MODE:
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	struct hl_context *devc =
		(sdi && sdi->priv) ? (struct hl_context *)sdi->priv : NULL;
	static uint64_t one_rate;
	GVariantBuilder gvb;
	GVariant *gvar;

	(void)cg;
	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		if (devc && devc->switchable)
			*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"),
				hwoptions_la48, sizeof(hwoptions_la48), TRUE, NULL, NULL);
		else
			*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"),
				hwoptions, sizeof(hwoptions), TRUE, NULL, NULL);
		break;
	case SR_CONF_DEVICE_SESSIONS:
		if (devc && devc->switchable)
			*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"),
				sessions_la48, sizeof(sessions_la48), TRUE, NULL, NULL);
		else
			*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"),
				sessions, sizeof(sessions), TRUE, NULL, NULL);
		break;
	case SR_CONF_CHANNEL_MODE:
		if (!devc || !devc->switchable)
			return SR_ERR_ARG;
		*data = g_variant_new_uint64((uint64_t)(uintptr_t)devc->mode_list);
		break;
	case SR_CONF_SAMPLERATE:
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		if (devc && devc->num_samplerates) {
			gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
				devc->samplerate_list, devc->num_samplerates,
				sizeof(uint64_t));
		} else {
			/* without the Pico's serial port the rate is fixed */
			one_rate = devc ? devc->samplerate : DEFAULT_SAMPLERATE;
			gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
				&one_rate, 1, sizeof(uint64_t));
		}
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(maxHeights, G_N_ELEMENTS(maxHeights));
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_uint64((uint64_t)(uintptr_t)&opmode_list);
		break;
	default:
		return SR_ERR_ARG;
	}
	return SR_OK;
}

/* DSView simple trigger (stage TriggerStages): per channel 'X' don't care,
 * '0'/'1' level, 'R'/'F' edge, 'C' either edge. Disabled channels are
 * ignored, like in the libsigrok driver. */
static void map_trigger(const struct hl_context *devc,
			const struct sr_dev_inst *sdi, struct hl_trigger_spec *t)
{
	GSList *l;

	memset(t, 0, sizeof(*t));
	if (!trigger || !trigger->trigger_en)
		return;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		uint8_t m;

		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled ||
		    ch->index >= HL_NUM_CHANNELS)
			continue;
		if (ch->index >= (int)devc->fw_channels) {
			char c = trigger->trigger0[TriggerStages][ch->index];
			if (c == '0' || c == '1' || c == 'R' || c == 'F' || c == 'C')
				sr_warn("Trigger on channel %d ignored: the firmware "
					"samples %u channels.", ch->index,
					devc->fw_channels);
			continue;
		}
		m = (uint8_t)(1u << ch->index);
		switch (trigger->trigger0[TriggerStages][ch->index]) {
		case '0': t->must_zero |= m; break;
		case '1': t->must_one |= m; break;
		case 'R': t->rising |= m; break;
		case 'F': t->falling |= m; break;
		case 'C': t->edge |= m; break;
		default: break;
		}
	}
}

static void refresh_enabled(struct hl_context *devc, const struct sr_dev_inst *sdi)
{
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled ||
		    ch->index >= HL_NUM_CHANNELS)
			continue;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0)
		for (n = 0; n < (int)devc->fw_channels; n++)
			devc->en_bits[n] = (uint8_t)n;
	devc->en_count = n;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct hl_context *devc = sdi->priv;
	struct hl_trigger_spec spec;
	uint64_t total, pre, post;
	int ret;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	/* a failed reopen in an earlier start left no handle: try again */
	if (!devc->hsdaoh && hsdaoh_open(&devc->hsdaoh, devc->hsdaoh_index) < 0) {
		devc->hsdaoh = NULL;
		sr_err("Failed to open hsdaoh device #%u.", devc->hsdaoh_index);
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	refresh_enabled(devc, sdi);
	if (devc->en_count && devc->en_bits[devc->en_count - 1] >= devc->fw_channels)
		sr_warn("Channels %u..%u are enabled but the firmware samples %u "
			"channels: they stay low.", devc->fw_channels,
			devc->channels - 1, devc->fw_channels);
	map_trigger(devc, sdi, &spec);

	/* LA_CROSS_DATA carries whole groups of 64 samples */
	total = devc->limit_samples / 64 * 64;
	if (total < 64)
		total = 64;

	g_mutex_lock(&devc->mutex);
	ret = hl_build_trigger(&devc->cap, &spec);
	g_mutex_unlock(&devc->mutex);
	if (ret < 0)
		return SR_ERR_MALLOC;

	pre = ret == 1 ? total * ds_trigger_get_pos() / 100 : 0;
	post = total - pre;
	if (!post) {
		post = 1;
		pre = total - 1;
	}

	devc->cross_buf = g_try_malloc(SEND_CHUNK / 64 * 8 * HL_NUM_CHANNELS);
	g_mutex_lock(&devc->mutex);
	ret = devc->cross_buf ? hl_alloc(&devc->cap, pre, post, 1) : -1;
	if (ret == 0)
		hl_reset(&devc->cap);
	g_mutex_unlock(&devc->mutex);
	if (ret != 0) {
		sr_err("Not enough memory for %" PRIu64 " samples.", total);
		g_free(devc->cross_buf);
		devc->cross_buf = NULL;
		ds_set_last_error(SR_ERR_MALLOC);
		return SR_ERR_MALLOC;
	}

	devc->stop_req = FALSE;
	devc->running = TRUE;
	std_session_send_df_header(sdi, LOG_PREFIX);
	sr_session_source_add(-1, 0, 20, receive_data, sdi);

	if (stream_start(devc) != SR_OK) {
		g_mutex_lock(&devc->mutex);
		devc->cap.state = HL_FAILED;
		g_mutex_unlock(&devc->mutex);
		return SR_OK;	/* receive_data ends the acquisition */
	}

	sr_info("Waiting for %s, then %" PRIu64 " + %" PRIu64 " samples at %" PRIu64 " Hz.",
		devc->cap.have_trigger ? "the trigger" : "data", pre, post,
		devc->samplerate);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct hl_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	/* receive_data sends the data and SR_DF_END on its next tick */
	devc->stop_req = TRUE;
	return SR_OK;
}

SR_PRIV struct sr_dev_driver hsdaoh_la_driver_info = {
	.name = "hsdaoh-la",
	.longname = "hsdaoh la8/la4/la48 (RP2350 + MS2130 HDMI capture)",
	.api_version = 1,
	.driver_type = DRIVER_TYPE_HARDWARE,
	.init = hw_init,
	.cleanup = hw_cleanup,
	.scan = hw_scan,
	.dev_mode_list = hw_dev_mode_list,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = hw_dev_open,
	.dev_close = hw_dev_close,
	.dev_destroy = hw_dev_destroy,
	.dev_status_get = hw_dev_status_get,
	.dev_acquisition_start = hw_dev_acquisition_start,
	.dev_acquisition_stop = hw_dev_acquisition_stop,
	.priv = NULL,
};
