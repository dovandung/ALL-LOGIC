/*
 * ALIENTEK (ATK) DL32 / DL32 Plus logic analyzer for ALL LOGIC.
 *
 * Copyright (C) 2026 Do Van Dzung
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
 * ALL LOGIC driver around the libsigrok alientek-dl32 protocol code.
 * Differences from the libsigrok driver, all due to the host:
 *   - detection in ALL LOGIC's own USB device list (hotplug works); the
 *     device is confirmed with the DL32 hello exchange, as 04b4:6a6a is a
 *     Cypress FX3 ID
 *   - Stream Mode / Buffer Mode is ALL LOGIC's operation mode; rates above
 *     100 MHz exist in Buffer Mode only
 *   - trigger: ALL LOGIC's simple trigger (0, 1, rising, falling, either
 *     edge per channel), which maps one to one onto the device's hardware
 *     trigger, and the trigger position slider
 *   - threshold: ALL LOGIC's VTH in volts, sent in 0.1 V steps
 *   - data: the device sends one bit stream per channel, 8 samples per
 *     byte, LSB first; that is already ALL LOGIC's LA_CROSS_DATA plane
 *     layout, so stream data is copied 8 bytes per channel and 64 samples
 *     instead of being interleaved
 *   - RLE: in Buffer Mode, the way ALL LOGIC does it for DSLogic: sample
 *     counts beyond the device memory are marked "(RLE)" and turn RLE on;
 *     if the compressed data still does not fit, the device reports it and
 *     the capture is shown as long as all channels have data
 */

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "dl32.h"

#define DL32_DEFAULT_CHANNELS	16
#define DL32_PLUS_CHANNELS	32
#define DL32_DEFAULT_SAMPLES	SR_Mn(1)
#define DL32_DEFAULT_VTH	1.6	/* 3.3 V CMOS */

/*
 * Buffer mode memory, in samples x captured channels, from ATK-Logic's DL16
 * settings (1 G for the plain model, 3.5 G for the Plus); the DL32 is
 * assumed to have the same. ALL LOGIC shows it per enabled channel.
 */
#define DL32_MEM_PLUS		(UINT64_C(3500) * SR_MHZ(1))
#define DL32_MEM		(UINT64_C(1000) * SR_MHZ(1))

/*
 * With RLE ATK-Logic offers up to 10x the memory (marked as maybe not
 * fitting); ALL LOGIC offers RLE counts up to 1024x, so longer RLE captures
 * may end early (shown with the actual length).
 */

/* samples per LA_CROSS_DATA packet when sending a buffer capture (x 64) */
#define DL32_SEND_CHUNK		(1024 * 1024)

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ LO_OP_BUFFER, "Buffer Mode" },
	{ -1, NULL },
};

static const char *dl32_speed_name(int speed)
{
	switch (speed) {
	case LIBUSB_SPEED_FULL: return "1.1 (full speed)";
	case LIBUSB_SPEED_HIGH: return "2.0 (high speed)";
	case LIBUSB_SPEED_SUPER: return "3.0 (SuperSpeed)";
	case LIBUSB_SPEED_SUPER_PLUS: return "3.x (SuperSpeed+)";
	default: return "speed unknown";
	}
}

struct dl32_context {
	const struct sr_dev_inst *sdi;
	struct sr_context *sr_ctx;
	libusb_device *usb_dev;
	libusb_device_handle *devhdl;
	struct dl32_info info;
	unsigned int num_channels;	/* 16 or 32 */
	unsigned int ch_offset;		/* first device input of the window */

	/* settings */
	uint64_t samplerate;
	uint64_t limit_samples;
	double vth;
	gboolean rle;			/* RLE asked for (Buffer Mode only) */
	int op_mode;			/* LO_OP_STREAM / LO_OP_BUFFER */
	int max_height;

	/* acquisition */
	gboolean running;
	gboolean aborted;
	gboolean data_started;
	gboolean buffer_mode;
	gboolean framed;
	gboolean rle_active;		/* this capture uses RLE */
	uint64_t actual_samples;	/* samples sent by the last capture */
	gboolean hw_trigger;
	gboolean trig_sent;
	struct libusb_transfer **transfers;
	unsigned int num_transfers;
	unsigned int submitted_transfers;
	unsigned int empty_transfer_count;
	uint64_t acq_limit;		/* samples to send, multiple of 64 */
	uint64_t pre_trigger;
	uint64_t sent_samples;
	uint32_t enabled_mask;		/* device channels captured */
	uint8_t en_bits[DL32_TRIGGER_POSITIONS];	/* channels shown, in order */
	int en_count;
	GByteArray *pending;
	struct dl32_capture *capture;
	uint8_t *group_buf;
	size_t group_bytes;
	size_t group_fill;
	uint8_t *cross_buf;
};

SR_PRIV struct sr_dev_driver alientek_dl32_driver_info;
static struct sr_dev_driver *di = &alientek_dl32_driver_info;

/* ------------------------------------------------------------ helpers */

static gboolean rate_allowed(const struct dl32_context *devc, uint64_t rate)
{
	return devc->op_mode == LO_OP_BUFFER || rate <= DL32_MAX_STREAM_RATE;
}

/* nearest allowed rate of the list for the current mode */
static uint64_t pick_rate(const struct dl32_context *devc, uint64_t want)
{
	uint64_t best = dl32_samplerates[0], best_err = UINT64_MAX;
	unsigned int i;

	for (i = 0; i < dl32_num_samplerates; i++) {
		uint64_t r = dl32_samplerates[i];
		uint64_t err = r > want ? r - want : want - r;
		if (!rate_allowed(devc, r))
			continue;
		if (err < best_err) {
			best_err = err;
			best = r;
		}
	}
	return best;
}

static int en_channel_count(const struct sr_dev_inst *sdi)
{
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type == SR_CHANNEL_LOGIC && ch->enabled)
			n++;
	}
	return n;
}

static uint64_t hw_depth(const struct sr_dev_inst *sdi)
{
	const struct dl32_context *devc = sdi->priv;
	int n = en_channel_count(sdi);
	uint64_t mem = devc->info.plus ? DL32_MEM_PLUS : DL32_MEM;

	if (n < 1)
		n = 1;
	return mem / (uint64_t)n / 64 * 64;
}

static void refresh_enabled(struct dl32_context *devc,
			    const struct sr_dev_inst *sdi)
{
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled ||
		    ch->index >= (int)devc->num_channels)
			continue;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0)
		for (n = 0; n < (int)devc->num_channels; n++)
			devc->en_bits[n] = (uint8_t)n;
	devc->en_count = n;
}

/* ------------------------------------------------------- send to session */

static void send_cross(struct dl32_context *devc, const uint8_t *data,
		       uint64_t groups)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	if (!groups)
		return;
	memset(&packet, 0, sizeof(packet));
	memset(&logic, 0, sizeof(logic));
	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = groups * 8 * (uint64_t)devc->en_count;
	logic.format = LA_CROSS_DATA;
	logic.data_error = 0;
	logic.data = (void *)data;
	ds_data_forward(devc->sdi, &packet);
	devc->sent_samples += groups * 64;
}

static void send_trigger_pos(struct dl32_context *devc, uint64_t index)
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
	devc->trig_sent = TRUE;
	sr_info("Trigger at sample %" PRIu64 ".", index);
}

/* ------------------------------------------------------------ stream mode */

/*
 * One group: a DL32_BLOCK_BYTES block per channel, each byte 8 samples of
 * that channel, LSB first. LA_CROSS_DATA wants, per 64 samples, one 8 byte
 * plane per shown channel: so copy 8 bytes per channel and 64 samples.
 */
static void send_group(struct dl32_context *devc, const uint8_t *group)
{
	uint64_t planes = DL32_BLOCK_BYTES / 8, p;
	uint8_t *out = devc->cross_buf;
	int e;

	if (devc->acq_limit) {
		uint64_t left = devc->acq_limit - devc->sent_samples;
		if (planes > left / 64)
			planes = left / 64;
	}
	for (p = 0; p < planes; p++) {
		for (e = 0; e < devc->en_count; e++) {
			memcpy(out, group + (size_t)devc->en_bits[e] * DL32_BLOCK_BYTES +
			       p * 8, 8);
			out += 8;
		}
	}
	send_cross(devc, devc->cross_buf, planes);
}

static void stream_trigger_check(struct dl32_context *devc)
{
	/* stream mode: the device reports where its trigger fired */
	if (devc->hw_trigger && !devc->trig_sent && devc->capture->have_offset &&
	    devc->capture->trigger_offset < devc->acq_limit)
		send_trigger_pos(devc, devc->capture->trigger_offset);
}

static void abort_acquisition(struct dl32_context *devc)
{
	unsigned int i;

	devc->aborted = TRUE;
	for (i = 0; i < devc->num_transfers; i++)
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
}

static void feed_raw(struct dl32_context *devc, const uint8_t *p, size_t left)
{
	while (left > 0 && !devc->aborted) {
		size_t take = MIN(left, devc->group_bytes - devc->group_fill);

		memcpy(devc->group_buf + devc->group_fill, p, take);
		devc->group_fill += take;
		p += take;
		left -= take;
		if (devc->group_fill < devc->group_bytes)
			break;
		devc->group_fill = 0;

		stream_trigger_check(devc);
		send_group(devc, devc->group_buf);
		if (devc->acq_limit && devc->sent_samples >= devc->acq_limit) {
			abort_acquisition(devc);
			break;
		}
	}
}

/* ------------------------------------------------------------ buffer mode */

/* 64 samples of one channel from bit 'bit' of its bit stream (zeros
 * past the end) */
static uint64_t load_bits64(const GByteArray *a, uint64_t bit)
{
	uint64_t q = bit >> 3, lo = 0, hi = 0;
	unsigned int b = bit & 7, k;

	for (k = 0; k < 8; k++)
		if (q + k < a->len)
			lo |= (uint64_t)a->data[q + k] << (8 * k);
	if (!b)
		return lo;
	if (q + 8 < a->len)
		hi = a->data[q + 8];
	return (lo >> b) | (hi << (64 - b));
}

/* Render 'groups' x 64 samples from 'first' as LA_CROSS_DATA. */
static void render_cross(const struct dl32_context *devc,
			 const struct dl32_plan *plan, uint64_t first,
			 uint64_t groups, uint8_t *out)
{
	uint64_t g, v;
	int e;

	for (g = 0; g < groups; g++) {
		for (e = 0; e < devc->en_count; e++) {
			unsigned int ch = devc->en_bits[e];
			int src = plan->src[ch];

			v = src < 0 ? 0 : load_bits64(devc->capture->chan[src],
					plan->start[ch] + first + g * 64);
			memcpy(out, &v, 8);	/* little endian hosts */
			out += 8;
		}
	}
}

static void flush_capture(struct dl32_context *devc)
{
	struct dl32_plan plan;
	uint64_t count, pos, n;

	if (!devc->capture)
		return;
	dl32_capture_plan(devc->capture, devc->num_channels, devc->ch_offset,
			  devc->enabled_mask, devc->buffer_mode, devc->acq_limit,
			  devc->pre_trigger, &plan);
	count = plan.count / 64 * 64;
	sr_info("Sending %" PRIu64 " collected samples (%" PRIu64
		" data frames).", count, devc->capture->frames);
	if (!count) {
		if (devc->capture->frames)
			sr_err("Collected data does not cover the requested window.");
		return;
	}
	if (devc->capture->exceed_capacity || devc->capture->exceed_bandwidth)
		sr_warn("The device reported %s.", devc->capture->exceed_capacity ?
			"running out of memory" : "exceeding its bandwidth");

	/* buffer mode: the window is trimmed so the trigger sits at the
	 * pre-trigger point */
	if (devc->hw_trigger && devc->capture->have_offset &&
	    devc->pre_trigger < count)
		send_trigger_pos(devc, devc->pre_trigger);

	if (devc->rle_active && count < devc->acq_limit)
		sr_warn("RLE capture: %" PRIu64 " of %" PRIu64 " samples fit in "
			"the device memory.", count, devc->acq_limit);

	for (pos = 0; pos < count; pos += n) {
		n = MIN((uint64_t)DL32_SEND_CHUNK, count - pos);
		render_cross(devc, &plan, pos, n / 64, devc->cross_buf);
		send_cross(devc, devc->cross_buf, n / 64);
	}
}

static void feed_framed(struct dl32_context *devc, const uint8_t *p, size_t len)
{
	dl32_feed_bytes(devc->capture, devc->pending, p, len);

	if (devc->capture->frames)
		devc->data_started = TRUE;

	/*
	 * If lots of data arrives but none of it is framed, the device sends
	 * raw channel blocks like in stream mode. Switch to that format.
	 */
	if (!devc->capture->frames && !devc->capture->done &&
	    devc->pending->len > 4 * devc->group_bytes) {
		sr_warn("No data frames in %u bytes; treating the data as raw "
			"channel blocks.", devc->pending->len);
		devc->framed = FALSE;
		devc->data_started = TRUE;
		feed_raw(devc, devc->pending->data, devc->pending->len);
		g_byte_array_set_size(devc->pending, 0);
		return;
	}

	if (devc->capture->done)
		abort_acquisition(devc);
}

/* ------------------------------------------------------------ USB transfers */

static void free_transfer(struct libusb_transfer *transfer)
{
	struct dl32_context *devc = transfer->user_data;
	unsigned int i;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	devc->submitted_transfers--;
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;
	sr_err("Failed to resubmit transfer: %s.", libusb_error_name(ret));
	free_transfer(transfer);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct dl32_context *devc = transfer->user_data;
	const uint8_t *p;
	size_t len;

	if (devc->aborted) {
		free_transfer(transfer);
		return;
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		sr_err("Device disconnected.");
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT:	/* may still carry data */
		break;
	default:
		sr_err("Transfer error: %s.", libusb_error_name(transfer->status));
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length <= 0) {
		/* waiting for a trigger or a buffer capture is not an error */
		if (devc->data_started &&
		    ++devc->empty_transfer_count > DL32_MAX_EMPTY_XFERS) {
			sr_err("Device stopped sending data.");
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	}
	devc->empty_transfer_count = 0;

	p = transfer->buffer;
	len = (size_t)transfer->actual_length;
	if (devc->framed) {
		feed_framed(devc, p, len);
	} else if (!devc->data_started && p[0] == 0x0a &&
		   len < DL32_STATUS_MAX_LEN) {
		/* status frames arrive as their own short reads */
		dl32_consume_frames(devc->capture, p, len);
	} else {
		devc->data_started = TRUE;
		feed_raw(devc, p, len);
	}

	if (devc->aborted)
		free_transfer(transfer);
	else
		resubmit_transfer(transfer);
}

static void free_acquisition(struct dl32_context *devc)
{
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	g_free(devc->group_buf);
	devc->group_buf = NULL;
	g_free(devc->cross_buf);
	devc->cross_buf = NULL;
	if (devc->pending) {
		g_byte_array_free(devc->pending, TRUE);
		devc->pending = NULL;
	}
	dl32_capture_free(devc->capture);
	devc->capture = NULL;
}

/* All transfers are back: stop the device, send what was collected, END. */
static void finish_acquisition(struct dl32_context *devc)
{
	struct sr_datafeed_packet packet;

	if (devc->devhdl)
		dl32_send_stop(devc->devhdl);
	if (devc->framed)
		flush_capture(devc);
	else
		stream_trigger_check(devc);

	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);
	sr_info("Acquisition done, %" PRIu64 " samples sent.", devc->sent_samples);
	devc->actual_samples = devc->sent_samples;

	devc->running = FALSE;
	free_acquisition(devc);
}

static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct dl32_context *devc;
	struct timeval tv = { 0, 0 };

	(void)fd;
	(void)revents;
	if (!sdi || !(devc = sdi->priv))
		return TRUE;
	if (!devc->running) {
		sr_session_source_remove(-1);
		return TRUE;
	}

	libusb_handle_events_timeout_completed(devc->sr_ctx->libusb_ctx, &tv, NULL);

	/* finish outside libusb's callbacks */
	if (devc->submitted_transfers == 0) {
		finish_acquisition(devc);
		sr_session_source_remove(-1);
	}
	return TRUE;
}

/* ----------------------------------------------------------- driver API */

static int hw_init(struct sr_context *sr_ctx)
{
	dl32_debug = g_getenv("ALIENTEK_DL32_DEBUG") != NULL;
	return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static int hw_cleanup(void)
{
	safe_free(di->priv);
	return SR_OK;
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc = di->priv;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *devices = NULL;
	unsigned int ch_offset = 0, force_16 = 0;
	int i;

	(void)options;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	/* ALIENTEK_DL32_OFFSET=<n>: a 16 channel window starting at CH<n> */
	if (g_getenv("ALIENTEK_DL32_OFFSET")) {
		ch_offset = (unsigned int)atoi(g_getenv("ALIENTEK_DL32_OFFSET"));
		if (ch_offset > DL32_TRIGGER_POSITIONS - DL32_DEFAULT_CHANNELS ||
		    ch_offset % 2) {
			sr_err("ALIENTEK_DL32_OFFSET must be even and at most %d.",
			       DL32_TRIGGER_POSITIONS - DL32_DEFAULT_CHANNELS);
			ch_offset = 0;
		}
		force_16 = 1;
	}

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		struct dl32_context *devc;
		struct sr_dev_inst *sdi;
		struct sr_usb_dev_inst *usb_info;
		struct dl32_info info;
		libusb_device_handle *hdl;
		char name[16], *model, *version;
		unsigned int nch, j;
		int ret;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0 ||
		    des.idVendor != DL32_VID || des.idProduct != DL32_PID)
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		/* 04b4:6a6a is a Cypress FX3 ID: confirm it is a DL32 */
		if ((ret = libusb_open(devlist[i], &hdl)) != 0) {
			if (ret == LIBUSB_ERROR_NOT_SUPPORTED)
				sr_err("Found a DL32, but it uses a driver libusb "
				       "cannot open (probably Cypress cyusb3 from "
				       "ATK-Logic). Switch it to WinUSB with Zadig.");
			else
				sr_err("Found a DL32, but cannot open it: %s.",
				       libusb_error_name(ret));
			continue;
		}
		if (libusb_kernel_driver_active(hdl, DL32_USB_INTERFACE) == 1)
			libusb_detach_kernel_driver(hdl, DL32_USB_INTERFACE);
		if ((ret = libusb_claim_interface(hdl, DL32_USB_INTERFACE)) != 0) {
			sr_err("Cannot claim the DL32: %s (close ATK-Logic or any "
			       "other program using it).", libusb_error_name(ret));
			libusb_close(hdl);
			continue;
		}
		ret = dl32_identify(hdl, &info);
		libusb_release_interface(hdl, DL32_USB_INTERFACE);
		libusb_close(hdl);
		if (ret != SR_OK) {
			sr_err("Device %04x:%04x did not answer as a DL32.",
			       des.idVendor, des.idProduct);
			continue;
		}

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;
		devc->info = info;
		devc->sr_ctx = drvc->sr_ctx;
		devc->usb_dev = libusb_ref_device(devlist[i]);
		nch = (!force_16 && info.plus) ? DL32_PLUS_CHANNELS : DL32_DEFAULT_CHANNELS;
		devc->num_channels = nch;
		devc->ch_offset = nch == DL32_PLUS_CHANNELS ? 0 : ch_offset;
		devc->samplerate = SR_MHZ(10);
		devc->limit_samples = DL32_DEFAULT_SAMPLES;
		devc->vth = DL32_DEFAULT_VTH;
		devc->op_mode = LO_OP_STREAM;
		devc->max_height = 1;

		model = g_strdup_printf("%s%s", info.name, info.plus ? " Plus" : "");
		version = g_strdup_printf("MCU %d.%d", info.mcu_version / 10,
					  info.mcu_version % 10);
		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE, "ALIENTEK", model, version);
		g_free(model);
		g_free(version);
		if (!sdi) {
			libusb_unref_device(devc->usb_dev);
			g_free(devc);
			break;
		}
		sdi->driver = di;
		sdi->dev_type = DEV_TYPE_USB;
		sdi->priv = devc;
		sdi->handle = (ds_device_handle)devc->usb_dev;
		devc->sdi = sdi;
		usb_info = sr_usb_dev_inst_new(libusb_get_bus_number(devc->usb_dev),
					       libusb_get_device_address(devc->usb_dev));
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		}
		for (j = 0; j < nch; j++) {
			g_snprintf(name, sizeof(name), "%u", j);
			sdi->channels = g_slist_append(sdi->channels,
				sr_channel_new((uint16_t)j, SR_CHANNEL_LOGIC, TRUE, name));
		}
		sr_info("Found ALIENTEK %s%s, MCU %d.%d, %u channels%s, USB %s.",
			info.name, info.plus ? " Plus" : "", info.mcu_version / 10,
			info.mcu_version % 10, nch,
			devc->ch_offset ? " (offset window)" : "",
			dl32_speed_name(libusb_get_device_speed(devc->usb_dev)));
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

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct dl32_context *devc = sdi->priv;
	int ret;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;
	if ((ret = libusb_open(devc->usb_dev, &devc->devhdl)) != 0) {
		sr_err("Cannot open the DL32: %s.", libusb_error_name(ret));
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	if (libusb_kernel_driver_active(devc->devhdl, DL32_USB_INTERFACE) == 1)
		libusb_detach_kernel_driver(devc->devhdl, DL32_USB_INTERFACE);
	if ((ret = libusb_claim_interface(devc->devhdl, DL32_USB_INTERFACE)) != 0) {
		sr_err("Cannot claim the DL32: %s.", libusb_error_name(ret));
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	devc->sdi = sdi;
	sdi->status = SR_ST_ACTIVE;
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct dl32_context *devc;

	if (!sdi || !(devc = sdi->priv))
		return SR_ERR;
	if (devc->devhdl) {
		dl32_send_stop(devc->devhdl);
		libusb_release_interface(devc->devhdl, DL32_USB_INTERFACE);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct dl32_context *devc;

	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		free_acquisition(devc);
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
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
	struct dl32_context *devc;

	(void)ch;
	(void)cg;
	if (!sdi || !(devc = sdi->priv))
		return SR_ERR_ARG;

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
		*data = g_variant_new_byte((uint8_t)devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(hw_depth(sdi));
		break;
	case SR_CONF_TOTAL_CH_NUM:
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16((int16_t)devc->num_channels);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(devc->op_mode == LO_OP_STREAM);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16((int16_t)devc->op_mode);
		break;
	case SR_CONF_CAPTURE_RATIO:
		/* the device keeps the pre-trigger part: free trigger position */
		*data = g_variant_new_uint64(trigger ? ds_trigger_get_pos() : 0);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_USB_SPEED:
		/* Actual link speed; without it ALL LOGIC assumes USB 2.0. */
		if (!devc->usb_dev)
			return SR_ERR;
		*data = g_variant_new_int32(
			(int32_t)libusb_get_device_speed(devc->usb_dev));
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(devc->rle && devc->op_mode == LO_OP_BUFFER);
		break;
	case SR_CONF_RLE_SUPPORT:
		/* lets ALL LOGIC offer "(RLE)" sample counts in Buffer Mode */
		*data = g_variant_new_boolean(devc->op_mode == LO_OP_BUFFER);
		break;
	case SR_CONF_ACTUAL_SAMPLES:
		*data = g_variant_new_uint64(devc->actual_samples ?
			devc->actual_samples : devc->limit_samples);
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
		      struct sr_channel *ch, struct sr_channel_group *cg)
{
	struct dl32_context *devc;
	const char *s;
	unsigned int i;

	(void)cg;
	if (!sdi || !(devc = sdi->priv))
		return SR_ERR_ARG;

	switch (id) {
	case SR_CONF_DEVICE_MODE:
		if (g_variant_get_int16(data) != LOGIC)
			return SR_ERR_ARG;
		sdi->mode = LOGIC;
		break;
	case SR_CONF_SAMPLERATE:
		devc->samplerate = pick_rate(devc, g_variant_get_uint64(data));
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		break;
	case SR_CONF_VTH: {
		double v = g_variant_get_double(data);
		/* ATK-Logic accepts -5 .. 5 V; ALL LOGIC offers 0 .. 5 V */
		if (v < -5.0)
			v = -5.0;
		if (v > 5.0)
			v = 5.0;
		devc->vth = floor(v * 10.0 + 0.5) / 10.0;
		break;
	}
	case SR_CONF_OPERATION_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != LO_OP_STREAM && nv != LO_OP_BUFFER)
			return SR_ERR_ARG;
		devc->op_mode = nv;
		/* rates above 100 MHz exist in Buffer Mode only */
		devc->samplerate = pick_rate(devc, devc->samplerate);
		break;
	}
	case SR_CONF_MAX_HEIGHT:
		s = g_variant_get_string(data, NULL);
		for (i = 0; i < G_N_ELEMENTS(maxHeights); i++)
			if (s && !strcmp(s, maxHeights[i]))
				devc->max_height = (int)i;
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		devc->max_height = g_variant_get_byte(data);
		if (devc->max_height > 4)
			devc->max_height = 4;
		break;
	case SR_CONF_RLE:
		devc->rle = g_variant_get_boolean(data);
		break;
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
	const struct dl32_context *devc =
		(sdi && sdi->priv) ? (const struct dl32_context *)sdi->priv : NULL;
	static uint64_t rate_buf[32];
	GVariantBuilder gvb;
	GVariant *gvar;
	unsigned int i, n = 0;

	(void)cg;
	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"), hwoptions,
			sizeof(hwoptions), TRUE, NULL, NULL);
		break;
	case SR_CONF_DEVICE_SESSIONS:
		*data = g_variant_new_from_data(G_VARIANT_TYPE("ai"), sessions,
			sizeof(sessions), TRUE, NULL, NULL);
		break;
	case SR_CONF_SAMPLERATE:
		for (i = 0; i < dl32_num_samplerates && n < 31; i++)
			if (!devc || rate_allowed(devc, dl32_samplerates[i]))
				rate_buf[n++] = dl32_samplerates[i];
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"), rate_buf, n,
						 sizeof(uint64_t));
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

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct dl32_context *devc = sdi->priv;
	uint8_t params[DL32_PARAMS_LEN], trig[DL32_TRIGGER_LEN];
	uint8_t codes[DL32_TRIGGER_POSITIONS];
	uint8_t one = 1, *reply;
	uint64_t limit, device_samples, trigger_pos;
	size_t xfer_size;
	unsigned int i;
	int hz_index, ret, e;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE || !devc->devhdl) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}
	devc->sdi = sdi;

	devc->samplerate = pick_rate(devc, devc->samplerate);
	hz_index = dl32_samplerate_index(devc->samplerate);
	if (hz_index < 0) {
		sr_err("Unsupported samplerate %" PRIu64 ".", devc->samplerate);
		return SR_ERR_ARG;
	}

	/* LA_CROSS_DATA carries whole groups of 64 samples */
	limit = devc->limit_samples / 64 * 64;
	if (limit < 64)
		limit = 64;
	devc->acq_limit = limit;
	devc->buffer_mode = devc->op_mode == LO_OP_BUFFER ||
		devc->samplerate > DL32_MAX_STREAM_RATE;
	devc->framed = devc->buffer_mode;
	/* RLE in Buffer Mode (ALL LOGIC turns it on for "(RLE)" counts) */
	devc->rle_active = devc->buffer_mode && devc->rle;
	devc->actual_samples = 0;

	refresh_enabled(devc, sdi);
	/* captured device channels: stream mode always sends all of them */
	devc->enabled_mask = 0;
	if (devc->framed)
		for (e = 0; e < devc->en_count; e++)
			devc->enabled_mask |= 1u << devc->en_bits[e];
	if (!devc->enabled_mask)
		devc->enabled_mask = devc->num_channels == 32 ? 0xffffffffu :
			(1u << devc->num_channels) - 1;

	/* ALL LOGIC's simple trigger is exactly the device's hardware trigger */
	devc->hw_trigger = dl32_trigger_codes(
		(trigger && trigger->trigger_en) ? trigger->trigger0[TriggerStages] : NULL,
		devc->num_channels, devc->framed ? devc->enabled_mask :
		(devc->num_channels == 32 ? 0xffffffffu : (1u << devc->num_channels) - 1),
		codes);
	for (i = 0; i < devc->num_channels; i++) {
		if (codes[i] != DL32_TRIG_NONE)
			devc->enabled_mask |= 1u << i;
		codes[i] = (devc->enabled_mask & (1u << i)) ?
			(uint8_t)(codes[i] | DL32_TRIG_ENABLE) : 0;
	}

	/*
	 * Buffer mode bandwidth: rate x captured channels must stay within the
	 * device's budget, or it uploads unusable data. Measured on a DL32
	 * Plus: 32 ch works at 250 MHz but not 400 MHz; 16 ch works at 400 MHz.
	 */
	if (devc->buffer_mode) {
		unsigned int nen = 0;
		uint64_t budget = devc->info.plus ? DL32_PLUS_BUDGET : DL32_BUDGET;

		for (i = 0; i < devc->num_channels; i++)
			nen += (devc->enabled_mask >> i) & 1;
		if (nen && devc->samplerate * nen > budget) {
			sr_err("At %" PRIu64 " MHz the %s can capture at most %"
			       PRIu64 " channels in Buffer Mode (%u enabled). "
			       "Disable channels or use %" PRIu64 " MHz or less.",
			       devc->samplerate / SR_MHZ(1),
			       devc->info.plus ? "DL32 Plus" : "DL32",
			       budget / devc->samplerate, nen,
			       budget / nen / SR_MHZ(1));
			return SR_ERR_ARG;
		}
	}

	/* samples requested from the device, and the pre-trigger position */
	trigger_pos = devc->hw_trigger ? limit / 100 * ds_trigger_get_pos() : 0;
	if (devc->buffer_mode) {
		device_samples = limit;
	} else {
		/* whole groups; the extra samples are dropped */
		uint64_t groups = (limit + DL32_BLOCK_SAMPLES - 1) / DL32_BLOCK_SAMPLES;
		device_samples = groups * DL32_BLOCK_SAMPLES;
	}
	device_samples = MIN(device_samples, DL32_MAX_DEVICE_SAMPLES);
	devc->pre_trigger = trigger_pos;
	devc->trig_sent = FALSE;

	devc->group_bytes = (size_t)devc->num_channels * DL32_BLOCK_BYTES;
	devc->group_fill = 0;
	devc->group_buf = g_try_malloc(devc->group_bytes);
	devc->cross_buf = g_try_malloc(MAX((size_t)DL32_SEND_CHUNK / 64,
		(size_t)DL32_BLOCK_BYTES / 8) * 8 * DL32_TRIGGER_POSITIONS);
	devc->pending = g_byte_array_new();
	devc->capture = dl32_capture_new(devc->rle_active);
	if (!devc->group_buf || !devc->cross_buf) {
		free_acquisition(devc);
		ds_set_last_error(SR_ERR_MALLOC);
		return SR_ERR_MALLOC;
	}
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->aborted = FALSE;
	devc->data_started = FALSE;

	dl32_build_params(params, devc->buffer_mode, devc->rle_active,
			  (int)floor(devc->vth * 10.0 + 0.5), hz_index,
			  device_samples, trigger_pos);
	dl32_build_trigger(trig, codes, devc->num_channels, devc->ch_offset);
	/*
	 * Without a trigger, buffer mode must be told to capture at once
	 * ("instant" in ATK-Logic); otherwise the device arms and waits.
	 */
	if (devc->buffer_mode && !devc->hw_trigger)
		trig[DL32_TRIGGER_INSTANT] = 1;
	{
		char ph[64], th[80];
		sr_info("Starting %s mode%s: %" PRIu64 " Hz, %" PRIu64 " device "
			"samples, %" PRIu64 " to show, %s trigger, %d channels shown, "
			"threshold %.1f V.", devc->buffer_mode ? "buffer" : "stream",
			devc->rle_active ? " (RLE)" : "", devc->samplerate, device_samples, limit,
			devc->hw_trigger ? "hardware" : "no", devc->en_count, devc->vth);
		sr_dbg("Params: %s", dl32_hex(ph, sizeof(ph), params, sizeof(params)));
		sr_dbg("Trigger: %s", dl32_hex(th, sizeof(th), trig, sizeof(trig)));
	}

	/* wake the FPGA, then send parameters and the start (trigger) command */
	dl32_usb_drain(devc->devhdl);
	if (dl32_send_raw(devc->devhdl, DL32_RAW_CMD_RESET, &one, 1) != SR_OK) {
		free_acquisition(devc);
		return SR_ERR;
	}
	reply = g_malloc(DL32_READ_LEN);
	dl32_usb_read(devc->devhdl, reply, DL32_READ_LEN, 200);
	g_free(reply);
	dl32_usb_drain(devc->devhdl);

	/* queue reads before starting so no data is lost at high rates */
	xfer_size = devc->group_bytes * DL32_GROUPS_PER_XFER;
	devc->transfers = g_malloc0(sizeof(*devc->transfers) * DL32_NUM_TRANSFERS);
	devc->num_transfers = DL32_NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	for (i = 0; i < DL32_NUM_TRANSFERS; i++) {
		struct libusb_transfer *t = libusb_alloc_transfer(0);
		uint8_t *buf = g_try_malloc(xfer_size);

		if (!t || !buf) {
			g_free(buf);
			if (t)
				libusb_free_transfer(t);
			ret = LIBUSB_ERROR_NO_MEM;
		} else {
			libusb_fill_bulk_transfer(t, devc->devhdl, DL32_EP_IN, buf,
						  (int)xfer_size, receive_transfer,
						  devc, DL32_USB_TIMEOUT_MS);
			ret = libusb_submit_transfer(t);
			if (ret != 0) {
				g_free(buf);
				libusb_free_transfer(t);
			}
		}
		if (ret != 0) {
			sr_err("Failed to submit transfer: %s.", libusb_error_name(ret));
			devc->aborted = TRUE;
			for (i = 0; i < devc->num_transfers; i++)
				if (devc->transfers[i])
					libusb_cancel_transfer(devc->transfers[i]);
			while (devc->submitted_transfers) {
				struct timeval tv = { 0, 100000 };
				libusb_handle_events_timeout_completed(
					devc->sr_ctx->libusb_ctx, &tv, NULL);
			}
			free_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = t;
		devc->submitted_transfers++;
	}

	devc->running = TRUE;
	std_session_send_df_header(sdi, LOG_PREFIX);
	sr_session_source_add(-1, 0, 1, receive_data, sdi);

	if (dl32_send_framed(devc->devhdl, DL32_CMD_PARAMS, params,
			     sizeof(params)) != SR_OK) {
		sr_err("Failed to send capture parameters.");
		abort_acquisition(devc);
		return SR_OK;	/* receive_data ends the acquisition */
	}
	/* ATK-Logic waits between the two commands so the FPGA can settle */
	g_usleep(30 * 1000);
	if (dl32_send_framed(devc->devhdl, DL32_CMD_TRIGGER, trig,
			     sizeof(trig)) != SR_OK) {
		sr_err("Failed to send the start command.");
		abort_acquisition(devc);
	}
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct dl32_context *devc;

	(void)cb_data;
	if (!sdi || !(devc = sdi->priv))
		return SR_ERR;
	/* receive_data finishes once all transfers are back */
	if (devc->running && !devc->aborted)
		abort_acquisition(devc);
	return SR_OK;
}

SR_PRIV struct sr_dev_driver alientek_dl32_driver_info = {
	.name = "alientek-dl32",
	.longname = "ALIENTEK DL32 / DL32 Plus",
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
