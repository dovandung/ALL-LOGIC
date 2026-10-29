/*
 * Cypress FX3 logic analyzer driver for ALL LOGIC (libsigrok4DSL).
 *
 * Copyright (C) 2021 Ashwin Nair <ashwin.nair@infineon.com>
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
 * Logic analyzer on a Cypress (Infineon) FX3: the FX3's GPIF samples its
 * inputs (PIB clock 400 MHz / integer factor) and streams them over USB 3
 * bulk endpoint 0x82. Two firmwares, chosen with the channel mode:
 *   8 channels,  1 byte per sample,  up to 200 MHz (cypress-fx3-8.fw)
 *   16 channels, 2 bytes per sample, up to 100 MHz (cypress-fx3-16.fw)
 *
 * Port of the libsigrok cypress-fx3-8 and cypress-fx3 (16 bit) drivers.
 * The firmware (DSView/res) is loaded into the FX3's RAM through its USB
 * boot loader with ezusb_upload_firmware_fx3() (../common/ezusb.c, as in
 * libsigrok); both then enumerate as 04B4:1234 with the strings
 * "sigrok" / "cypress-fx3", so which one runs cannot be read from the
 * device: the driver records the firmware it loaded (in the user's config
 * directory, so it survives a restart of ALL LOGIC). Neither firmware can
 * be told to go back to the boot loader, so a different channel mode takes
 * effect when the FX3 is plugged in again. Commands (vendor requests):
 *   0xB0 GET_FW_VERSION  -> { major, minor }, major must be 1
 *   0xB2 GET_REVID       -> 1 byte
 *   0xB1 START           <- u16 sampling factor = 400 MHz / sample rate
 *
 * The trigger is applied on the PC (like the libsigrok driver's software
 * trigger): pre- and post-trigger samples are collected in RAM by the
 * shared capture core and sent when the capture is complete.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "../common/capture_core.h"

#undef LOG_PREFIX
#define LOG_PREFIX "cypress-fx3: "

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define FX3_EP_IN		(2 | LIBUSB_ENDPOINT_IN)

#define CMD_GET_FW_VERSION	0xb0
#define CMD_START		0xb1
#define CMD_GET_REVID_VERSION	0xb2
#define FX3_REQUIRED_VERSION_MAJOR 1
#define CMD_TIMEOUT_MS		100

#define FX3_PIB_CLOCK		SR_MHZ(400)
#define NUM_CHANNELS		16	/* the most any mode has */
/* the libsigrok driver ends the capture at the second empty/failed
 * transfer in a row: the FX3 has stopped */
#define MAX_EMPTY_TRANSFERS	1
#define MAX_RENUM_DELAY_MS	4000
#define DEFAULT_SAMPLES		SR_Mn(1)

#if GLIB_SIZEOF_VOID_P >= 8
#define MAX_SAMPLES		4000000000ULL
#else
#define MAX_SAMPLES		400000000ULL
#endif
/* samples per LA_CROSS_DATA packet sent to the session (multiple of 64) */
#define SEND_CHUNK		(1024 * 1024)

#define FX3_VID			0x04b4
#define FX3_PID_FW		0x1234	/* running firmware (either mode) */
#define FX3_PID_BOOT		0x00f3	/* FX3 ROM boot loader */

static const uint64_t samplerates_8[] = {
	SR_KHZ(500), SR_MHZ(1), SR_MHZ(5), SR_MHZ(10), SR_MHZ(25),
	SR_MHZ(50), SR_MHZ(100), SR_MHZ(200),
};

static const uint64_t samplerates_16[] = {
	SR_KHZ(500), SR_MHZ(1), SR_MHZ(5), SR_MHZ(10), SR_MHZ(25),
	SR_MHZ(50), SR_MHZ(100),
};

/* One per firmware; transfer sizing as in the respective libsigrok driver. */
struct fx3_mode {
	const char *firmware;
	int channels;
	unsigned int unit;		/* bytes per sample */
	const uint64_t *rates;
	unsigned int num_rates;
	unsigned int buf_ms;		/* transfer size: buf_ms * rate / 1000 bytes */
	unsigned int max_transfers;
	unsigned int headroom_div;	/* timeout = t + t / headroom_div */
};

enum { FX3_MODE_8CH = 0, FX3_MODE_16CH = 1, FX3_NUM_MODES };

static const struct fx3_mode modes[FX3_NUM_MODES] = {
	{ "cypress-fx3-8.fw",  8, 1, samplerates_8,  G_N_ELEMENTS(samplerates_8),
	  10, 64, 2 },
	{ "cypress-fx3-16.fw", 16, 2, samplerates_16, G_N_ELEMENTS(samplerates_16),
	  20, 32, 4 },
};

static const struct sr_list_item channel_mode_list[] = {
	{ FX3_MODE_8CH, "Use Channels 0~7 (Max 200MHz)" },
	{ FX3_MODE_16CH, "Use Channels 0~15 (Max 100MHz)" },
	{ -1, NULL },
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
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

struct fx3_context {
	const struct sr_dev_inst *sdi;
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;

	uint64_t samplerate;
	uint64_t limit_samples;
	int max_height;

	int ch_mode;			/* chosen mode: firmware to load next time */
	int fw_mode;			/* mode of the firmware running now */
	gboolean fw_mode_known;		/* FALSE: running firmware assumed */

	/*
	 * USB completions run wherever libusb events are handled (the session
	 * thread, or the host's hotplug thread), so the capture and transfer
	 * bookkeeping is under `mutex`.
	 */
	GMutex mutex;
	struct hl_capture cap;
	gboolean running;		/* acquisition started, END not sent yet */
	gboolean aborted;		/* stop requested or capture finished */
	gboolean user_stop;		/* Stop pressed */
	gboolean usb_error;		/* device gone / transfers failed */
	int empty_transfer_count;
	int submitted_transfers;
	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	uint8_t en_bits[NUM_CHANNELS];
	int en_count;
	uint8_t *cross_buf;
};

SR_PRIV struct sr_dev_driver cypress_fx3_driver_info;
static struct sr_dev_driver *di = &cypress_fx3_driver_info;

/* ------------------------------------------------------------ helpers */

static char *fw_path(int mode)
{
	if (DS_RES_PATH[0] == '\0')
		return NULL;
	return g_build_filename(DS_RES_PATH, modes[mode].firmware, NULL);
}

static const struct fx3_mode *cur_mode(const struct fx3_context *devc)
{
	return &modes[devc->fw_mode];
}

/* ------------------------------------------------------------ settings */

/*
 * <user config dir>/AllLogic/cypress-fx3.ini:
 *   channel_mode     the mode chosen in the device options (firmware to load)
 *   loaded_firmware  the mode of the firmware loaded last (-1: none yet)
 * Both firmwares look the same on USB, so the second one is how a running
 * FX3's mode is known, also after ALL LOGIC is restarted.
 */
static char *settings_path(void)
{
	return g_build_filename(g_get_user_config_dir(), "AllLogic",
				"cypress-fx3.ini", NULL);
}

static int settings_get(const char *key, int def)
{
	char *path = settings_path();
	GKeyFile *kf = g_key_file_new();
	GError *err = NULL;
	int v = def;

	if (g_key_file_load_from_file(kf, path, G_KEY_FILE_NONE, NULL)) {
		v = g_key_file_get_integer(kf, "cypress-fx3", key, &err);
		if (err) {
			v = def;
			g_clear_error(&err);
		}
	}
	g_key_file_free(kf);
	g_free(path);
	return v;
}

static void settings_set(const char *key, int value)
{
	char *path = settings_path();
	char *dir = g_path_get_dirname(path);
	GKeyFile *kf = g_key_file_new();
	GError *err = NULL;

	g_key_file_load_from_file(kf, path, G_KEY_FILE_KEEP_COMMENTS, NULL);
	g_key_file_set_integer(kf, "cypress-fx3", key, value);
	g_mkdir_with_parents(dir, 0755);
	if (!g_key_file_save_to_file(kf, path, &err)) {
		sr_warn("Cannot save %s: %s", path, err ? err->message : "?");
		g_clear_error(&err);
	}
	g_key_file_free(kf);
	g_free(dir);
	g_free(path);
}

static int valid_mode(int m)
{
	return m >= 0 && m < FX3_NUM_MODES;
}

static int chosen_mode(void)
{
	int m = settings_get("channel_mode", FX3_MODE_8CH);
	return valid_mode(m) ? m : FX3_MODE_8CH;
}

static int loaded_mode(void)
{
	int m = settings_get("loaded_firmware", -1);
	return valid_mode(m) ? m : -1;
}

/* nearest offered rate of the running firmware (the rate lists differ) */
static void clamp_samplerate(struct fx3_context *devc)
{
	const struct fx3_mode *m = cur_mode(devc);
	uint64_t best = m->rates[0], d, bd = UINT64_MAX;
	unsigned int i;

	for (i = 0; i < m->num_rates; i++) {
		d = m->rates[i] > devc->samplerate ? m->rates[i] - devc->samplerate
						   : devc->samplerate - m->rates[i];
		if (d < bd) {
			bd = d;
			best = m->rates[i];
		}
	}
	devc->samplerate = best;
}

/* channels beyond the running firmware's are off */
static void apply_channel_count(struct sr_dev_inst *sdi)
{
	struct fx3_context *devc = sdi->priv;
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type == SR_CHANNEL_LOGIC)
			ch->enabled = ch->index < cur_mode(devc)->channels;
	}
}

/* The firmware runs if the device says "sigrok" / "cypress-fx3". */
static gboolean has_running_fw(libusb_device *dev)
{
	libusb_device_handle *hdl = NULL;
	struct libusb_device_descriptor des;
	unsigned char man[64], prod[64];
	gboolean ok;

	if (!dev || libusb_get_device_descriptor(dev, &des) != 0)
		return FALSE;
	if (des.idVendor != FX3_VID || des.idProduct != FX3_PID_FW)
		return FALSE;
	if (libusb_open(dev, &hdl) != 0)
		return FALSE;
	man[0] = prod[0] = 0;
	if (des.iManufacturer)
		libusb_get_string_descriptor_ascii(hdl, des.iManufacturer,
						   man, sizeof(man));
	if (des.iProduct)
		libusb_get_string_descriptor_ascii(hdl, des.iProduct,
						   prod, sizeof(prod));
	libusb_close(hdl);
	ok = !strcmp((char *)man, "sigrok") && !strcmp((char *)prod, "cypress-fx3");
	return ok;
}

static gboolean is_fx3(const struct libusb_device_descriptor *des)
{
	return des->idVendor == FX3_VID &&
	       (des->idProduct == FX3_PID_FW || des->idProduct == FX3_PID_BOOT);
}

/* After the firmware upload the FX3 leaves the bus and comes back (as a
 * SuperSpeed device when it can). Pump libusb events while waiting, or
 * Windows never sees the new device. */
static libusb_device *wait_for_firmware(struct sr_context *sr_ctx)
{
	gint64 deadline = g_get_monotonic_time() + MAX_RENUM_DELAY_MS * 1000;
	libusb_device *found = NULL;

	g_usleep(300 * 1000);
	while (!found && g_get_monotonic_time() < deadline) {
		libusb_device **devlist;
		struct timeval tv = { 0, 100000 };
		int i;

		libusb_handle_events_timeout_completed(sr_ctx->libusb_ctx, &tv, NULL);
		if (libusb_get_device_list(sr_ctx->libusb_ctx, &devlist) < 0)
			continue;
		for (i = 0; devlist[i]; i++) {
			if (sr_usb_device_is_exists(devlist[i]))
				continue;
			if (has_running_fw(devlist[i])) {
				found = libusb_ref_device(devlist[i]);
				break;
			}
		}
		libusb_free_device_list(devlist, 1);
	}
	return found;
}

static void refresh_enabled(struct fx3_context *devc, const struct sr_dev_inst *sdi)
{
	int nch = cur_mode(devc)->channels;
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled ||
		    ch->index >= nch)
			continue;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0)
		for (n = 0; n < nch; n++)
			devc->en_bits[n] = (uint8_t)n;
	devc->en_count = n;
}

/* DSView simple trigger (stage TriggerStages): per channel 'X' don't care,
 * '0'/'1' level, 'R'/'F' edge, 'C' either edge; disabled channels ignored. */
static void map_trigger(const struct sr_dev_inst *sdi, struct hl_trigger_spec *t)
{
	const struct fx3_context *devc = sdi->priv;
	GSList *l;

	memset(t, 0, sizeof(*t));
	if (!trigger || !trigger->trigger_en)
		return;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		uint16_t m;

		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled ||
		    ch->index >= cur_mode(devc)->channels)
			continue;
		m = (uint16_t)(1u << ch->index);
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

/* ------------------------------------------------------------ commands */

static int command_get_fw_version(libusb_device_handle *hdl, uint8_t *major,
				  uint8_t *minor)
{
	unsigned char vi[2];
	int ret;

	ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_FW_VERSION, 0, 0, vi, sizeof(vi),
		CMD_TIMEOUT_MS);
	if (ret != (int)sizeof(vi)) {
		sr_err("Unable to get the firmware version: %s.",
		       ret < 0 ? libusb_error_name(ret) : "short answer");
		return SR_ERR;
	}
	*major = vi[0];
	*minor = vi[1];
	return SR_OK;
}

static int command_get_revid(libusb_device_handle *hdl, uint8_t *revid)
{
	int ret;

	ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_REVID_VERSION, 0, 0, revid, 1,
		CMD_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}
	return SR_OK;
}

static int command_start_acquisition(struct fx3_context *devc)
{
	unsigned char cmd[2];
	uint64_t factor;
	int ret;

	if (devc->samplerate == 0 ||
	    devc->samplerate > cur_mode(devc)->rates[cur_mode(devc)->num_rates - 1]) {
		sr_err("Unable to sample at %" PRIu64 " Hz.", devc->samplerate);
		return SR_ERR;
	}
	factor = FX3_PIB_CLOCK / devc->samplerate;
	if (factor > 0xffff)
		factor = 0xffff;
	cmd[0] = (unsigned char)(factor & 0xff);	/* u16, little endian */
	cmd[1] = (unsigned char)(factor >> 8);

	ret = libusb_control_transfer(devc->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, CMD_START, 0, 0, cmd, sizeof(cmd),
		CMD_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("Unable to send the start command: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}
	sr_info("START: %d channels, %" PRIu64 " Hz (sampling factor %u).",
		cur_mode(devc)->channels, devc->samplerate, (unsigned int)factor);
	return SR_OK;
}

/* ------------------------------------------------------------ transfers */

static unsigned int bytes_per_ms(const struct fx3_context *devc)
{
	unsigned int b = (unsigned int)(devc->samplerate / 1000);
	return b ? b : 1;
}

/*
 * As in the libsigrok drivers (where "bytes per ms" is rate / 1000 also
 * for 16-bit samples): each transfer is buf_ms * rate / 1000 bytes, a
 * multiple of 1024, i.e. 10 ms of data in both modes, and together they
 * hold 500 * rate / 1000 bytes, at most max_transfers of them.
 */
static size_t get_buffer_size(const struct fx3_context *devc)
{
	size_t s = cur_mode(devc)->buf_ms * (size_t)bytes_per_ms(devc);
	return (s + 1023) & ~(size_t)1023;
}

static unsigned int get_number_of_transfers(const struct fx3_context *devc)
{
	unsigned int n = (unsigned int)(500 * (size_t)bytes_per_ms(devc) /
					get_buffer_size(devc));
	if (n < 2)
		n = 2;
	if (n > cur_mode(devc)->max_transfers)
		n = cur_mode(devc)->max_transfers;
	return n;
}

static unsigned int get_timeout(const struct fx3_context *devc)
{
	size_t total = get_buffer_size(devc) * get_number_of_transfers(devc);
	unsigned int t = (unsigned int)(total / bytes_per_ms(devc));
	return t + t / cur_mode(devc)->headroom_div;	/* 50 % / 25 % headroom */
}

/* Called with the mutex held. */
static void cancel_transfers(struct fx3_context *devc)
{
	unsigned int i;

	devc->aborted = TRUE;
	for (i = 0; i < devc->num_transfers; i++)
		if (devc->transfers && devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
}

/* Called with the mutex held. */
static void free_transfer(struct libusb_transfer *transfer)
{
	struct fx3_context *devc = transfer->user_data;
	unsigned int i;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	libusb_free_transfer(transfer);
	devc->submitted_transfers--;
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct fx3_context *devc = transfer->user_data;
	gboolean error = FALSE;
	int ret;

	g_mutex_lock(&devc->mutex);

	if (devc->aborted) {
		free_transfer(transfer);
		goto out;
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		sr_err("The device is gone.");
		devc->usb_error = TRUE;
		cancel_transfers(devc);
		free_transfer(transfer);
		goto out;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT:	/* we may have received some data */
		break;
	default:
		error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || error) {
		if (++devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			/* The FX3 gave up: end the acquisition with what we have. */
			sr_err("No data from the FX3 (transfer status %d), ending "
			       "the acquisition.", (int)transfer->status);
			devc->usb_error = TRUE;
			cancel_transfers(devc);
			free_transfer(transfer);
			goto out;
		}
	} else {
		devc->empty_transfer_count = 0;
		hl_feed_bytes(&devc->cap, transfer->buffer,
			      (size_t)transfer->actual_length);
		if (devc->cap.state == HL_DONE) {
			cancel_transfers(devc);
			free_transfer(transfer);
			goto out;
		}
	}

	ret = libusb_submit_transfer(transfer);
	if (ret != 0) {
		sr_err("Resubmitting a transfer failed: %s.", libusb_error_name(ret));
		devc->usb_error = TRUE;
		cancel_transfers(devc);
		free_transfer(transfer);
	}
out:
	g_mutex_unlock(&devc->mutex);
}

static int start_transfers(struct fx3_context *devc)
{
	unsigned int i, n = get_number_of_transfers(devc);
	size_t size = get_buffer_size(devc);
	unsigned int timeout = get_timeout(devc);
	int ret;

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * n);
	if (!devc->transfers)
		return SR_ERR_MALLOC;
	devc->num_transfers = n;
	devc->submitted_transfers = 0;
	devc->empty_transfer_count = 0;

	sr_info("%u transfers of %u bytes, timeout %u ms.", n, (unsigned int)size,
		timeout);
	g_mutex_lock(&devc->mutex);
	for (i = 0; i < n; i++) {
		struct libusb_transfer *t;
		unsigned char *buf = g_try_malloc(size);

		if (!buf || !(t = libusb_alloc_transfer(0))) {
			g_free(buf);
			sr_err("USB transfer allocation failed.");
			cancel_transfers(devc);
			g_mutex_unlock(&devc->mutex);
			return SR_ERR_MALLOC;
		}
		libusb_fill_bulk_transfer(t, devc->devhdl, FX3_EP_IN, buf,
					  (int)size, receive_transfer, devc, timeout);
		if ((ret = libusb_submit_transfer(t)) != 0) {
			sr_err("Failed to submit a transfer: %s.", libusb_error_name(ret));
			g_free(buf);
			libusb_free_transfer(t);
			cancel_transfers(devc);
			g_mutex_unlock(&devc->mutex);
			return SR_ERR;
		}
		devc->transfers[i] = t;
		devc->submitted_transfers++;
	}
	g_mutex_unlock(&devc->mutex);
	return SR_OK;
}

/* ------------------------------------------------------- send to session */

/* n samples from src (cur_mode(devc)->unit bytes each) */
static void send_cross(struct fx3_context *devc, const uint8_t *src, uint64_t n)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	while (n) {
		uint64_t k = n < SEND_CHUNK ? n : SEND_CHUNK;

		hl_to_cross(src, k, devc->en_bits, devc->en_count, devc->cross_buf,
			    cur_mode(devc)->unit);
		memset(&packet, 0, sizeof(packet));
		memset(&logic, 0, sizeof(logic));
		packet.type = SR_DF_LOGIC;
		packet.status = SR_PKT_OK;
		packet.payload = &logic;
		logic.length = k / 64 * 8 * (uint64_t)devc->en_count;
		logic.format = LA_CROSS_DATA;
		logic.data = devc->cross_buf;
		ds_data_forward(devc->sdi, &packet);
		src += k * cur_mode(devc)->unit;
		n -= k;
	}
}

static void send_trigger(struct fx3_context *devc, uint64_t index)
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

/* Send the collected samples (whole groups of 64), the trigger marker
 * between the pre- and post-trigger part. */
static void send_capture(struct fx3_context *devc, gboolean complete)
{
	struct hl_capture *c = &devc->cap;
	uint64_t total = c->buf_fill / 64 * 64;
	uint64_t pre = c->trigger_pos / 64 * 64;

	if (!total) {
		sr_info("No samples captured.");
		return;
	}
	if (!complete)
		sr_info("Acquisition ended early, sending %" PRIu64 " samples.", total);
	if (pre > total)
		pre = total;
	send_cross(devc, c->buf, pre);
	if (c->have_trigger)
		send_trigger(devc, c->trigger_pos);
	send_cross(devc, c->buf + pre * c->unit, total - pre);
}

/* All transfers are back: send the data and SR_DF_END. */
static void finish_acquisition(struct fx3_context *devc)
{
	struct sr_datafeed_packet packet;
	int state;

	g_mutex_lock(&devc->mutex);
	state = devc->cap.state;
	devc->cap.state = HL_IDLE;
	g_mutex_unlock(&devc->mutex);

	/* complete, or stopped / failed after the trigger: send what we have */
	if (state == HL_DONE || state == HL_CAPTURING)
		send_capture(devc, state == HL_DONE);

	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);
	sr_info("SR_DF_END.");

	devc->running = FALSE;
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	hl_free_buffers(&devc->cap);
	g_free(devc->cross_buf);
	devc->cross_buf = NULL;
}

static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct fx3_context *devc;
	struct timeval tv = { 0, 0 };
	int pending;

	(void)fd;
	(void)revents;
	if (!sdi || !(devc = sdi->priv))
		return TRUE;
	if (!devc->running) {
		sr_session_source_remove(-1);
		return TRUE;
	}

	libusb_handle_events_timeout_completed(devc->sr_ctx->libusb_ctx, &tv, NULL);

	g_mutex_lock(&devc->mutex);
	if (devc->user_stop && !devc->aborted)
		cancel_transfers(devc);
	pending = devc->submitted_transfers;
	g_mutex_unlock(&devc->mutex);

	if (pending == 0) {
		finish_acquisition(devc);
		sr_session_source_remove(-1);
	}
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

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *devices = NULL;
	static gint64 last_upload;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		struct fx3_context *devc;
		struct sr_dev_inst *sdi;
		struct sr_usb_dev_inst *usb_info;
		libusb_device *dev = devlist[i], *live = NULL;
		char name[16];
		int ch, fw_mode;
		gboolean fw_known;

		if (libusb_get_device_descriptor(dev, &des) != 0 || !is_fx3(&des))
			continue;
		if (sr_usb_device_is_exists(dev))
			continue;

		fw_mode = chosen_mode();
		fw_known = TRUE;
		if (has_running_fw(dev)) {
			/* both firmwares look alike: use what we loaded */
			int lm = loaded_mode();
			if (lm >= 0) {
				fw_mode = lm;
			} else {
				fw_known = FALSE;
				sr_warn("An FX3 already runs a firmware not loaded by "
					"ALL LOGIC; assuming the %d channel one. Plug it "
					"in again to load the chosen firmware.",
					modes[fw_mode].channels);
			}
		} else {
			char *path;
			int ret;

			/* an FX3 that has not come back from an upload yet may
			 * still show up as the boot loader: do not upload again */
			if (last_upload && g_get_monotonic_time() - last_upload <
					   5 * G_TIME_SPAN_SECOND) {
				sr_info("Waiting for an FX3 to re-enumerate, not uploading again.");
				continue;
			}
			path = fw_path(fw_mode);
			if (!path) {
				sr_err("Firmware directory not set.");
				continue;
			}
			sr_info("Loading the %d channel firmware (%s).",
				modes[fw_mode].channels, modes[fw_mode].firmware);
			ret = ezusb_upload_firmware_fx3(dev, USB_CONFIGURATION, path);
			g_free(path);
			if (ret != SR_OK) {
				sr_err("Firmware upload to %04x:%04x failed.",
				       des.idVendor, des.idProduct);
				continue;
			}
			settings_set("loaded_firmware", fw_mode);
			last_upload = g_get_monotonic_time();
			live = wait_for_firmware(drvc->sr_ctx);
			if (!live) {
				sr_err("The FX3 did not come back after the firmware upload.");
				continue;
			}
			/* back with its firmware: a boot loader seen from now on is
			 * a new plug-in, not this upload still in progress */
			last_upload = 0;
			dev = live;
		}

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc) {
			if (live)
				libusb_unref_device(live);
			break;
		}
		g_mutex_init(&devc->mutex);
		devc->usb_dev = live ? live : libusb_ref_device(dev);
		devc->sr_ctx = drvc->sr_ctx;
		devc->samplerate = SR_MHZ(1);
		devc->limit_samples = DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->cap.state = HL_IDLE;
		devc->ch_mode = chosen_mode();
		devc->fw_mode = fw_mode;
		devc->fw_mode_known = fw_known;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE, "Cypress", "FX3",
				      "cypress-fx3");
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
		usb_info = sr_usb_dev_inst_new(libusb_get_bus_number(devc->usb_dev),
					       libusb_get_device_address(devc->usb_dev));
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		}
		for (ch = 0; ch < NUM_CHANNELS; ch++) {
			snprintf(name, sizeof(name), "%d", ch);
			sdi->channels = g_slist_append(sdi->channels,
				sr_channel_new(ch, SR_CHANNEL_LOGIC, TRUE, name));
		}
		apply_channel_count(sdi);
		sr_info("Found a Cypress FX3 with the %d channel firmware%s on %d.%d.",
			modes[fw_mode].channels, fw_known ? "" : " (assumed)",
			libusb_get_bus_number(devc->usb_dev),
			libusb_get_device_address(devc->usb_dev));
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);
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
	struct fx3_context *devc = sdi->priv;
	uint8_t major = 0, minor = 0, revid = 0;
	int ret;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;

	ret = libusb_open(devc->usb_dev, &devc->devhdl);
	if (ret != 0) {
		sr_err("Failed to open the device: %s.", libusb_error_name(ret));
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	libusb_set_auto_detach_kernel_driver(devc->devhdl, 1);
	ret = libusb_claim_interface(devc->devhdl, USB_INTERFACE);
	if (ret != 0) {
		sr_err("Unable to claim the USB interface: %s.", libusb_error_name(ret));
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	if (command_get_fw_version(devc->devhdl, &major, &minor) != SR_OK ||
	    command_get_revid(devc->devhdl, &revid) != SR_OK)
		goto fail;
	/* a new major version means an incompatible protocol */
	if (major != FX3_REQUIRED_VERSION_MAJOR) {
		sr_err("Expected firmware version %d.x, got %d.%d.",
		       FX3_REQUIRED_VERSION_MAJOR, major, minor);
		goto fail;
	}
	if (libusb_get_device_speed(devc->usb_dev) < LIBUSB_SPEED_SUPER)
		sr_warn("The FX3 is not on a USB 3 link: high sample rates will "
			"lose data.");

	devc->sdi = sdi;
	sdi->status = SR_ST_ACTIVE;
	clamp_samplerate(devc);
	sr_info("Opened, firmware %d.%d (%d channels%s), REVID 0x%02x.", major,
		minor, cur_mode(devc)->channels,
		devc->fw_mode_known ? "" : ", assumed", revid);
	return SR_OK;

fail:
	libusb_release_interface(devc->devhdl, USB_INTERFACE);
	libusb_close(devc->devhdl);
	devc->devhdl = NULL;
	ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
	return SR_ERR;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct fx3_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	if (devc->devhdl) {
		libusb_release_interface(devc->devhdl, USB_INTERFACE);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct fx3_context *devc;

	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		hl_free(&devc->cap);
		g_free(devc->cross_buf);
		g_free(devc->transfers);
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
	struct fx3_context *devc;

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
		*data = g_variant_new_int16(NUM_CHANNELS);
		break;
	case SR_CONF_VLD_CH_NUM:
		/* what the running firmware samples */
		*data = g_variant_new_int16(cur_mode(devc)->channels);
		break;
	case SR_CONF_CHANNEL_MODE:
		/* the chosen mode (loaded at the next plug-in if different) */
		*data = g_variant_new_int16(devc->ch_mode);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_CAPTURE_RATIO:
		/* pre-trigger samples are kept in host memory: free position */
		*data = g_variant_new_uint64(trigger ? ds_trigger_get_pos() : 0);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(LO_OP_STREAM);
		break;
	case SR_CONF_LOOP_MODE:
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
	struct fx3_context *devc;
	uint64_t v;
	unsigned int i;

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
		v = g_variant_get_uint64(data);
		for (i = 0; i < cur_mode(devc)->num_rates; i++)
			if (cur_mode(devc)->rates[i] == v)
				break;
		if (i == cur_mode(devc)->num_rates)
			return SR_ERR_ARG;
		devc->samplerate = v;
		break;
	case SR_CONF_CHANNEL_MODE: {
		int nv = g_variant_get_int16(data);

		if (!valid_mode(nv))
			return SR_ERR_ARG;
		devc->ch_mode = nv;
		settings_set("channel_mode", nv);
		if (nv != devc->fw_mode || !devc->fw_mode_known) {
			/*
			 * The running firmware cannot be swapped without
			 * re-plugging (it has no command to go back to the boot
			 * loader). The device options dialog shows "Please
			 * reconnect the device!" on this error; until then the
			 * captures use the firmware that runs.
			 */
			sr_info("Channel mode %d channels chosen; plug the FX3 in "
				"again to load %s.", modes[nv].channels,
				modes[nv].firmware);
			ds_set_last_error(SR_ERR_DEVICE_FIRMWARE_VERSION_LOW);
		} else {
			ds_set_last_error(SR_OK);
		}
		break;
	}
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
		for (i = 0; i < G_N_ELEMENTS(maxHeights); i++) {
			if (s && strcmp(s, maxHeights[i]) == 0) {
				devc->max_height = (int)i;
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
	GVariantBuilder gvb;
	GVariant *gvar;

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
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		{
			const struct fx3_mode *m = (sdi && sdi->priv) ?
				cur_mode(sdi->priv) : &modes[FX3_MODE_8CH];
			gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
				m->rates, m->num_rates, sizeof(uint64_t));
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
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_uint64((uint64_t)(uintptr_t)&channel_mode_list);
		break;
	default:
		return SR_ERR_ARG;
	}
	return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct fx3_context *devc = sdi->priv;
	struct hl_trigger_spec spec;
	uint64_t total, pre, post;
	int ret;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE || !devc->devhdl) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	clamp_samplerate(devc);
	refresh_enabled(devc, sdi);
	map_trigger(sdi, &spec);
	if (devc->ch_mode != devc->fw_mode)
		sr_warn("The %d channel firmware runs (the %d channel one is chosen; "
			"plug the FX3 in again to load it).",
			cur_mode(devc)->channels, modes[devc->ch_mode].channels);

	/* LA_CROSS_DATA carries whole groups of 64 samples */
	total = devc->limit_samples / 64 * 64;
	if (total < 64)
		total = 64;
	ret = hl_build_trigger(&devc->cap, &spec);
	if (ret < 0)
		return SR_ERR_MALLOC;
	pre = ret == 1 ? total * ds_trigger_get_pos() / 100 : 0;
	post = total - pre;
	if (!post) {
		post = 1;
		pre = total - 1;
	}

	devc->cross_buf = g_try_malloc(SEND_CHUNK / 64 * 8 * NUM_CHANNELS);
	if (!devc->cross_buf || hl_alloc(&devc->cap, pre, post, cur_mode(devc)->unit) != 0) {
		sr_err("Not enough memory for %" PRIu64 " samples.", total);
		g_free(devc->cross_buf);
		devc->cross_buf = NULL;
		ds_set_last_error(SR_ERR_MALLOC);
		return SR_ERR_MALLOC;
	}
	hl_reset(&devc->cap);

	devc->aborted = FALSE;
	devc->user_stop = FALSE;
	devc->usb_error = FALSE;

	if ((ret = start_transfers(devc)) != SR_OK) {
		/* wait for the ones already submitted to come back */
		while (devc->submitted_transfers > 0) {
			struct timeval tv = { 0, 10000 };
			libusb_handle_events_timeout_completed(devc->sr_ctx->libusb_ctx,
							       &tv, NULL);
		}
		g_free(devc->transfers);
		devc->transfers = NULL;
		hl_free_buffers(&devc->cap);
		g_free(devc->cross_buf);
		devc->cross_buf = NULL;
		return ret;
	}

	devc->running = TRUE;
	std_session_send_df_header(sdi, LOG_PREFIX);
	/* polled: the USB completions are handled in receive_data() */
	sr_session_source_add(-1, 0, 1, receive_data, sdi);

	if (command_start_acquisition(devc) != SR_OK) {
		g_mutex_lock(&devc->mutex);
		devc->usb_error = TRUE;
		cancel_transfers(devc);
		g_mutex_unlock(&devc->mutex);
		return SR_OK;	/* receive_data ends the acquisition */
	}
	sr_info("Waiting for %s, then %" PRIu64 " + %" PRIu64 " samples.",
		devc->cap.have_trigger ? "the trigger" : "data", pre, post);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct fx3_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	/* receive_data cancels the transfers and ends the acquisition */
	devc->user_stop = TRUE;
	return SR_OK;
}

SR_PRIV struct sr_dev_driver cypress_fx3_driver_info = {
	.name = "cypress-fx3",
	.longname = "Cypress FX3 logic analyzer (cypress-fx3-8)",
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
