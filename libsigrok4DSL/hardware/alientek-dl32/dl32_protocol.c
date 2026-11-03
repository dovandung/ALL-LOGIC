/*
 * This file is part of the libsigrok project.
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
 * ALIENTEK DL32 / DL32 Plus for ALL LOGIC (libsigrok4DSL): the protocol
 * helpers of the libsigrok alientek-dl32 driver (dovandung/libsigrok,
 * master, 96aad41), with only the host glue changed: logging, hex dumps
 * and the C library includes. The acquisition is in dl32.c.
 */
#include <inttypes.h>
#include <string.h>
#include "dl32.h"

/*
 * Up to 100 MHz the device streams; faster rates use buffer mode (capture
 * into on-board memory, then upload).
 */
SR_PRIV const uint64_t dl32_samplerates[] = {
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(4), SR_MHZ(5), SR_MHZ(10),
	SR_MHZ(20), SR_MHZ(25), SR_MHZ(40), SR_MHZ(50), SR_MHZ(100),
	SR_MHZ(200), SR_MHZ(250), SR_MHZ(400), SR_MHZ(500), SR_MHZ(800),
	SR_GHZ(1),
};
SR_PRIV const unsigned int dl32_num_samplerates = ARRAY_SIZE(dl32_samplerates);

/* Sample rate -> index used by the 0x11 parameter command. */
static const struct {
	uint64_t rate;
	int index;
} hz_index_table[] = {
	{ SR_MHZ(1), 1 }, { SR_MHZ(2), 2 }, { SR_MHZ(4), 3 }, { SR_MHZ(5), 4 },
	{ SR_MHZ(10), 5 }, { SR_MHZ(20), 6 }, { SR_MHZ(25), 7 },
	{ SR_MHZ(40), 8 }, { SR_MHZ(50), 9 }, { SR_MHZ(100), 10 },
	{ SR_MHZ(200), 11 }, { SR_MHZ(250), 12 }, { SR_MHZ(500), 13 },
	{ SR_GHZ(1), 14 }, { SR_MHZ(400), 15 }, { SR_MHZ(800), 16 },
};

/* Reflected CRC32 (0xEDB88320), init 0, final inversion. */
SR_PRIV uint32_t dl32_crc32(const uint8_t *buf, size_t len)
{
	static uint32_t table[256];
	static gboolean table_ready;
	uint32_t crc, c;
	size_t i;
	int k;

	if (!table_ready) {
		for (i = 0; i < 256; i++) {
			c = i;
			for (k = 0; k < 8; k++)
				c = (c & 1) ? (0xedb88320 ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		table_ready = TRUE;
	}

	crc = 0;
	for (i = 0; i < len; i++)
		crc = table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);

	return ~crc;
}

SR_PRIV void dl32_build_raw(uint8_t out[DL32_RAW_PACKET_LEN], uint8_t cmd,
	const uint8_t *payload, size_t len)
{
	memset(out, 0, DL32_RAW_PACKET_LEN);
	if (len > DL32_RAW_PACKET_LEN - 3)
		len = DL32_RAW_PACKET_LEN - 3;
	out[0] = 0x0a;
	out[1] = cmd;
	if (len)
		memcpy(&out[2], payload, len);
	out[2 + len] = 0x0b;
}

SR_PRIV size_t dl32_build_framed(uint8_t *out, uint8_t cmd,
	const uint8_t *data, size_t len)
{
	uint32_t crc;

	memset(out, 0, len + DL32_FRAME_OVERHEAD);
	out[8] = 0x0a;
	out[9] = cmd;
	out[10] = (len + 1) & 0xff;
	if (len)
		memcpy(&out[11], data, len);
	out[11 + len] = 0x0b;
	/* CRC covers cmd, length and data. */
	crc = dl32_crc32(&out[9], len + 2);
	WL32(&out[12 + len], crc);

	return len + DL32_FRAME_OVERHEAD;
}

SR_PRIV int dl32_samplerate_index(uint64_t samplerate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hz_index_table); i++) {
		if (hz_index_table[i].rate == samplerate)
			return hz_index_table[i].index;
	}

	return -1;
}

static void write_le40(uint8_t *p, uint64_t v)
{
	int i;

	for (i = 0; i < 5; i++)
		p[i] = (v >> (8 * i)) & 0xff;
}

SR_PRIV void dl32_build_params(uint8_t out[DL32_PARAMS_LEN],
	gboolean buffer_mode, gboolean rle, int threshold_tenths,
	int hz_index, uint64_t samples, uint64_t trigger_pos)
{
	out[0] = (buffer_mode ? 0x80 : 0) | (rle ? 0x40 : 0);
	out[1] = (threshold_tenths < 0 ? 0x80 : 0) |
		(ABS(threshold_tenths) & 0x7f);
	out[2] = hz_index;
	write_le40(&out[3], samples);
	write_le40(&out[8], trigger_pos);
}

/*
 * Simple trigger command: one nibble per channel position (even position in
 * the high nibble). DL32_TRIG_ENABLE marks a captured channel, the low three
 * bits hold its trigger condition (DL32_TRIG_NONE = don't care).
 * codes[] has one entry per channel; 0 means the channel is not captured.
 */
SR_PRIV void dl32_build_trigger(uint8_t out[DL32_TRIGGER_LEN],
	const uint8_t *codes, unsigned int num_channels, unsigned int ch_offset)
{
	unsigned int ch, pos;
	uint8_t nib;

	memset(out, 0, DL32_TRIGGER_LEN);
	for (ch = 0; ch < num_channels; ch++) {
		pos = ch_offset + ch;
		if (pos >= DL32_TRIGGER_POSITIONS)
			break;
		nib = codes[ch] & 0x0f;
		out[pos / 2] |= (pos & 1) ? nib : (uint8_t)(nib << 4);
	}
	/*
	 * out[16]: loop capture, out[17]: instant capture (no trigger wait).
	 * Both 0 = single capture that waits for the trigger.
	 */
}

/*
 * Translate the ALL LOGIC simple trigger (one character per channel: 'X'
 * don't care, '0' / '1' level, 'R' / 'F' edge, 'C' either edge; all
 * conditions must hold at once) into per-channel hardware codes. This is
 * exactly what the device can do, so no software fallback is needed.
 * conds may be NULL (no trigger). Returns TRUE when any condition is set.
 */
SR_PRIV gboolean dl32_trigger_codes(const char *conds,
	unsigned int num_channels, uint32_t enabled_mask, uint8_t *codes)
{
	unsigned int ch;
	gboolean any = FALSE;

	for (ch = 0; ch < num_channels; ch++) {
		codes[ch] = DL32_TRIG_NONE;
		if (!conds || !(enabled_mask & (1u << ch)))
			continue;
		switch (conds[ch]) {
		case '0': codes[ch] = DL32_TRIG_LOW; break;
		case '1': codes[ch] = DL32_TRIG_HIGH; break;
		case 'R': codes[ch] = DL32_TRIG_RISING; break;
		case 'F': codes[ch] = DL32_TRIG_FALLING; break;
		case 'C': codes[ch] = DL32_TRIG_EDGE; break;
		default: continue;
		}
		any = TRUE;
	}

	return any;
}

/* Reply to raw 0x81: [0A][81][01][mode][mcu_hi][mcu_lo][variant][?][plus]. */
SR_PRIV gboolean dl32_parse_hello(const uint8_t *buf, size_t len,
	struct dl32_info *info)
{
	if (len < 9 || buf[0] != 0x0a || buf[1] != DL32_RAW_CMD_HELLO ||
			buf[2] != 0x01)
		return FALSE;

	info->mode = buf[3];
	info->mcu_version = buf[4] * 10 + buf[5];
	info->variant = buf[6];
	info->plus = buf[8] == 1;

	return TRUE;
}

/* The device data reply carries a NUL terminated name such as "DL32". */
SR_PRIV void dl32_parse_name(const uint8_t *buf, size_t len,
	struct dl32_info *info)
{
	size_t i, j;

	for (i = 0; i + 1 < len; i++) {
		if (buf[i] != 'D' || buf[i + 1] != 'L')
			continue;
		for (j = 0; i + j < len && j < sizeof(info->name) - 1; j++) {
			if (buf[i + j] < 0x20 || buf[i + j] > 0x7e)
				break;
			info->name[j] = buf[i + j];
		}
		info->name[j] = '\0';
		return;
	}
}

/*
 * Convert one group (num_channels blocks of DL32_BLOCK_BYTES) into
 * DL32_BLOCK_SAMPLES samples of num_channels / 8 bytes each.
 *
 * spread[b] puts bit k of b into the lowest bit of byte k, so OR-ing
 * spread[b] << c for 8 channels gives 8 samples of 8 channels at once.
 */
/* spread[b] puts bit k of b into the lowest bit of byte k. */
static const uint64_t *spread_table(void)
{
	static uint64_t spread[256];
	static gboolean spread_ready;
	unsigned int i, k;

	if (!spread_ready) {
		for (i = 0; i < 256; i++) {
			spread[i] = 0;
			for (k = 0; k < 8; k++)
				spread[i] |= (uint64_t)((i >> k) & 1) << (8 * k);
		}
		spread_ready = TRUE;
	}

	return spread;
}

SR_PRIV void dl32_deinterleave(const uint8_t *group, unsigned int num_channels,
	uint8_t *out)
{
	const uint64_t *spread;
	unsigned int unitsize, i, c, j, k;
	uint64_t acc[4];

	spread = spread_table();
	unitsize = num_channels / 8;
	for (i = 0; i < DL32_BLOCK_BYTES; i++) {
		for (j = 0; j < unitsize; j++) {
			acc[j] = 0;
			for (c = 0; c < 8; c++) {
				acc[j] |= spread[group[(j * 8 + c) *
					DL32_BLOCK_BYTES + i]] << c;
			}
		}
		for (k = 0; k < 8; k++) {
			for (j = 0; j < unitsize; j++)
				out[k * unitsize + j] = (acc[j] >> (8 * k)) & 0xff;
		}
		out += 8 * unitsize;
	}
}

/*
 * --- Framed device data (status, buffer mode, RLE).
 *
 * Layout and meaning follow the DL16 code in ATK-Logic (pv/data/analysis.cpp,
 * pv/thread/thread_work.cpp): [0A][type][len16 LE][payload][00][0B], with
 * zero padding allowed between frames.
 */

static uint64_t read_le40(const uint8_t *p)
{
	uint64_t v;
	int i;

	v = 0;
	for (i = 4; i >= 0; i--)
		v = (v << 8) | p[i];

	return v;
}

/*
 * Try to read a frame at buf[0] with 'gap' bytes between header and payload.
 * Returns 1 = frame, 0 = not a frame, -1 = need more data.
 */
static int frame_at(const uint8_t *buf, size_t len, size_t gap,
	struct dl32_frame *frame)
{
	size_t plen, i;

	plen = buf[2] | (buf[3] << 8);
	if (4 + gap + plen + 2 > len)
		return -1;
	for (i = 0; i < gap; i++) {
		if (buf[4 + i] != 0x00)
			return 0;
	}
	if (buf[4 + gap + plen] != 0x00 || buf[5 + gap + plen] != 0x0b)
		return 0;
	frame->type = buf[1];
	frame->payload = &buf[4 + gap];
	frame->len = plen;
	frame->size = 4 + gap + plen + 2;

	return 1;
}

/*
 * Frames are [0A][type][len16 LE][gap][payload (len)][00][0B]. The DL16 has
 * no gap; the DL32 FPGA puts DL32_FRAME_GAP zero bytes after the header and
 * pads data frames to 2 KiB. Both layouts are accepted, DL32 first.
 */
SR_PRIV enum dl32_parse_result dl32_parse_frame(const uint8_t *buf,
	size_t len, size_t *skip, struct dl32_frame *frame)
{
	size_t i;
	int r32, r16;

	for (i = 0; i < len; i++) {
		if (buf[i] != 0x0a)
			continue;
		if (i + 1 >= len) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
		if (buf[i + 1] < DL32_FRAME_DATA || buf[i + 1] > DL32_FRAME_DONE)
			continue;
		if (i + 4 > len) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
		r32 = frame_at(buf + i, len - i, DL32_FRAME_GAP, frame);
		if (r32 == 1) {
			*skip = i;
			return DL32_PARSE_FRAME;
		}
		r16 = frame_at(buf + i, len - i, 0, frame);
		if (r16 == 1) {
			*skip = i;
			return DL32_PARSE_FRAME;
		}
		if (r32 < 0 || r16 < 0) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
	}
	*skip = len;

	return DL32_PARSE_NONE;
}

SR_PRIV struct dl32_capture *dl32_capture_new(gboolean rle)
{
	struct dl32_capture *cap;

	cap = g_malloc0(sizeof(*cap));
	cap->rle = rle;

	return cap;
}

SR_PRIV void dl32_capture_free(struct dl32_capture *cap)
{
	unsigned int i;

	if (!cap)
		return;
	for (i = 0; i < DL32_TRIGGER_POSITIONS; i++) {
		if (cap->chan[i])
			g_byte_array_free(cap->chan[i], TRUE);
	}
	g_free(cap);
}

/* Returns the frame type that was handled. */
SR_PRIV int dl32_capture_feed(struct dl32_capture *cap,
	const struct dl32_frame *frame)
{
	const uint8_t *p, *data;
	size_t n, rest, i, k;
	unsigned int id;

	p = frame->payload;
	n = frame->len;

	switch (frame->type) {
	case DL32_FRAME_DATA:
		/* [channel id][?][samples...], 8 samples per byte, LSB first. */
		if (n < 2)
			break;
		id = p[0];
		if (id >= DL32_TRIGGER_POSITIONS) {
			sr_dbg("Data for unknown channel id %u.", id);
			break;
		}
		if (!cap->chan[id])
			cap->chan[id] = g_byte_array_new();
		data = p + 2;
		rest = n - 2;
		if (!cap->rle) {
			g_byte_array_append(cap->chan[id], data, rest);
		} else {
			/* (count, value) pairs: count copies of the value byte. */
			size_t total = 0, at;
			for (i = 0; i + 1 < rest; i += 2)
				total += data[i];
			at = cap->chan[id]->len;
			g_byte_array_set_size(cap->chan[id], at + total);
			for (i = 0; i + 1 < rest; i += 2) {
				memset(cap->chan[id]->data + at, data[i + 1], data[i]);
				at += data[i];
			}
		}
		cap->frames++;
		break;
	case DL32_FRAME_OFFSET:
		/*
		 * [2 bytes][trigger offset, 5 bytes][per-channel byte totals,
		 * 5 bytes each][RLE only: flags, bit 0 over capacity,
		 * bit 1 over bandwidth].
		 */
		if (n < 7)
			break;
		cap->have_offset = TRUE;
		cap->trigger_offset = read_le40(p + 2);
		rest = n - 7;
		if (cap->rle && rest % 5 == 1) {
			cap->exceed_capacity = p[n - 1] & 1;
			cap->exceed_bandwidth = (p[n - 1] >> 1) & 1;
			rest--;
		}
		cap->num_totals = MIN(rest / 5, DL32_TRIGGER_POSITIONS);
		for (k = 0; k < cap->num_totals; k++)
			cap->totals[k] = read_le40(p + 7 + 5 * k);
		sr_info("Trigger offset %" PRIu64 ", %u channel totals%s%s.",
			cap->trigger_offset, cap->num_totals,
			cap->exceed_capacity ? ", over capacity" : "",
			cap->exceed_bandwidth ? ", over bandwidth" : "");
		break;
	case DL32_FRAME_ACK:
		/* [2 bytes][command][result]; a start (0x12) should report 3. */
		if (n < 3)
			break;
		sr_dbg("Ack for command 0x%02x: 0x%02x.", p[2], n > 3 ? p[3] : 0);
		if (p[2] == DL32_CMD_TRIGGER && n > 3 && p[3] != 3) {
			sr_dbg("Start acknowledged with code %u (DL16 uses 3).",
				p[3]);
			cap->ack_error = TRUE;
		}
		if (p[2] == DL32_CMD_STOP)
			cap->done = TRUE;
		break;
	case DL32_FRAME_PROGRESS:
		if (n >= 7)
			cap->progress = read_le40(p + 2);
		break;
	case DL32_FRAME_DONE:
		cap->done = TRUE;
		if (n >= 3 && p[2] == 1) {
			sr_warn("Device reported a stream overflow.");
			cap->exceed_capacity = TRUE;
		}
		break;
	default:
		break;
	}

	return frame->type;
}

static void log_frame(const struct dl32_frame *f)
{
	char hex[80];

	if (f->type == DL32_FRAME_DATA || !dl32_debug)
		return;
	sr_dbg("Frame type %u, %zu bytes: %s", f->type, f->len,
		dl32_hex(hex, sizeof(hex), f->payload, MIN(f->len, 24)));
}

/*
 * Parse all complete frames in buf into cap. Returns how many bytes were
 * consumed; the rest (a partial frame) must be kept for the next call.
 */
SR_PRIV size_t dl32_consume_frames(struct dl32_capture *cap,
	const uint8_t *buf, size_t len)
{
	struct dl32_frame frame;
	size_t pos, skip;
	enum dl32_parse_result r;

	pos = 0;
	while (pos < len) {
		r = dl32_parse_frame(buf + pos, len - pos, &skip, &frame);
		if (r != DL32_PARSE_FRAME) {
			pos += skip;
			break;
		}
		log_frame(&frame);
		dl32_capture_feed(cap, &frame);
		pos += skip + frame.size;
	}

	return pos;
}

/*
 * Add one USB read to the pending bytes and parse what is complete.
 *
 * Status frames (including "done") always arrive in their own short read.
 * If such a read holds a complete frame at its start while bytes are still
 * pending, those bytes are leftovers that look like an unfinished frame
 * (seen after RLE data); drop them so the status frame is not stuck
 * behind them.
 */
SR_PRIV void dl32_feed_bytes(struct dl32_capture *cap, GByteArray *pending,
	const uint8_t *p, size_t len)
{
	struct dl32_frame frame;
	size_t skip, used;

	if (pending->len && len < DL32_STATUS_MAX_LEN && p[0] == 0x0a &&
			dl32_parse_frame(p, len, &skip, &frame) == DL32_PARSE_FRAME &&
			skip == 0) {
		if (dl32_debug) {
			char hex[64];
			sr_dbg("Dropping %u stale bytes before a status frame: %s",
				pending->len, dl32_hex(hex, sizeof(hex),
				pending->data, MIN(pending->len, 16)));
		}
		g_byte_array_set_size(pending, 0);
	}

	g_byte_array_append(pending, p, len);
	used = dl32_consume_frames(cap, pending->data, pending->len);
	if (used)
		g_byte_array_remove_range(pending, 0, used);
}

/*
 * Work out which device channel feeds each sigrok channel and where the
 * requested window starts, mirroring ATK-Logic's DL16 trimming: the device
 * may send more data than asked for, and the trigger offset plus per-channel
 * totals tell where the pre-trigger window begins.
 */
SR_PRIV void dl32_capture_plan(const struct dl32_capture *cap,
	unsigned int num_channels, unsigned int ch_offset,
	uint32_t enabled_mask, gboolean buffer_mode, uint64_t depth,
	uint64_t pre_trigger, struct dl32_plan *plan)
{
	unsigned int ch, id, shift, tidx;
	uint64_t t, min_total, avail, count;
	gboolean any;

	/* Device ids are either 0-based or include the channel offset. */
	shift = 0;
	for (id = num_channels; id < DL32_TRIGGER_POSITIONS; id++) {
		if (cap->chan[id]) {
			shift = ch_offset;
			break;
		}
	}

	min_total = UINT64_MAX;
	for (ch = 0; ch < num_channels; ch++) {
		id = ch + shift;
		plan->src[ch] = -1;
		plan->start[ch] = 0;
		if (!(enabled_mask & (1u << ch)) || id >= DL32_TRIGGER_POSITIONS ||
				!cap->chan[id])
			continue;
		plan->src[ch] = id;

		if (!buffer_mode || !cap->have_offset)
			continue;
		tidx = (cap->num_totals > num_channels &&
			cap->num_totals >= ch_offset + num_channels) ?
			ch + ch_offset : ch;
		t = tidx < cap->num_totals ? cap->totals[tidx] : 0;
		/* A total larger than what arrived is not usable; use the data. */
		if (t > cap->chan[id]->len) {
			sr_dbg("D%u: reported %" PRIu64 " bytes, received %u.",
				ch, t, cap->chan[id]->len);
			t = cap->chan[id]->len;
		}
		if (t && t * 8 > depth && t * 8 > pre_trigger + cap->trigger_offset)
			plan->start[ch] = t * 8 - pre_trigger - cap->trigger_offset;
		/*
		 * Keep a full window: if the reported offset would leave fewer
		 * than 'depth' samples (e.g. an instant capture reporting 0),
		 * use the last 'depth' samples instead.
		 */
		if (t && depth && t * 8 >= depth && plan->start[ch] > t * 8 - depth)
			plan->start[ch] = t * 8 - depth;
		if (t && t < min_total)
			min_total = t;
	}

	/* Over capacity (RLE): align all channels to the shortest one. */
	if (buffer_mode && cap->rle && cap->exceed_capacity &&
			min_total != UINT64_MAX) {
		for (ch = 0; ch < num_channels; ch++) {
			if (plan->src[ch] < 0)
				continue;
			tidx = (cap->num_totals > num_channels &&
				cap->num_totals >= ch_offset + num_channels) ?
				ch + ch_offset : ch;
			t = tidx < cap->num_totals ? cap->totals[tidx] : 0;
			plan->start[ch] = t ? (t - min_total) * 8 : 0;
		}
	}

	/* Output as many samples as every captured channel can provide. */
	any = FALSE;
	count = UINT64_MAX;
	for (ch = 0; ch < num_channels; ch++) {
		if (plan->src[ch] < 0)
			continue;
		avail = (uint64_t)cap->chan[plan->src[ch]]->len * 8;
		avail = avail > plan->start[ch] ? avail - plan->start[ch] : 0;
		count = MIN(count, avail);
		any = TRUE;
	}
	if (!any)
		count = 0;
	if (depth)
		count = MIN(count, depth);
	plan->count = count;
}

/* Interleave n output samples starting at sample 'first' of the window. */
SR_PRIV void dl32_capture_render(const struct dl32_capture *cap,
	const struct dl32_plan *plan, unsigned int num_channels,
	uint64_t first, uint64_t n, uint8_t *out)
{
	const uint64_t *spread;
	const uint8_t *src[DL32_TRIGGER_POSITIONS];
	unsigned int unitsize, ch, j, c, k, b;
	uint64_t s, bit, q, blocks, acc;
	uint8_t v;

	spread = spread_table();
	unitsize = num_channels / 8;
	for (ch = 0; ch < num_channels; ch++)
		src[ch] = plan->src[ch] < 0 ? NULL :
			cap->chan[plan->src[ch]]->data;

	/* Whole blocks of 8 samples: build each output byte lane at once. */
	blocks = n / 8;
	for (s = 0; s < blocks; s++) {
		for (j = 0; j < unitsize; j++) {
			acc = 0;
			for (c = 0; c < 8; c++) {
				ch = j * 8 + c;
				if (!src[ch])
					continue;
				bit = plan->start[ch] + first + s * 8;
				q = bit >> 3;
				b = bit & 7;
				v = src[ch][q] >> b;
				if (b)
					v |= src[ch][q + 1] << (8 - b);
				acc |= spread[v] << c;
			}
			for (k = 0; k < 8; k++)
				out[(s * 8 + k) * unitsize + j] = (acc >> (8 * k)) & 0xff;
		}
	}

	/* Remaining samples one at a time. */
	memset(out + blocks * 8 * unitsize, 0, (n - blocks * 8) * unitsize);
	for (ch = 0; ch < num_channels; ch++) {
		if (!src[ch])
			continue;
		for (s = blocks * 8; s < n; s++) {
			bit = plan->start[ch] + first + s;
			if (src[ch][bit >> 3] & (1 << (bit & 7)))
				out[s * unitsize + ch / 8] |= 1 << (ch % 8);
		}
	}
}

/* --- Synchronous helpers (scan, start, stop; never in transfer callbacks). */

SR_PRIV int dl32_usb_write(libusb_device_handle *hdl, const uint8_t *buf, int len)
{
	int ret, done;

	ret = libusb_bulk_transfer(hdl, DL32_EP_OUT, (uint8_t *)buf, len,
		&done, 300);
	if (ret != 0 || done != len) {
		sr_dbg("Write of %d bytes failed: %s.", len,
			ret ? libusb_error_name(ret) : "short write");
		return SR_ERR;
	}

	return SR_OK;
}

/*
 * Bulk IN read. Always read into buffers of whole packets (DL32_READ_LEN):
 * with WinUSB, a buffer smaller than the device's packet (1024 bytes on
 * USB 3) makes the transfer fail instead of returning the data.
 * Returns the byte count (0 on timeout) or -1 on error.
 */
SR_PRIV int dl32_usb_read(libusb_device_handle *hdl, uint8_t *buf, int len,
	unsigned int timeout_ms)
{
	int ret, done;

	done = 0;
	ret = libusb_bulk_transfer(hdl, DL32_EP_IN, buf, len, &done, timeout_ms);
	if (ret == 0 || ret == LIBUSB_ERROR_TIMEOUT)
		return done;
	if (ret == LIBUSB_ERROR_OVERFLOW && done > 0) {
		sr_dbg("Read overflow after %d bytes.", done);
		return done;
	}

	sr_warn("USB read failed: %s.", libusb_error_name(ret));
	if (ret == LIBUSB_ERROR_PIPE)
		libusb_clear_halt(hdl, DL32_EP_IN);

	return -1;
}

/*
 * Read until a reply starting with prefix shows up (anywhere in a read, so
 * stale bytes in front of it do not matter) or timeout_ms passes.
 * On success the reply is moved to the start of buf and its length returned.
 */
static int read_reply(libusb_device_handle *hdl, uint8_t *buf,
	const uint8_t *prefix, size_t prefix_len, unsigned int timeout_ms)
{
	gint64 deadline;
	int n, i;

	deadline = g_get_monotonic_time() + (gint64)timeout_ms * 1000;
	do {
		n = dl32_usb_read(hdl, buf, DL32_READ_LEN, 100);
		if (n < 0)
			return -1;
		for (i = 0; i + (int)prefix_len <= n; i++) {
			if (!memcmp(buf + i, prefix, prefix_len)) {
				memmove(buf, buf + i, n - i);
				return n - i;
			}
		}
		if (n > 0 && dl32_debug) {
			char hex[64];
			sr_dbg("Skipping %d unexpected bytes: %s", n,
				dl32_hex(hex, sizeof(hex), buf, MIN(n, 16)));
		}
	} while (g_get_monotonic_time() < deadline);

	return 0;
}

/* Discard anything still queued in the device (e.g. a previous stream). */
SR_PRIV void dl32_usb_drain(libusb_device_handle *hdl)
{
	uint8_t *buf;
	int i, n;
	size_t total;

	buf = g_malloc(DL32_READ_LEN);
	total = 0;
	for (i = 0; i < 4096; i++) {
		n = dl32_usb_read(hdl, buf, DL32_READ_LEN, 50);
		if (n <= 0)
			break;
		total += n;
	}
	g_free(buf);
	if (total)
		sr_dbg("Drained %zu stale bytes.", total);
}

SR_PRIV int dl32_send_raw(libusb_device_handle *hdl, uint8_t cmd,
	const uint8_t *payload, size_t len)
{
	uint8_t pkt[DL32_RAW_PACKET_LEN];

	dl32_build_raw(pkt, cmd, payload, len);

	return dl32_usb_write(hdl, pkt, sizeof(pkt));
}

SR_PRIV int dl32_send_framed(libusb_device_handle *hdl, uint8_t cmd,
	const uint8_t *data, size_t len)
{
	uint8_t pkt[64 + DL32_FRAME_OVERHEAD];
	size_t n;

	if (len > 64)
		return SR_ERR_ARG;
	n = dl32_build_framed(pkt, cmd, data, len);

	return dl32_usb_write(hdl, pkt, n);
}

SR_PRIV int dl32_identify(libusb_device_handle *hdl, struct dl32_info *info)
{
	static const uint8_t hello_prefix[] = { 0x0a, DL32_RAW_CMD_HELLO, 0x01 };
	static const uint8_t info_prefix[] = { 0x0a, DL32_FRAME_INFO };
	uint8_t *buf;
	int n, attempt, ret;

	memset(info, 0, sizeof(*info));
	buf = g_malloc(DL32_READ_LEN);
	ret = SR_ERR;

	/* The first exchange after opening sometimes gets lost; retry. */
	n = 0;
	for (attempt = 1; attempt <= 3; attempt++) {
		dl32_usb_drain(hdl);
		if (dl32_send_raw(hdl, DL32_RAW_CMD_HELLO, NULL, 0) != SR_OK) {
			sr_warn("Could not send the hello command (attempt %d).",
				attempt);
			continue;
		}
		n = read_reply(hdl, buf, ARRAY_AND_SIZE(hello_prefix), 500);
		if (n > 0 && dl32_parse_hello(buf, n, info))
			break;
		sr_warn("No valid hello reply (attempt %d, %s).", attempt,
			n < 0 ? "read error" : "timeout");
		n = 0;
	}
	if (n <= 0)
		goto out;

	sr_dbg("Hello: mode '%c', MCU %d, variant %d, plus %d.",
		info->mode, info->mcu_version, info->variant, info->plus);
	if (info->mode != 'a') {
		sr_err("Device is not in application mode (mode 0x%02x); "
			"update its firmware with ATK-Logic first.", info->mode);
		goto out;
	}

	if (dl32_send_framed(hdl, DL32_CMD_DEVICE_DATA, NULL, 0) == SR_OK) {
		n = read_reply(hdl, buf, ARRAY_AND_SIZE(info_prefix), 500);
		if (n > 0) {
			dl32_parse_name(buf, n, info);
			if (dl32_debug) {
				char hex[160];
				sr_detail("Device data: %s", dl32_hex(hex,
					sizeof(hex), buf, MIN(n, 48)));
			}
		}
	}
	if (!info->name[0])
		g_strlcpy(info->name, "DL32", sizeof(info->name));

	dl32_usb_drain(hdl);
	ret = SR_OK;
out:
	g_free(buf);

	return ret;
}

SR_PRIV void dl32_send_stop(libusb_device_handle *hdl)
{
	const uint8_t zero = 0;

	dl32_send_framed(hdl, DL32_CMD_STOP, &zero, 1);
	dl32_send_raw(hdl, DL32_RAW_CMD_RESET, &zero, 1);
	dl32_usb_drain(hdl);
}



SR_PRIV gboolean dl32_debug;

/* "0a 81 01 ..." for the log */
SR_PRIV const char *dl32_hex(char *out, size_t size, const uint8_t *buf,
	size_t len)
{
	size_t i, at = 0;

	if (!size)
		return "";
	out[0] = '\0';
	for (i = 0; i < len && at + 4 <= size; i++)
		at += (size_t)g_snprintf(out + at, size - at, i ? " %02x" : "%02x",
			buf[i]);
	return out;
}
