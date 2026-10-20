/*
 * Host-side capture core for ALL LOGIC (libsigrok4DSL) drivers.
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
 * For analyzers that stream every sample to the PC and leave the trigger to
 * the host (hsdaoh-la, cypress-fx3): trigger search, pre-trigger history,
 * capture buffer, lost-data handling (from the libsigrok hsdaoh-la driver's
 * protocol.c) and the conversion to DSView's LA_CROSS_DATA layout. Up to 8
 * channels with 1 byte per sample, or up to 16 with 2 bytes per sample
 * (little endian, bit k = channel k). No dependency on DSView or a device
 * library, so it can be tested on its own.
 *
 * Acquisition model:
 *   - the stream is scanned on the PC for the trigger condition
 *   - pre-trigger + post-trigger samples are collected in RAM
 *   - only when the capture is complete is it sent to the session
 *   - lost data (the device's word counter jumps) restarts the pre-trigger
 *     history, and discards a capture in progress (it would have a hole)
 *
 * Not thread safe: the driver serializes access with its own mutex.
 */

#ifndef LIBSIGROK_HARDWARE_COMMON_CAPTURE_CORE_H
#define LIBSIGROK_HARDWARE_COMMON_CAPTURE_CORE_H

#include <stdint.h>
#include <stddef.h>

#define HL_MAX_CHANNELS 16

enum hl_state {
	HL_IDLE,	/* no acquisition running */
	HL_ARMED,	/* waiting for the trigger */
	HL_CAPTURING,	/* trigger found, collecting post-trigger samples */
	HL_DONE,	/* buffer complete, waiting to be sent */
	HL_FAILED,	/* device error, end the acquisition */
};

/* Per-channel trigger condition masks (bit n = channel n), all ANDed. */
struct hl_trigger_spec {
	uint16_t must_zero, must_one, rising, falling, edge;
};

struct hl_capture {
	/* trigger: tbl[(prev << 8) | cur] != 0 where the conditions on
	 * channels 0..7 are met; trig_tbl_hi the same for channels 8..15
	 * (NULL when none of them is used) */
	int have_trigger;
	uint8_t *trig_tbl;
	uint8_t *trig_tbl_hi;
	unsigned int unit;	/* bytes per sample: 1 or 2 */

	int state;
	uint64_t pre;		/* samples before the trigger */
	uint64_t post;		/* samples from the trigger on */
	uint8_t *buf;		/* pre + post samples, `unit` bytes each */
	uint64_t buf_fill;
	uint64_t trigger_pos;	/* index of the trigger sample in buf */
	uint8_t *hist;		/* pre-trigger ring buffer */
	uint64_t hist_pos, hist_len;
	int have_prev;
	uint16_t prev;
	uint64_t next_cnt;	/* expected stream word counter */
	int cnt_valid;
	uint64_t gaps;		/* lost-data events during this acquisition */
	uint64_t gap_words;	/* words lost in the last event (for logging) */
	uint8_t *unpack;	/* scratch for one unpacked buffer */
	size_t unpack_size;
	uint8_t carry[2];	/* half a 16-bit sample left by a transfer */
	int carry_len;
};

/* Build the trigger lookup table. Returns 0 (no trigger needed), 1 (trigger
 * armed) or -1 (out of memory). */
int hl_build_trigger(struct hl_capture *c, const struct hl_trigger_spec *t);

/* Allocate the capture and history buffers for pre + post samples of
 * `unit` (1 or 2) bytes. Returns 0 or -1 (out of memory). */
int hl_alloc(struct hl_capture *c, uint64_t pre, uint64_t post, unsigned int unit);
void hl_free_buffers(struct hl_capture *c);
void hl_free(struct hl_capture *c);

/* Start waiting for the trigger (or for data, without a trigger). */
void hl_reset(struct hl_capture *c);
/* Forget the stream position, e.g. after a sample rate change. */
void hl_invalidate_counter(struct hl_capture *c);

/*
 * Feed one buffer from libhsdaoh: `words` 16-bit little-endian link words,
 * two samples each with the earlier sample in the high byte; `data_cnt` is
 * the device's count of words sent before this buffer. Only acts while the
 * state is HL_ARMED or HL_CAPTURING. Returns 1 when data was lost before
 * this buffer (gaps / gap_words tell how much), 0 otherwise, -1 if out of
 * memory (the state is then HL_FAILED). 1-byte samples only.
 */
int hl_feed(struct hl_capture *c, const uint8_t *buf, size_t words,
	    uint64_t data_cnt);
/*
 * Same with `spw` samples per word: 2 (la8, as hl_feed) or 4 (la4: 4-bit
 * samples, the earliest in bits 15:12 of the word, stored one per byte in
 * bits 3:0). The lost-data counter still counts words.
 */
int hl_feed_spw(struct hl_capture *c, const uint8_t *buf, size_t words,
		uint64_t data_cnt, unsigned int spw);

/*
 * Feed nbytes of samples (`unit` bytes each, bit k = channel k) in stream
 * order, from a device without a stream counter (lost data cannot be
 * detected). A 16-bit sample split between two calls is joined. Only acts
 * while the state is HL_ARMED or HL_CAPTURING.
 */
void hl_feed_bytes(struct hl_capture *c, const uint8_t *s, size_t nbytes);

/*
 * Convert n samples (n a multiple of 64; `unit` bytes per sample, bit k =
 * channel k) to LA_CROSS_DATA: for every group of 64 samples, one
 * little-endian 64-bit plane per enabled channel (bit i = sample i of the
 * group), in the order of en_bits. Writes n / 64 * 8 * en_count bytes.
 */
void hl_to_cross(const uint8_t *src, uint64_t n, const uint8_t *en_bits,
		 int en_count, uint8_t *out, unsigned int unit);

#endif
