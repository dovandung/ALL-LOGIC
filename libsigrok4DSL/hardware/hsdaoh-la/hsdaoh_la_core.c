/*
 * hsdaoh-la capture core for ALL LOGIC (libsigrok4DSL).
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

#include <stdlib.h>
#include <string.h>
#include "hsdaoh_la_core.h"

#define HL_MIN(a, b) ((a) < (b) ? (a) : (b))

/* ------------------------------------------------------------- trigger */

/*
 * A 64K lookup table: tbl[(previous sample << 8) | sample] is 1 where all
 * conditions are true. One table lookup per sample keeps the trigger
 * search fast enough for the full stream rate.
 */
int hl_build_trigger(struct hl_capture *c, const struct hl_trigger_spec *t)
{
	unsigned int p, s;

	free(c->trig_tbl);
	c->trig_tbl = NULL;
	c->have_trigger = 0;

	if (!t || !(t->must_zero | t->must_one | t->rising | t->falling | t->edge))
		return 0;	/* nothing to wait for */

	c->trig_tbl = malloc(65536);
	if (!c->trig_tbl)
		return -1;

	for (p = 0; p < 256; p++) {
		for (s = 0; s < 256; s++) {
			int ok = (s & t->must_zero) == 0 &&
				 (s & t->must_one) == t->must_one &&
				 ((~p & s) & t->rising) == t->rising &&
				 ((p & ~s) & t->falling) == t->falling &&
				 ((p ^ s) & t->edge) == t->edge;
			c->trig_tbl[(p << 8) | s] = ok ? 1 : 0;
		}
	}
	c->have_trigger = 1;
	return 1;
}

/* ------------------------------------------------------------- buffers */

void hl_free_buffers(struct hl_capture *c)
{
	free(c->buf);
	c->buf = NULL;
	free(c->hist);
	c->hist = NULL;
	free(c->unpack);
	c->unpack = NULL;
	c->unpack_size = 0;
}

void hl_free(struct hl_capture *c)
{
	hl_free_buffers(c);
	free(c->trig_tbl);
	c->trig_tbl = NULL;
	c->have_trigger = 0;
}

int hl_alloc(struct hl_capture *c, uint64_t pre, uint64_t post)
{
	hl_free_buffers(c);
	if ((uint64_t)(size_t)(pre + post) != pre + post)
		return -1;	/* does not fit this process */
	c->pre = pre;
	c->post = post;
	c->buf = malloc((size_t)(pre + post));
	if (pre)
		c->hist = malloc((size_t)pre);
	if (!c->buf || (pre && !c->hist)) {
		hl_free_buffers(c);
		return -1;
	}
	return 0;
}

/* forget the pre-trigger history (after a gap in the stream) */
static void hist_reset(struct hl_capture *c)
{
	c->hist_pos = 0;
	c->hist_len = 0;
	c->have_prev = 0;
}

static void hist_push(struct hl_capture *c, const uint8_t *s, size_t n)
{
	uint64_t cap = c->pre;

	if (!cap)
		return;
	if (n >= cap) {
		memcpy(c->hist, s + n - cap, (size_t)cap);
		c->hist_pos = 0;
		c->hist_len = cap;
		return;
	}
	while (n) {
		size_t k = (size_t)HL_MIN((uint64_t)n, cap - c->hist_pos);
		memcpy(c->hist + c->hist_pos, s, k);
		c->hist_pos = (c->hist_pos + k) % cap;
		c->hist_len = HL_MIN(c->hist_len + k, cap);
		s += k;
		n -= k;
	}
}

/* copy the newest 'want' history samples into the capture buffer */
static uint64_t hist_copy(struct hl_capture *c, uint8_t *dst, uint64_t want)
{
	uint64_t cap = c->pre;
	uint64_t n = HL_MIN(want, c->hist_len);
	uint64_t start, first;

	if (!n)
		return 0;
	start = (c->hist_pos + cap - n) % cap;
	first = HL_MIN(n, cap - start);
	memcpy(dst, c->hist + start, (size_t)first);
	memcpy(dst + first, c->hist, (size_t)(n - first));
	return n;
}

void hl_reset(struct hl_capture *c)
{
	hist_reset(c);
	c->buf_fill = 0;
	c->trigger_pos = 0;
	c->cnt_valid = 0;
	c->gaps = 0;
	c->gap_words = 0;
	c->state = HL_ARMED;
}

void hl_invalidate_counter(struct hl_capture *c)
{
	c->cnt_valid = 0;
}

/* index of the first sample in s[from..n) that meets the trigger, or n */
static size_t find_trigger(struct hl_capture *c, const uint8_t *s, size_t from,
			   size_t n)
{
	const uint8_t *tbl = c->trig_tbl;
	unsigned int p;
	size_t j;

	if (from >= n)
		return n;
	/* the sample before s[from]; without history the first sample cannot
	 * be an edge: use itself */
	if (from)
		p = s[from - 1];
	else
		p = c->have_prev ? c->prev : s[0];
	for (j = from; j < n; j++) {
		unsigned int v = s[j];
		if (tbl[(p << 8) | v])
			return j;
		p = v;
	}
	return n;
}

static void start_capture(struct hl_capture *c, const uint8_t *s, size_t at)
{
	/* pre-trigger: history from earlier buffers plus this buffer up to 'at' */
	uint64_t want = c->pre;
	uint64_t from_buf = HL_MIN((uint64_t)at, want);
	uint64_t from_hist = 0;

	if (want > from_buf)
		from_hist = hist_copy(c, c->buf, want - from_buf);
	memcpy(c->buf + from_hist, s + at - from_buf, (size_t)from_buf);
	c->buf_fill = from_hist + from_buf;
	c->trigger_pos = c->buf_fill;
	c->state = HL_CAPTURING;
}

static void process(struct hl_capture *c, const uint8_t *s, size_t n)
{
	size_t i = 0;

	if (c->state == HL_ARMED) {
		size_t at = 0;
		if (c->have_trigger) {
			/*
			 * Like other logic analyzers: a trigger only counts once
			 * the pre-trigger part is complete, i.e. at least 'pre'
			 * samples were received (since Run, or since lost data).
			 * Else a busy signal triggers at once with (almost) no
			 * samples before the trigger.
			 */
			size_t from = 0;
			if (c->hist_len < c->pre) {
				uint64_t need = c->pre - c->hist_len;
				from = need < n ? (size_t)need : n;
			}
			at = find_trigger(c, s, from, n);
		}
		if (at >= n) {
			hist_push(c, s, n);
			c->prev = s[n - 1];
			c->have_prev = 1;
			return;
		}
		start_capture(c, s, at);
		i = at;
	}

	if (c->state == HL_CAPTURING) {
		uint64_t room = c->pre + c->post - c->buf_fill;
		/* never take more pre-trigger samples than exist: the capture
		 * ends post samples after the trigger */
		uint64_t until_end = c->trigger_pos + c->post - c->buf_fill;
		uint64_t k = HL_MIN(HL_MIN(room, until_end), (uint64_t)(n - i));
		memcpy(c->buf + c->buf_fill, s + i, (size_t)k);
		c->buf_fill += k;
		if (c->buf_fill >= c->trigger_pos + c->post)
			c->state = HL_DONE;
	}
}

int hl_feed(struct hl_capture *c, const uint8_t *buf, size_t words,
	    uint64_t data_cnt)
{
	size_t n, w;
	int lost = 0;

	if (c->state != HL_ARMED && c->state != HL_CAPTURING)
		return 0;
	if (!words)
		return 0;

	/* lost data: the device's word counter does not continue */
	if (c->cnt_valid && data_cnt != c->next_cnt) {
		c->gaps++;
		c->gap_words = data_cnt - c->next_cnt;
		lost = 1;
		if (c->state == HL_CAPTURING) {
			/* the capture would contain a hole: start over */
			c->state = HL_ARMED;
			c->buf_fill = 0;
		}
		hist_reset(c);
	}
	c->next_cnt = data_cnt + words;
	c->cnt_valid = 1;

	n = words * 2;
	if (n > c->unpack_size) {
		free(c->unpack);
		c->unpack = malloc(n);
		c->unpack_size = c->unpack ? n : 0;
		if (!c->unpack) {
			c->state = HL_FAILED;
			return -1;
		}
	}
	/* two samples per 16-bit word, the earlier one in the high byte; the
	 * words arrive little endian: swap the bytes of every word */
	for (w = 0; w < words; w++) {
		c->unpack[2 * w] = buf[2 * w + 1];
		c->unpack[2 * w + 1] = buf[2 * w];
	}
	process(c, c->unpack, n);
	return lost;
}

/* ---------------------------------------------------------- cross format */

/* Transpose an 8x8 bit matrix: byte j bit k -> byte k bit j. */
static inline uint64_t transpose8(uint64_t x)
{
	uint64_t t;

	t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AAULL;
	x = x ^ t ^ (t << 7);
	t = (x ^ (x >> 14)) & 0x0000CCCC0000CCCCULL;
	x = x ^ t ^ (t << 14);
	t = (x ^ (x >> 28)) & 0x00000000F0F0F0F0ULL;
	x = x ^ t ^ (t << 28);
	return x;
}

void hl_to_cross(const uint8_t *src, uint64_t n, const uint8_t *en_bits,
		 int en_count, uint8_t *out)
{
	uint64_t g, groups = n / 64;
	int b, e;

	for (g = 0; g < groups; g++) {
		uint64_t tb[8];
		for (b = 0; b < 8; b++) {
			const uint8_t *p = src + g * 64 + b * 8;
			uint64_t x = (uint64_t)p[0] | (uint64_t)p[1] << 8 |
				     (uint64_t)p[2] << 16 | (uint64_t)p[3] << 24 |
				     (uint64_t)p[4] << 32 | (uint64_t)p[5] << 40 |
				     (uint64_t)p[6] << 48 | (uint64_t)p[7] << 56;
			/* byte k of tb[b]: channel k for samples 8b..8b+7 */
			tb[b] = transpose8(x);
		}
		for (e = 0; e < en_count; e++) {
			unsigned sh = 8u * en_bits[e];
			for (b = 0; b < 8; b++)
				out[b] = (uint8_t)(tb[b] >> sh);
			out += 8;
		}
	}
}
