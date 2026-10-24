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

#include <stdlib.h>
#include <string.h>
#include "capture_core.h"

#define HL_MIN(a, b) ((a) < (b) ? (a) : (b))

/* ------------------------------------------------------------- trigger */

/*
 * Trigger: all conditions must hold at one sample, comparing it with the
 * previous one. For a sample c after p, the bits that violate the
 * conditions are
 *     (c & zero) | (one & ~c) | (rise & (p | ~c)) | (fall & (~p | c))
 *     | (edge & ~(p ^ c))
 * and the sample matches when that is 0. find_trigger() evaluates this
 * bitwise on 64-bit words holding 8 samples (1 byte each) or 4 samples
 * (16 bits each) at once and finds the earliest match with a zero-lane
 * test; no tables, no SIMD instructions needed.
 */
int hl_build_trigger(struct hl_capture *c, const struct hl_trigger_spec *t)
{
	c->have_trigger = 0;
	memset(&c->trig, 0, sizeof(c->trig));
	if (!t || !(t->must_zero | t->must_one | t->rising | t->falling | t->edge))
		return 0;	/* nothing to wait for */
	c->trig = *t;
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
	c->have_trigger = 0;
}

int hl_alloc(struct hl_capture *c, uint64_t pre, uint64_t post, unsigned int unit)
{
	uint64_t bytes;

	hl_free_buffers(c);
	if (unit != 1 && unit != 2)
		return -1;
	bytes = (pre + post) * unit;
	if ((uint64_t)(size_t)bytes != bytes)
		return -1;	/* does not fit this process */
	c->unit = unit;
	c->pre = pre;
	c->post = post;
	c->buf = malloc((size_t)bytes);
	if (pre)
		c->hist = malloc((size_t)(pre * unit));
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

/* n samples (positions and lengths below are in samples) */
static void hist_push(struct hl_capture *c, const uint8_t *s, size_t n)
{
	uint64_t cap = c->pre;
	unsigned int u = c->unit;

	if (!cap)
		return;
	if (n >= cap) {
		memcpy(c->hist, s + (n - cap) * u, (size_t)(cap * u));
		c->hist_pos = 0;
		c->hist_len = cap;
		return;
	}
	while (n) {
		size_t k = (size_t)HL_MIN((uint64_t)n, cap - c->hist_pos);
		memcpy(c->hist + c->hist_pos * u, s, k * u);
		c->hist_pos = (c->hist_pos + k) % cap;
		c->hist_len = HL_MIN(c->hist_len + k, cap);
		s += k * u;
		n -= k;
	}
}

/* copy the newest 'want' history samples into the capture buffer */
static uint64_t hist_copy(struct hl_capture *c, uint8_t *dst, uint64_t want)
{
	uint64_t cap = c->pre;
	uint64_t n = HL_MIN(want, c->hist_len);
	unsigned int u = c->unit;
	uint64_t start, first;

	if (!n)
		return 0;
	start = (c->hist_pos + cap - n) % cap;
	first = HL_MIN(n, cap - start);
	memcpy(dst, c->hist + start * u, (size_t)(first * u));
	memcpy(dst + first * u, c->hist, (size_t)((n - first) * u));
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
	c->carry_len = 0;
	c->state = HL_ARMED;
}

void hl_invalidate_counter(struct hl_capture *c)
{
	c->cnt_valid = 0;
}

static inline unsigned int sample_at(const struct hl_capture *c,
				     const uint8_t *s, size_t j)
{
	return c->unit == 2 ? (unsigned int)s[2 * j] | (unsigned int)s[2 * j + 1] << 8
			    : s[j];
}

/* bits of sample c (after p) that violate the trigger; 0 = match. Works
 * on one sample or lane by lane on several samples packed in a word. */
#define TRIG_BAD(c, p, z, o, r, f, e) \
	(((c) & (z)) | ((o) & ~(c)) | ((r) & ((p) | ~(c))) | \
	 ((f) & (~(p) | (c))) | ((e) & ~((p) ^ (c))))

static inline uint64_t load_u64(const uint8_t *p)
{
	uint64_t v;
	memcpy(&v, p, sizeof(v));	/* unaligned load */
	return v;
}

static inline unsigned int ctz64(uint64_t v)
{
#if defined(__GNUC__) || defined(__clang__)
	return (unsigned int)__builtin_ctzll(v);
#else
	unsigned int n = 0;
	while (!(v & 1)) {
		v >>= 1;
		n++;
	}
	return n;
#endif
}

static inline int little_endian(void)
{
	const uint16_t one = 1;
	return *(const uint8_t *)&one == 1;
}

/*
 * Index of the first sample in s[from..n) that meets the trigger, or n.
 * 8 one-byte or 4 two-byte samples per step: a sample matches when its
 * lane of `bad` is 0; (bad - 0x01..) & ~bad & 0x80.. marks zero lanes and
 * is exact for the lowest one, the earliest sample (little endian).
 */
static size_t find_trigger(struct hl_capture *c, const uint8_t *s, size_t from,
			   size_t n)
{
	const unsigned int z = c->trig.must_zero, o = c->trig.must_one,
		r = c->trig.rising, f = c->trig.falling, e = c->trig.edge;
	const unsigned int u = c->unit;
	const unsigned int mask = u == 2 ? 0xffff : 0xff;
	unsigned int p;
	size_t j;

	if (from >= n)
		return n;
	/* the sample before s[from]; without history the first sample cannot
	 * be an edge: use itself */
	if (from)
		p = sample_at(c, s, from - 1);
	else
		p = c->have_prev ? c->prev : sample_at(c, s, 0);
	j = from;
	if (!(TRIG_BAD(sample_at(c, s, j), p, z, o, r, f, e) & mask))
		return j;
	j++;

	if (little_endian()) {
		/* s[j - 1] exists from here on */
		const uint64_t rep = u == 2 ? UINT64_C(0x0001000100010001)
					    : UINT64_C(0x0101010101010101);
		const uint64_t Z = (z & mask) * rep, O = (o & mask) * rep,
			R = (r & mask) * rep, F = (f & mask) * rep,
			E = (e & mask) * rep;
		const uint64_t HIGHS = rep << (u * 8 - 1);
		const size_t per = 8 / u;	/* samples per word */

		for (; j + per <= n; j += per) {
			uint64_t cv = load_u64(s + j * u);
			uint64_t pv = load_u64(s + j * u - u);
			uint64_t bad = TRIG_BAD(cv, pv, Z, O, R, F, E);
			uint64_t hit = (bad - rep) & ~bad & HIGHS;
			if (hit)
				return j + ctz64(hit) / (u * 8);
		}
	}
	for (; j < n; j++)
		if (!(TRIG_BAD(sample_at(c, s, j), sample_at(c, s, j - 1),
			       z, o, r, f, e) & mask))
			return j;
	return n;
}

static void start_capture(struct hl_capture *c, const uint8_t *s, size_t at)
{
	/* pre-trigger: history from earlier buffers plus this buffer up to 'at' */
	unsigned int u = c->unit;
	uint64_t want = c->pre;
	uint64_t from_buf = HL_MIN((uint64_t)at, want);
	uint64_t from_hist = 0;

	if (want > from_buf)
		from_hist = hist_copy(c, c->buf, want - from_buf);
	memcpy(c->buf + from_hist * u, s + (at - from_buf) * u, (size_t)(from_buf * u));
	c->buf_fill = from_hist + from_buf;
	c->trigger_pos = c->buf_fill;
	c->state = HL_CAPTURING;
}

/* n samples of c->unit bytes */
static void process(struct hl_capture *c, const uint8_t *s, size_t n)
{
	unsigned int u = c->unit;
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
			c->prev = (uint16_t)sample_at(c, s, n - 1);
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
		memcpy(c->buf + c->buf_fill * u, s + i * u, (size_t)(k * u));
		c->buf_fill += k;
		if (c->buf_fill >= c->trigger_pos + c->post)
			c->state = HL_DONE;
	}
}

/* la4 unpacking: byte -> its two nibbles, high nibble (earlier sample)
 * first, as a little-endian 16-bit value */
static uint16_t nibble_tbl[256];

static void nibble_tbl_init(void)
{
	unsigned int b;

	if (nibble_tbl[0x10])
		return;
	for (b = 0; b < 256; b++)
		nibble_tbl[b] = (uint16_t)((b >> 4) | ((b & 0x0f) << 8));
}

/*
 * Unpack `words` 16-bit link words (little endian) to one byte per sample:
 * spw 2 (la8): high byte = earlier sample; spw 4 (la4): bits 15:12 first.
 * As in the libsigrok hsdaoh-la driver.
 */
static void unpack_words(uint8_t *dst, const uint8_t *src, size_t words,
			 unsigned int spw)
{
	size_t w = 0;

	if (spw == 4) {
		if (little_endian()) {
			/* 2 table lookups and one 32-bit store per word */
			nibble_tbl_init();
			for (; w < words; w++) {
				uint32_t v = (uint32_t)nibble_tbl[src[2 * w + 1]] |
					((uint32_t)nibble_tbl[src[2 * w]] << 16);
				memcpy(dst + 4 * w, &v, 4);
			}
			return;
		}
		for (; w < words; w++) {
			uint8_t hi = src[2 * w + 1], lo = src[2 * w];
			dst[4 * w] = hi >> 4;
			dst[4 * w + 1] = hi & 0x0f;
			dst[4 * w + 2] = lo >> 4;
			dst[4 * w + 3] = lo & 0x0f;
		}
		return;
	}
	/* la8: swap the two bytes of every word, 4 words at a time (the swap
	 * within 16-bit lanes does not depend on the byte order) */
	for (; w + 4 <= words; w += 4) {
		uint64_t v = load_u64(src + 2 * w);
		v = ((v >> 8) & UINT64_C(0x00ff00ff00ff00ff)) |
		    ((v << 8) & UINT64_C(0xff00ff00ff00ff00));
		memcpy(dst + 2 * w, &v, 8);
	}
	for (; w < words; w++) {
		dst[2 * w] = src[2 * w + 1];
		dst[2 * w + 1] = src[2 * w];
	}
}

int hl_feed(struct hl_capture *c, const uint8_t *buf, size_t words,
	    uint64_t data_cnt)
{
	return hl_feed_spw(c, buf, words, data_cnt, 2);
}

int hl_feed_spw(struct hl_capture *c, const uint8_t *buf, size_t words,
		uint64_t data_cnt, unsigned int spw)
{
	size_t n;
	int lost = 0;

	if (c->state != HL_ARMED && c->state != HL_CAPTURING)
		return 0;
	if (!words || c->unit != 1 || (spw != 2 && spw != 4))
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

	/*
	 * Capturing (after the trigger): unpack straight into the capture
	 * buffer, no intermediate copy. Whole words only; when the capture
	 * ends inside a word, that word is unpacked separately.
	 */
	if (c->state == HL_CAPTURING) {
		uint64_t need = c->trigger_pos + c->post - c->buf_fill;
		size_t direct = (size_t)HL_MIN((uint64_t)words, need / spw);
		uint8_t last[4];

		unpack_words(c->buf + c->buf_fill, buf, direct, spw);
		c->buf_fill += (uint64_t)direct * spw;
		if (c->buf_fill >= c->trigger_pos + c->post) {
			c->state = HL_DONE;
			return lost;
		}
		if (direct == words)
			return lost;
		unpack_words(last, buf + 2 * direct, 1, spw);
		need = c->trigger_pos + c->post - c->buf_fill;
		memcpy(c->buf + c->buf_fill, last, (size_t)need);
		c->buf_fill += need;
		c->state = HL_DONE;
		return lost;
	}

	/* waiting for the trigger: unpack, then search and keep the history */
	n = words * spw;
	if (n > c->unpack_size) {
		free(c->unpack);
		c->unpack = malloc(n);
		c->unpack_size = c->unpack ? n : 0;
		if (!c->unpack) {
			c->state = HL_FAILED;
			return -1;
		}
	}
	unpack_words(c->unpack, buf, words, spw);
	process(c, c->unpack, n);
	return lost;
}

void hl_feed_bytes(struct hl_capture *c, const uint8_t *s, size_t nbytes)
{
	size_t n;

	if (c->state != HL_ARMED && c->state != HL_CAPTURING)
		return;
	if (c->unit == 1) {
		if (nbytes)
			process(c, s, nbytes);
		return;
	}
	/* 16-bit samples: complete a sample split between two buffers */
	if (c->carry_len && nbytes) {
		uint8_t pair[2];
		pair[0] = c->carry[0];
		pair[1] = s[0];
		c->carry_len = 0;
		s++;
		nbytes--;
		process(c, pair, 1);
		if (c->state != HL_ARMED && c->state != HL_CAPTURING)
			return;
	}
	n = nbytes / 2;
	if (n)
		process(c, s, n);
	if (nbytes & 1) {
		c->carry[0] = s[nbytes - 1];
		c->carry_len = 1;
	}
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

/* 8 bytes, `stride` apart, as a little-endian u64 */
static inline uint64_t gather8(const uint8_t *p, unsigned int stride)
{
	uint64_t x = 0;
	int k;

	for (k = 7; k >= 0; k--)
		x = x << 8 | p[k * stride];
	return x;
}

void hl_to_cross(const uint8_t *src, uint64_t n, const uint8_t *en_bits,
		 int en_count, uint8_t *out, unsigned int unit)
{
	uint64_t g, groups = n / 64;
	int b, e;

	for (g = 0; g < groups; g++) {
		uint64_t tb[2][8];
		const uint8_t *grp = src + g * 64 * unit;

		for (b = 0; b < 8; b++) {
			/* byte k of tb[0][b]: channel k for samples 8b..8b+7,
			 * tb[1][b]: channel 8 + k */
			tb[0][b] = transpose8(gather8(grp + b * 8 * unit, unit));
			if (unit == 2)
				tb[1][b] = transpose8(gather8(grp + b * 16 + 1, 2));
		}
		for (e = 0; e < en_count; e++) {
			unsigned int ch = en_bits[e];
			const uint64_t *t = tb[ch >> 3];
			unsigned int sh = 8u * (ch & 7);
			for (b = 0; b < 8; b++)
				out[b] = (uint8_t)(t[b] >> sh);
			out += 8;
		}
	}
}
