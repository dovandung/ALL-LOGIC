/*
 * Cypress FX3 firmware upload over the USB boot loader, for ALL LOGIC
 * (libsigrok4DSL).
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
 * An FX3 without firmware in flash enumerates with its ROM boot loader
 * (04B4:00F3, or a second stage boot loader with the same protocol). It
 * takes a firmware image (the "CY" .img format of the FX3 SDK) into RAM:
 *
 *   image:  'C' 'Y' bImageCTL bImageType
 *           { u32 length (in 32-bit words), u32 address, data } ...
 *           { u32 0, u32 entry address }  u32 checksum
 *   (little endian; checksum = sum of all data words of all sections)
 *
 *   write:  vendor OUT request 0xA0, wValue = address bits 0..15,
 *           wIndex = address bits 16..31, up to 4 KB of data
 *   read:   the same as vendor IN request (used to verify)
 *   jump:   vendor OUT request 0xA0 to the entry address, no data
 *
 * The FX3 then runs the firmware and enumerates again with its own IDs.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "fx3load.h"

#undef LOG_PREFIX
#define LOG_PREFIX "fx3load: "

#define FX3_REQ_LOAD	0xa0
#define FX3_CHUNK	4096
#define FX3_TIMEOUT_MS	3000

static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
	       (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

SR_PRIV int fx3_image_parse(const uint8_t *img, size_t len,
			    struct fx3_section *sections, int max_sections,
			    int *num_sections, uint32_t *entry)
{
	size_t off = 4;
	uint32_t sum = 0;
	int n = 0;

	if (len < 4 + 8 + 4 || img[0] != 'C' || img[1] != 'Y')
		return FX3_IMG_BAD_HEADER;
	/* bImageCTL bit 0 set: not executable code (e.g. an I2C/SPI image) */
	if (img[2] & 0x01)
		return FX3_IMG_NOT_EXECUTABLE;
	/* bImageType 0xB0: normal firmware with checksum */
	if (img[3] != 0xb0)
		return FX3_IMG_BAD_HEADER;

	for (;;) {
		uint32_t words, addr, k;

		if (off + 8 > len)
			return FX3_IMG_TRUNCATED;
		words = rd32(img + off);
		addr = rd32(img + off + 4);
		off += 8;
		if (words == 0) {
			if (off + 4 > len)
				return FX3_IMG_TRUNCATED;
			if (rd32(img + off) != sum)
				return FX3_IMG_BAD_CHECKSUM;
			*entry = addr;
			*num_sections = n;
			return FX3_IMG_OK;
		}
		if ((uint64_t)words * 4 > len - off)
			return FX3_IMG_TRUNCATED;
		if (n >= max_sections)
			return FX3_IMG_TOO_MANY;
		sections[n].addr = addr;
		sections[n].data = img + off;
		sections[n].len = words * 4;
		n++;
		for (k = 0; k < words; k++)
			sum += rd32(img + off + 4 * k);
		off += (size_t)words * 4;
	}
}

static const char *img_error(int r)
{
	switch (r) {
	case FX3_IMG_BAD_HEADER: return "not an FX3 firmware image (\"CY\", type 0xB0)";
	case FX3_IMG_NOT_EXECUTABLE: return "image is not executable firmware";
	case FX3_IMG_TRUNCATED: return "image is truncated";
	case FX3_IMG_BAD_CHECKSUM: return "image checksum mismatch";
	case FX3_IMG_TOO_MANY: return "too many sections";
	default: return "unknown error";
	}
}

static int write_and_verify(libusb_device_handle *hdl, uint32_t addr,
			    const uint8_t *data, size_t len)
{
	uint8_t chunk[FX3_CHUNK], back[FX3_CHUNK];

	while (len) {
		int n = len < FX3_CHUNK ? (int)len : FX3_CHUNK;
		int ret;

		memcpy(chunk, data, (size_t)n);
		ret = libusb_control_transfer(hdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			FX3_REQ_LOAD, addr & 0xffff, addr >> 16,
			chunk, (uint16_t)n, FX3_TIMEOUT_MS);
		if (ret != n) {
			sr_err("Write of %d bytes at 0x%08x failed: %s.", n, addr,
			       ret < 0 ? libusb_error_name(ret) : "short write");
			return SR_ERR;
		}
		ret = libusb_control_transfer(hdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
			FX3_REQ_LOAD, addr & 0xffff, addr >> 16,
			back, (uint16_t)n, FX3_TIMEOUT_MS);
		if (ret != n || memcmp(back, chunk, (size_t)n) != 0) {
			sr_err("Verify of %d bytes at 0x%08x failed%s%s.", n, addr,
			       ret < 0 ? ": " : "",
			       ret < 0 ? libusb_error_name(ret) : "");
			return SR_ERR;
		}
		addr += (uint32_t)n;
		data += n;
		len -= (size_t)n;
	}
	return SR_OK;
}

SR_PRIV int fx3_upload_firmware(libusb_device *dev, const char *filename)
{
	struct fx3_section sections[FX3_MAX_SECTIONS];
	struct libusb_device_handle *hdl = NULL;
	gchar *img = NULL;
	gsize len = 0;
	GError *err = NULL;
	uint32_t entry = 0;
	int num = 0, i, r, ret = SR_ERR;

	sr_info("Uploading %s to the FX3 on %d.%d.", filename,
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	if (!g_file_get_contents(filename, &img, &len, &err)) {
		sr_err("Cannot read %s: %s", filename, err ? err->message : "?");
		g_clear_error(&err);
		ds_set_last_error(SR_ERR_FIRMWARE_NOT_EXIST);
		return SR_ERR;
	}
	r = fx3_image_parse((const uint8_t *)img, len, sections,
			    FX3_MAX_SECTIONS, &num, &entry);
	if (r != FX3_IMG_OK) {
		sr_err("%s: %s.", filename, img_error(r));
		goto out;
	}

	if ((r = libusb_open(dev, &hdl)) < 0) {
		sr_err("Cannot open the FX3 boot loader: %s.", libusb_error_name(r));
		hdl = NULL;
		goto out;
	}

	for (i = 0; i < num; i++) {
		sr_dbg("Section %d: %u bytes at 0x%08x.", i, (unsigned)sections[i].len,
		       sections[i].addr);
		if (write_and_verify(hdl, sections[i].addr, sections[i].data,
				     sections[i].len) != SR_OK)
			goto out;
	}

	/* Start the firmware. The FX3 may leave the bus before it answers,
	 * so an error here is not a failure. */
	r = libusb_control_transfer(hdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		FX3_REQ_LOAD, entry & 0xffff, entry >> 16, NULL, 0, FX3_TIMEOUT_MS);
	if (r < 0)
		sr_dbg("Jump to 0x%08x returned %s (expected when the FX3 "
		       "re-enumerates at once).", entry, libusb_error_name(r));
	sr_info("Firmware loaded (%d sections), started at 0x%08x.", num, entry);
	ret = SR_OK;
out:
	if (hdl)
		libusb_close(hdl);
	g_free(img);
	return ret;
}
