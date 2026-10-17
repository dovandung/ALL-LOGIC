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

#ifndef LIBSIGROK_HARDWARE_COMMON_FX3LOAD_H
#define LIBSIGROK_HARDWARE_COMMON_FX3LOAD_H

#include <stdint.h>
#include <stddef.h>

#define FX3_MAX_SECTIONS 64

struct fx3_section {
	uint32_t addr;
	const uint8_t *data;
	size_t len;
};

enum {
	FX3_IMG_OK = 0,
	FX3_IMG_BAD_HEADER = -1,
	FX3_IMG_NOT_EXECUTABLE = -2,
	FX3_IMG_TRUNCATED = -3,
	FX3_IMG_BAD_CHECKSUM = -4,
	FX3_IMG_TOO_MANY = -5,
};

/* Check an FX3 "CY" firmware image and list its sections (pointing into
 * img) and entry address. Returns FX3_IMG_OK or an FX3_IMG_* error. */
SR_PRIV int fx3_image_parse(const uint8_t *img, size_t len,
			    struct fx3_section *sections, int max_sections,
			    int *num_sections, uint32_t *entry);

/* Load an FX3 firmware image file into the RAM of an FX3 in boot loader
 * mode and start it; each chunk is read back and compared. The device then
 * leaves the bus and enumerates again with the firmware's IDs. */
SR_PRIV int fx3_upload_firmware(libusb_device *dev, const char *filename);

#endif
