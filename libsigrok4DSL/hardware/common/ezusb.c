/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
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
 * Helper functions for the Cypress EZ-USB / FX2 series chips.
 */

#include "../../libsigrok-internal.h"
#include <glib.h>
#include <glib/gstdio.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "../../log.h"

#undef LOG_PREFIX
#define LOG_PREFIX "ezusb: "

SR_PRIV int ezusb_reset(struct libusb_device_handle *hdl, int set_clear)
{
	int ret;
	unsigned char buf[1];

    sr_info("setting CPU reset mode %s...",
		set_clear ? "on" : "off");
	buf[0] = set_clear ? 1 : 0;
	ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR, 0xa0,
                      0xe600, 0x0000, buf, 1, 3000);
	if (ret < 0)
        sr_err("Unable to send control request: %s.",
				libusb_error_name(ret));

	return ret;
}

SR_PRIV int ezusb_install_firmware(libusb_device_handle *hdl,
				   const char *filename)
{
	FILE *fw;
	int offset, chunksize, ret, result;
	unsigned char buf[4096];

    sr_info("Uploading firmware at %s", filename);
    if ((fw = fopen(filename, "rb")) == NULL) {
        sr_err("Unable to open firmware file %s for reading: %s",
		       filename, strerror(errno));
		ds_set_last_error(SR_ERR_FIRMWARE_NOT_EXIST);
        return SR_ERR;
	}

    result = SR_OK;
	offset = 0;

	while (1) {
		chunksize = fread(buf, 1, 4096, fw);
		if (chunksize == EOF){
			sr_err("ezusb_install_firmware(), f-read returns EOF.");
			result = SR_ERR;
			break;			
		}
		if (chunksize == 0)
			break;

		ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
					      LIBUSB_ENDPOINT_OUT, 0xa0, offset,
                          0x0000, buf, chunksize, 3000);
		if (ret < 0) {
            sr_err("Unable to send firmware to device: %s.",
					libusb_error_name(ret));
            result = SR_ERR;
			break;
		}
        sr_info("Uploaded %d bytes", chunksize);
		offset += chunksize;
	}
	fclose(fw);
    sr_info("Firmware upload done");

	return result;
}

SR_PRIV int ezusb_upload_firmware(libusb_device *dev, int configuration,
				  const char *filename)
{
	struct libusb_device_handle *hdl;
	int ret;

    sr_info("uploading firmware to device on %d.%d",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	if ((ret = libusb_open(dev, &hdl)) < 0) {
        sr_err("%s:%d, Failed to open device: %s.", 
				__func__, __LINE__, libusb_error_name(ret));
        return SR_ERR;
	}

/*
 * The libusbx darwin backend is broken: it can report a kernel driver being
 * active, but detaching it always returns an error.
 */
#if !defined(__APPLE__)
	if (libusb_kernel_driver_active(hdl, 0) == 1) {
		if ((ret = libusb_detach_kernel_driver(hdl, 0)) < 0) {
            sr_err("failed to detach kernel driver: %s",
					libusb_error_name(ret));
            return SR_ERR;
		}
	}
#endif

	if ((ret = libusb_set_configuration(hdl, configuration)) < 0) {
        sr_err("Unable to set configuration: %s",
				libusb_error_name(ret));
        return SR_ERR;
	}

	if ((ezusb_reset(hdl, 1)) < 0)
        return SR_ERR;

	if (ezusb_install_firmware(hdl, filename) < 0)
        return SR_ERR;

	if ((ezusb_reset(hdl, 0)) < 0)
        return SR_ERR;

	if (hdl != NULL)
		libusb_close(hdl);

    return SR_OK;
}

/*
 * Cypress FX3: load a firmware image (FX3 SDK "CY" .img) into the RAM of an
 * FX3 in USB boot loader mode and start it. From libsigrok's ezusb.c as
 * used by the cypress-fx3-8 driver; changes for libsigrok4DSL: the image is
 * read from a file path (libsigrok4DSL has no sr_resource_load()), no
 * sr_context parameter, every section is bounds checked against the file
 * size, the device handle is closed on error paths, and results are
 * compared with SR_OK (SR_ERR is +1 here, -1 in libsigrok).
 *
 * Image: 'C' 'Y' bImageCTL bImageType, then sections
 * { u32 length in 32-bit words, u32 address, data }, ended by
 * { u32 0, u32 entry address } and u32 checksum (sum of all data words).
 */

#define FX3_FW_CHUNKSIZE (4 * 1024)
#define FX3_FW_MAXSIZE   (1 << 18)	/* 256 KB */

#define GET_LSW(v)  ((unsigned short)((v) & 0xFFFF))	/* least significant word */
#define GET_MSW(v)  ((unsigned short)((v) >> 16))	/* most significant word */

SR_PRIV int ezusb_fx3_ram_write(libusb_device_handle *hdl, unsigned char *buf,
				unsigned int ramAddress, int len)
{
	int r;
	int index = 0;
	int size;

	while (len > 0) {
		size = MIN(len, FX3_FW_CHUNKSIZE);

		r = libusb_control_transfer(hdl, 0x40, 0xA0, GET_LSW(ramAddress),
					    GET_MSW(ramAddress), &buf[index], size, 100);
		if (r != size) {
			sr_err("Vendor write to FX3 RAM failed at 0x%08x: %s.",
			       ramAddress, r < 0 ? libusb_error_name(r) : "short write");
			return SR_ERR;
		}
		ramAddress += size;
		index += size;
		len -= size;
	}

	return SR_OK;
}

SR_PRIV int ezusb_install_firmware_fx3(libusb_device_handle *hdl,
				       const char *filename)
{
	gchar *contents = NULL;
	unsigned char *firmware;
	gsize filesize = 0;
	GError *err = NULL;
	unsigned int *data_p;
	unsigned int i, checksum, length, address;
	size_t index;
	int r;

	if (!g_file_get_contents(filename, &contents, &filesize, &err)) {
		sr_err("Unable to read FX3 firmware %s: %s", filename,
		       err ? err->message : "?");
		g_clear_error(&err);
		ds_set_last_error(SR_ERR_FIRMWARE_NOT_EXIST);
		return SR_ERR;
	}
	firmware = (unsigned char *)contents;
	if (filesize > FX3_FW_MAXSIZE || filesize < 4 ||
	    firmware[0] != 'C' || firmware[1] != 'Y') {
		sr_err("%s is not an FX3 firmware image.", filename);
		g_free(contents);
		return SR_ERR;
	}

	sr_info("Uploading FX3 firmware '%s'.", filename);

	/* Run through each section of code, and use vendor commands to
	 * download them to RAM. */
	index = 4;
	checksum = 0;
	while (index < filesize) {
		if (filesize - index < 8) {
			sr_err("FX3 firmware image is truncated.");
			g_free(contents);
			return SR_ERR;
		}
		data_p = (unsigned int *)(firmware + index);
		length = data_p[0];
		address = data_p[1];
		if (length != 0) {
			if ((uint64_t)length * 4 > filesize - index - 8) {
				sr_err("FX3 firmware image is truncated.");
				g_free(contents);
				return SR_ERR;
			}
			for (i = 0; i < length; i++)
				checksum += data_p[2 + i];
			r = ezusb_fx3_ram_write(hdl, firmware + index + 8, address,
						(int)(length * 4));
			if (r != SR_OK) {
				sr_err("Failed to download data to FX3 RAM.");
				g_free(contents);
				return SR_ERR;
			}
		} else {
			if (filesize - index < 12 || checksum != data_p[2]) {
				sr_err("Checksum error in the FX3 firmware image.");
				g_free(contents);
				return SR_ERR;
			}
			/* Jump to the entry point. The FX3 may leave the bus
			 * before it answers, so an error is ignored. */
			r = libusb_control_transfer(hdl, 0x40, 0xA0, GET_LSW(address),
						    GET_MSW(address), NULL, 0, 100);
			if (r != 0)
				sr_info("Ignored error in the FX3 start request: %d.", r);
			break;
		}
		index += 8 + (size_t)length * 4;
	}
	g_free(contents);

	sr_info("FX3 firmware upload done.");
	return SR_OK;
}

SR_PRIV int ezusb_upload_firmware_fx3(libusb_device *dev, int configuration,
				      const char *filename)
{
	struct libusb_device_handle *hdl;
	int ret;

	sr_info("Uploading firmware to the FX3 device on %d.%d.",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	if ((ret = libusb_open(dev, &hdl)) < 0) {
		sr_err("Failed to open the FX3 device: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

/*
 * The libusb Darwin backend is broken: it can report a kernel driver being
 * active, but detaching it always returns an error.
 */
#if !defined(__APPLE__)
	if (libusb_kernel_driver_active(hdl, 0) == 1) {
		if ((ret = libusb_detach_kernel_driver(hdl, 0)) < 0) {
			sr_err("Failed to detach the kernel driver: %s.",
			       libusb_error_name(ret));
			libusb_close(hdl);
			return SR_ERR;
		}
	}
#endif

	if ((ret = libusb_set_configuration(hdl, configuration)) < 0) {
		sr_err("Unable to set configuration: %s.", libusb_error_name(ret));
		libusb_close(hdl);
		return SR_ERR;
	}

	/* libsigrok4DSL's SR_ERR is positive (libsigrok's is negative) */
	if (ezusb_install_firmware_fx3(hdl, filename) != SR_OK) {
		libusb_close(hdl);
		return SR_ERR;
	}

	libusb_close(hdl);
	return SR_OK;
}
