/*
 * Minimal serial port access for the hsdaoh-la driver (ALL LOGIC).
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
 * libsigrok4DSL has no serial layer, so this is just enough to talk to the
 * la8 firmware on the Pico's USB CDC port: list ports (with USB IDs where
 * the OS gives them), open with DTR set (the firmware only answers then),
 * write, and read with a timeout.
 */

#ifndef LIBSIGROK_HARDWARE_HSDAOH_LA_PICO_SERIAL_H
#define LIBSIGROK_HARDWARE_HSDAOH_LA_PICO_SERIAL_H

#include <stdint.h>
#include <stddef.h>
#include <glib.h>

struct pico_port {
	char *name;		/* "COM5", "/dev/ttyACM0" */
	char *description;	/* may be NULL */
	uint16_t vid, pid;	/* 0 if unknown */
};

/* All serial ports, as a list of struct pico_port *. Free with
 * pico_port_list_free(). */
GSList *pico_port_list(void);
void pico_port_list_free(GSList *ports);

struct pico_serial;

/* Open `port` at 115200 8N1, DTR on, RTS off. NULL on error. */
struct pico_serial *pico_serial_open(const char *port);
void pico_serial_close(struct pico_serial *s);
/* Returns bytes written, or -1. */
int pico_serial_write(struct pico_serial *s, const void *buf, size_t len,
		      int timeout_ms);
/* Returns bytes read (0 on timeout), or -1. */
int pico_serial_read(struct pico_serial *s, void *buf, size_t len,
		     int timeout_ms);


/* ---- la8 / la4 / la48 firmware protocol ---- */
enum {
	PICO_LA8_OK = 0,
	PICO_LA8_REFUSED = 1,		/* "divider must be ...", "channels must
					 * be ...", "unknown command" */
	PICO_LA8_NO_ANSWER = -1,	/* nothing (complete) within 2 s */
	PICO_LA8_OPEN_ERROR = -2,	/* port busy or gone */
	PICO_LA8_IO_ERROR = -3,
};
/* Send one command line to the la8 firmware on `port` and collect its
 * answer (NUL-terminated, possibly partial on error) in `reply`. */
int pico_la8_command(const char *port, const char *cmd, char *reply,
		     size_t reply_size);
/* Parse "... (sys 280 MHz / 2), minimum divider 2, ...". 0 or -1. */
int pico_la8_parse_status(const char *reply, unsigned int *sys_mhz,
			  unsigned int *div, unsigned int *min_div);
/* Parse the la48 line "la48 switchable: ch 8 (GP0..GP7, minimum divider 2)
 * or ch 4 (GP0..GP3, minimum divider 1)". 0, or -1 if not la48. */
int pico_la8_parse_la48(const char *reply, unsigned int *min_div8,
			unsigned int *min_div4);

#endif
