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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico_serial.h"

static void port_free(gpointer p)
{
	struct pico_port *port = p;

	if (!port)
		return;
	g_free(port->name);
	g_free(port->description);
	g_free(port);
}

void pico_port_list_free(GSList *ports)
{
	g_slist_free_full(ports, port_free);
}

#ifdef _WIN32
/* ===================================================================== */
/* Windows                                                               */
/* ===================================================================== */
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>

struct pico_serial {
	HANDLE h;
};

/* "...VID_2E8A&PID_0009..." or "...vid_2e8a&pid_0009..." */
static void parse_vid_pid(const char *s, uint16_t *vid, uint16_t *pid)
{
	const char *v, *p;

	if (!s)
		return;
	v = strstr(s, "VID_");
	if (!v)
		v = strstr(s, "vid_");
	p = strstr(s, "PID_");
	if (!p)
		p = strstr(s, "pid_");
	if (v && p) {
		*vid = (uint16_t)strtoul(v + 4, NULL, 16);
		*pid = (uint16_t)strtoul(p + 4, NULL, 16);
	}
}

GSList *pico_port_list(void)
{
	GSList *ports = NULL;
	HDEVINFO devs;
	SP_DEVINFO_DATA info;
	DWORD i;

	devs = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
	if (devs == INVALID_HANDLE_VALUE)
		return NULL;

	info.cbSize = sizeof(info);
	for (i = 0; SetupDiEnumDeviceInfo(devs, i, &info); i++) {
		char hwid[512] = "", fname[256] = "", pname[64] = "";
		DWORD type = 0, len = sizeof(pname);
		struct pico_port *port;
		HKEY key;

		SetupDiGetDeviceRegistryPropertyA(devs, &info, SPDRP_HARDWAREID,
			NULL, (PBYTE)hwid, sizeof(hwid) - 1, NULL);
		SetupDiGetDeviceRegistryPropertyA(devs, &info, SPDRP_FRIENDLYNAME,
			NULL, (PBYTE)fname, sizeof(fname) - 1, NULL);

		key = SetupDiOpenDevRegKey(devs, &info, DICS_FLAG_GLOBAL, 0,
					   DIREG_DEV, KEY_READ);
		if (key == INVALID_HANDLE_VALUE)
			continue;
		if (RegQueryValueExA(key, "PortName", NULL, &type, (LPBYTE)pname,
				     &len) != ERROR_SUCCESS || type != REG_SZ) {
			RegCloseKey(key);
			continue;
		}
		RegCloseKey(key);
		pname[sizeof(pname) - 1] = 0;
		if (strncmp(pname, "COM", 3) != 0)
			continue;	/* LPT ports are in this class too */

		port = g_malloc0(sizeof(*port));
		port->name = g_strdup(pname);
		port->description = fname[0] ? g_strdup(fname) : NULL;
		parse_vid_pid(hwid, &port->vid, &port->pid);
		ports = g_slist_append(ports, port);
	}
	SetupDiDestroyDeviceInfoList(devs);
	return ports;
}

struct pico_serial *pico_serial_open(const char *port)
{
	struct pico_serial *s;
	char path[64];
	DCB dcb;
	COMMTIMEOUTS to;
	HANDLE h;

	/* "\\.\COMn" works for every port number (COM10 and up need it) */
	snprintf(path, sizeof(path), "\\\\.\\%s", port);
	h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
			OPEN_EXISTING, 0, NULL);
	if (h == INVALID_HANDLE_VALUE)
		return NULL;

	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(h, &dcb)) {
		CloseHandle(h);
		return NULL;
	}
	dcb.BaudRate = 115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	if (!SetCommState(h, &dcb)) {
		CloseHandle(h);
		return NULL;
	}
	memset(&to, 0, sizeof(to));
	SetCommTimeouts(h, &to);
	EscapeCommFunction(h, SETDTR);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

	s = g_malloc0(sizeof(*s));
	s->h = h;
	return s;
}

void pico_serial_close(struct pico_serial *s)
{
	if (!s)
		return;
	CloseHandle(s->h);
	g_free(s);
}

int pico_serial_write(struct pico_serial *s, const void *buf, size_t len,
		      int timeout_ms)
{
	COMMTIMEOUTS to;
	DWORD n = 0;

	memset(&to, 0, sizeof(to));
	to.WriteTotalTimeoutConstant = (DWORD)timeout_ms;
	SetCommTimeouts(s->h, &to);
	if (!WriteFile(s->h, buf, (DWORD)len, &n, NULL))
		return -1;
	return (int)n;
}

int pico_serial_read(struct pico_serial *s, void *buf, size_t len,
		     int timeout_ms)
{
	COMMTIMEOUTS to;
	DWORD n = 0;

	/* return as soon as anything arrived, or after timeout_ms */
	memset(&to, 0, sizeof(to));
	to.ReadIntervalTimeout = MAXDWORD;
	to.ReadTotalTimeoutMultiplier = MAXDWORD;
	to.ReadTotalTimeoutConstant = (DWORD)(timeout_ms > 0 ? timeout_ms : 1);
	SetCommTimeouts(s->h, &to);
	if (!ReadFile(s->h, buf, (DWORD)len, &n, NULL))
		return -1;
	return (int)n;
}

#else
/* ===================================================================== */
/* POSIX (Linux, macOS)                                                  */
/* ===================================================================== */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

struct pico_serial {
	int fd;
};

/* Linux: USB IDs of /dev/ttyACMn from sysfs */
static void sysfs_ids(const char *dev, uint16_t *vid, uint16_t *pid)
{
	const char *base = strrchr(dev, '/');
	char path[256], val[16];
	FILE *f;

	if (!base)
		return;
	base++;
	snprintf(path, sizeof(path), "/sys/class/tty/%s/device/../idVendor", base);
	f = fopen(path, "r");
	if (!f)
		return;
	if (fgets(val, sizeof(val), f))
		*vid = (uint16_t)strtoul(val, NULL, 16);
	fclose(f);
	snprintf(path, sizeof(path), "/sys/class/tty/%s/device/../idProduct", base);
	f = fopen(path, "r");
	if (!f)
		return;
	if (fgets(val, sizeof(val), f))
		*pid = (uint16_t)strtoul(val, NULL, 16);
	fclose(f);
}

GSList *pico_port_list(void)
{
	static const char *patterns[] = {
		"/dev/ttyACM*", "/dev/cu.usbmodem*",
	};
	GSList *ports = NULL;
	unsigned int k;
	size_t i;

	for (k = 0; k < G_N_ELEMENTS(patterns); k++) {
		glob_t g;
		if (glob(patterns[k], 0, NULL, &g) != 0)
			continue;
		for (i = 0; i < g.gl_pathc; i++) {
			struct pico_port *port = g_malloc0(sizeof(*port));
			port->name = g_strdup(g.gl_pathv[i]);
			sysfs_ids(port->name, &port->vid, &port->pid);
			ports = g_slist_append(ports, port);
		}
		globfree(&g);
	}
	return ports;
}

struct pico_serial *pico_serial_open(const char *port)
{
	struct pico_serial *s;
	struct termios tio;
	int fd, bits = TIOCM_DTR;

	fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
		return NULL;
	if (tcgetattr(fd, &tio) != 0) {
		close(fd);
		return NULL;
	}
	cfmakeraw(&tio);
	cfsetispeed(&tio, B115200);
	cfsetospeed(&tio, B115200);
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~(CRTSCTS | HUPCL);
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	if (tcsetattr(fd, TCSANOW, &tio) != 0) {
		close(fd);
		return NULL;
	}
	ioctl(fd, TIOCMBIS, &bits);
	bits = TIOCM_RTS;
	ioctl(fd, TIOCMBIC, &bits);
	tcflush(fd, TCIOFLUSH);

	s = g_malloc0(sizeof(*s));
	s->fd = fd;
	return s;
}

void pico_serial_close(struct pico_serial *s)
{
	if (!s)
		return;
	close(s->fd);
	g_free(s);
}

int pico_serial_write(struct pico_serial *s, const void *buf, size_t len,
		      int timeout_ms)
{
	const uint8_t *p = buf;
	size_t done = 0;
	gint64 deadline = g_get_monotonic_time() + (gint64)timeout_ms * 1000;

	while (done < len) {
		struct pollfd pfd = { s->fd, POLLOUT, 0 };
		gint64 left = (deadline - g_get_monotonic_time()) / 1000;
		ssize_t n;

		if (left <= 0 || poll(&pfd, 1, (int)left) <= 0)
			break;
		n = write(s->fd, p + done, len - done);
		if (n < 0) {
			if (errno == EAGAIN || errno == EINTR)
				continue;
			return -1;
		}
		done += (size_t)n;
	}
	return (int)done;
}

int pico_serial_read(struct pico_serial *s, void *buf, size_t len,
		     int timeout_ms)
{
	struct pollfd pfd = { s->fd, POLLIN, 0 };
	ssize_t n;
	int r;

	r = poll(&pfd, 1, timeout_ms);
	if (r < 0)
		return errno == EINTR ? 0 : -1;
	if (r == 0)
		return 0;
	n = read(s->fd, buf, len);
	if (n < 0)
		return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
	return (int)n;
}
#endif

/* ===================================================================== */
/* la8 firmware commands                                                 */
/* ===================================================================== */

int pico_la8_command(const char *port, const char *cmd, char *reply,
		     size_t reply_size)
{
	struct pico_serial *s;
	char line[64];
	size_t len = 0;
	gint64 deadline;
	int ret = PICO_LA8_NO_ANSWER;

	if (!port || reply_size < 2)
		return PICO_LA8_IO_ERROR;
	reply[0] = 0;

	s = pico_serial_open(port);
	if (!s)
		return PICO_LA8_OPEN_ERROR;

	/* drop anything printed earlier */
	while (pico_serial_read(s, reply, reply_size - 1, 100) > 0)
		;
	reply[0] = 0;

	snprintf(line, sizeof(line), "%s\r", cmd);
	if (pico_serial_write(s, line, strlen(line), 500) < (int)strlen(line)) {
		pico_serial_close(s);
		return PICO_LA8_IO_ERROR;
	}

	/* The status answer ends with the "commands:" help line; a refused
	 * divider is a single "divider must be ..." line, a refused channel
	 * count (la48 "ch N") a single "channels must be ..." line. */
	deadline = g_get_monotonic_time() + 2 * G_USEC_PER_SEC;
	while (g_get_monotonic_time() < deadline && len < reply_size - 1) {
		int n = pico_serial_read(s, reply + len, reply_size - 1 - len, 50);
		const char *refused;
		if (n < 0) {
			ret = PICO_LA8_IO_ERROR;
			break;
		}
		len += (size_t)n;
		reply[len] = 0;
		if (strstr(reply, "commands:")) {
			ret = (strstr(reply, "divider must be") ||
			       strstr(reply, "channels must be") ||
			       strstr(reply, "unknown command")) ?
				PICO_LA8_REFUSED : PICO_LA8_OK;
			break;
		}
		refused = strstr(reply, "divider must be");
		if (!refused)
			refused = strstr(reply, "channels must be");
		if (refused && strchr(refused, '\n')) {
			ret = PICO_LA8_REFUSED;
			break;
		}
	}
	reply[len] = 0;
	pico_serial_close(s);
	return ret;
}

int pico_la8_parse_status(const char *reply, unsigned int *sys_mhz,
			  unsigned int *div, unsigned int *min_div)
{
	const char *p;

	p = strstr(reply, "(sys ");
	if (!p || sscanf(p, "(sys %u MHz / %u)", sys_mhz, div) != 2 ||
	    !*sys_mhz || !*div)
		return -1;
	p = strstr(reply, "minimum divider ");
	if (!p || sscanf(p, "minimum divider %u", min_div) != 1 || !*min_div)
		return -1;
	return 0;
}

int pico_la8_parse_la48(const char *reply, unsigned int *min_div8,
			unsigned int *min_div4)
{
	const char *p, *p8, *p4;

	p = strstr(reply, "la48 switchable:");
	if (!p)
		return -1;
	p8 = strstr(p, "ch 8 (");
	p4 = strstr(p, "ch 4 (");
	if (!p8 || !p4)
		return -1;
	p8 = strstr(p8, "minimum divider ");
	p4 = strstr(p4, "minimum divider ");
	if (!p8 || !p4 ||
	    sscanf(p8, "minimum divider %u", min_div8) != 1 ||
	    sscanf(p4, "minimum divider %u", min_div4) != 1 ||
	    !*min_div8 || !*min_div4)
		return -1;
	return 0;
}
