/*

   garmini - download track log from Garmin GPSs
   Copyright (C) 2007  Tom Payne

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/* TODO splitting by segment */
/* TODO splitting by flight acceptance */
/* TODO factor out garmin interface code */

#include <ctype.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#if __BYTE_ORDER != __LITTLE_ENDIAN
#error Only little-endian machines are currently supported
#endif

#define GARMINI_TIMEOUT (10 * 1000)

#define DIE(syscall, _errno) die(__FILE__, __LINE__, __FUNCTION__, (syscall), (_errno))

enum {
	DLE = 16,
	ETX =  3
};

enum {
	Pid_Ack_Byte =  6,
	Pid_Nak_Byte = 21
};

/* L000 */

enum {
	Pid_Protocol_Array   = 253,
	Pid_Product_Rqst     = 254,
	Pid_Product_Data     = 255,
	Pid_Ext_Product_Data = 248
};

/* L001 */

enum {
	Pid_Command_Data = 10,
	Pid_Xfer_Cmplt   = 12,
	Pid_Records      = 27,
	Pid_Trk_Data     = 34,
	Pid_Trk_Hdr      = 99
};

/* A001 */

enum {
	Tag_Phys_Prot_Id = 'P',
	Tag_Link_Prot_Id = 'L',
	Tag_Appl_Prot_Id = 'A',
	Tag_Data_Prot_Id = 'D'
};

/* A010 */

enum {
	Cmnd_Abort_Transfer = 0,
	Cmnd_Transfer_Trk   = 6,
	Cmnd_Turn_Off_Pwr   = 8
};

typedef struct {
	uint16_t product_id;
	int16_t software_version;
	char product_description[1];
} Product_Data_Type;

typedef struct {
	uint8_t tag;
	uint16_t data __attribute__ ((packed));
} Protocol_Data_Type;

/* track point types */

typedef struct {
	int32_t lat;
	int32_t lon;
} position_t;

typedef struct {
	position_t posn;
	uint32_t time;
	uint8_t new_trk;
} D300_Trk_Point_Type;

typedef struct {
	position_t posn;
	uint32_t time;
	float alt;
	float dpth;
	uint8_t new_trk;
} D301_Trk_Point_Type;

typedef struct {
	position_t posn;
	uint32_t time;
	float alt;
	float dpth;
	float temp;
	uint8_t new_trk;
} D302_Trk_Point_Type;

typedef struct {
	position_t posn;
	uint32_t time;
	float alt;
	uint8_t heart_rate;
} D303_Trk_Point_Type;

typedef struct {
	position_t posn;
	uint32_t time;
	float alt;
	float distance;
	uint8_t heart_rate;
	uint8_t cadence;
	uint8_t sensor;
} D304_Trk_Point_Type;

/* track header types */

typedef struct {
	uint8_t dspl;
	uint8_t color;
	char trk_ident[51];
} D310_Trk_Hdr_Type;

typedef struct {
	uint16_t index;
} D311_Trk_Hdr_Type;

typedef struct {
	uint8_t dspl;
	uint8_t color;
	char trk_ident[51];
} D312_Trk_Hdr_Type;

typedef struct {
	int id;
	int size;
	unsigned char data[255];
} garmin_packet_t;

typedef struct {
	const char *device;
	int fd;
	FILE *logfile;
	Product_Data_Type *product_data;
	int nprotocols;
	Protocol_Data_Type *protocols;
	unsigned char *next;
	unsigned char *end;
	unsigned char buf[1024];
} garmin_t;

typedef struct {
	time_t time;
	position_t posn;
	float alt;
	char validity;
} garmin_trk_point_t;

int _sc_clk_tck = -1;
const char *program_name = 0;
const char *device = 0;
FILE *logfile = 0;
const char *directory = 0;
int power_off = 0;
const char *manufacturer = "XXX";
int serial_number = 0;
int barometric_altimeter = -1;
const char *glider_id = 0;
const char *glider_type = 0;
const char *pilot = 0;
const char *competition_class = 0;
const char *competition_id = 0;
int quiet = 0;
int minimum_trk_points = 0; /* FIXME */
int minimum_duration = 0; /* FIXME */

void error(const char *message, ...)
{
	fprintf(stderr, "%s: ", program_name);
	va_list ap;
	va_start(ap, message);
	vfprintf(stderr, message, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

void die(const char *file, int line, const char *function, const char *message, int _errno)
{
	if (_errno)
		error("%s:%d: %s: %s: %s", file, line, function, message, strerror(_errno));
	else
		error("%s:%d: %s: %s", file, line, function, message);
}

void *alloc(int size)
{
	void *p = malloc(size);
	if (!p)
		DIE("malloc", errno);
	memset(p, 0, size);
	return p;
}

static void print_string(FILE *file, const char *s, int len)
{
	const char *end = s + (len < 0 ? (int) strlen(s) : len);
	const char *p;
	for (p = s; p < end; ++p) {
		const char *format;
		switch (*p) {
			case '\a':
				format = "\\a";
				break;
			case '\b':
				format = "\\b";
				break;
			case '\f':
				format = "\\f";
				break;
			case '\n':
				format = "\\n";
				break;
			case '\r':
				format = "\\r";
				break;
			case '\t':
				format = "\\t";
				break;
			case '\v':
				format = "\\v";
				break;
			case '\"':
				format = "\\\"";
				break;
			default:
				format = isprint(*p) ? "%c" : "\\x%02x";
				break;
		}
		fprintf(file, format, ((unsigned) *p) & 0xff);
	}
}

static void garmin_read(garmin_t *garmin)
{
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(garmin->fd, &readfds);
	int rc;
	do {
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = GARMINI_TIMEOUT;
		rc = select(garmin->fd + 1, &readfds, 0, 0, &timeout);
	} while (rc == -1 && errno == EINTR);
	if (rc == -1)
		DIE("select", errno);
	if (FD_ISSET(garmin->fd, &readfds)) {
		int n;
		do {
			n = read(garmin->fd, garmin->buf, sizeof garmin->buf);
		} while (n == -1 && errno == EINTR);
		if (n == -1)
			DIE("read", errno);
		else if (n == 0)
			DIE("read", 0);
		garmin->next = garmin->buf;
		garmin->end = garmin->buf + n;
	} else {
		garmin->next = garmin->end = garmin->buf;
	}
}

int garmin_select(garmin_t *garmin)
{
	if (garmin->next != garmin->end)
		return 1;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(garmin->fd, &readfds);
	int rc;
	do {
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = GARMINI_TIMEOUT;
		rc = select(garmin->fd + 1, &readfds, 0, 0, &timeout);
	} while (rc == -1 && errno == EINTR);
	return rc > 0 && FD_ISSET(garmin->fd, &readfds);
}

static int garmin_getc(garmin_t *garmin)
{
	if (garmin->next == garmin->end)
		garmin_read(garmin);
	if (garmin->next == garmin->end)
		return EOF;
	return *garmin->next++;
}

static int garmin_getc_dle(garmin_t *garmin)
{
	int c = garmin_getc(garmin);
	if (c == DLE && garmin_getc(garmin) != DLE)
		error("%s: expected DLE", garmin->device);
	return c;
}

static int garmin_peekc(garmin_t *garmin)
{
	if (garmin->next == garmin->end)
		garmin_read(garmin);
	if (garmin->next == garmin->end)
		return EOF;
	return *garmin->next;
}

static void garmin_log_packet(garmin_t *garmin, const garmin_packet_t *packet, int direction)
{
	if (!garmin->logfile)
		return;
	fprintf(garmin->logfile, "%c { %3d, \"", direction, packet->id);
	print_string(garmin->logfile, (char *) packet->data, packet->size);
	fprintf(garmin->logfile, "\" }\n");
}

int garmin_read_packet(garmin_t *garmin, garmin_packet_t *packet)
{
	memset(packet, 0, sizeof packet);
	int c = garmin_getc(garmin);
	if (c == EOF)
		return EOF;
	c = garmin_getc_dle(garmin);
	if (c == EOF)
		goto eof;
	int checksum = packet->id = c;
	c = garmin_getc_dle(garmin);
	if (c == EOF)
		goto eof;
	checksum += packet->size = c;
	int i;
	for (i = 0; i < packet->size; ++i) {
		c = garmin_getc_dle(garmin);
		if (c == EOF)
			goto eof;
		checksum += packet->data[i] = c;
	}
	checksum = (~checksum + 1) & 0xff;
	c = garmin_getc_dle(garmin);
	if (c == EOF)
		goto eof;
	if (c != checksum)
		error("%s: checksum failed", garmin->device);
	c = garmin_getc(garmin);
	if (c == EOF)
		goto eof;
	if (c != DLE)
		error("%s: expected DLE", garmin->device);
	c = garmin_getc(garmin);
	if (c == EOF)
		goto eof;
	if (c != ETX)
		error("%s: expected ETX", garmin->device);
	garmin_log_packet(garmin, packet, '<');
	return packet->id;
eof:
	error("%s: incomplete packet", garmin->device);
	return EOF;
}

void garmin_write_packet(garmin_t *garmin, garmin_packet_t *packet)
{
	garmin_log_packet(garmin, packet, '>');
	unsigned char buf[1024];
	unsigned char *p = buf;
	*p++ = DLE;
	unsigned char checksum = *p++ = packet->id;
	if (packet->id == DLE)
		*p++ = DLE;
	checksum += *p++ = packet->size;
	if (packet->size == DLE)
		*p++ = DLE;
	int i;
	for (i = 0; i < packet->size; ++i) {
		checksum += *p++ = packet->data[i];
		if (packet->data[i] == DLE)
			*p++ = DLE;
	}
	checksum = ~checksum + 1;
	*p++ = checksum;
	if (checksum == DLE)
		*p++ = DLE;
	*p++ = DLE;
	*p++ = ETX;
	int rc;
	do {
		rc = write(garmin->fd, buf, p - buf);
	} while (rc == -1 && errno == EINTR);
	if (rc == -1)
		DIE("write", errno);
	if (rc != p - buf)
		error("%s: short write", garmin->device);
}

int garmin_read_packet_ack(garmin_t *garmin, garmin_packet_t *packet)
{
	if (garmin_read_packet(garmin, packet) == EOF)
		return EOF;
	garmin_packet_t ack;
	ack.id = Pid_Ack_Byte;
	ack.size = 2;
	*((uint16_t *) ack.data) = packet->id;
	garmin_write_packet(garmin, &ack);
	return packet->id;
}

int garmin_expect_packet_ack(garmin_t *garmin, garmin_packet_t *packet, int id)
{
	if (garmin_read_packet_ack(garmin, packet) != id)
		error("%s: unexpected packet", garmin->device);
	return packet->id;
}

void garmin_write_packet_ack(garmin_t *garmin, garmin_packet_t *packet)
{
	garmin_write_packet(garmin, packet);
	garmin_packet_t ack;
	garmin_read_packet(garmin, &ack);
	if (ack.id != Pid_Ack_Byte)
		error("%s: expected ack packet", garmin->device);
	if (ack.size < 1)
		error("%s: ack packet too short", garmin->device);
	else if ((ack.size == 1 && *(uint8_t *) ack.data != packet->id) || (ack.size == 2 && *(uint16_t *) ack.data != packet->id))
		error("%s: ack to wrong packet!", garmin->device);
}

Protocol_Data_Type *garmin_grep_protocol(garmin_t *garmin, int tag, int data)
{
	int i;
	for (i = 0; i < garmin->nprotocols; ++i)
		if (garmin->protocols[i].tag == tag && garmin->protocols[i].data == data)
			return garmin->protocols + i;
	return 0;
}

garmin_t *garmin_new(const char *device, FILE *logfile)
{
	garmin_t *garmin = alloc(sizeof(garmin_t));
	garmin->device = device;
	garmin->fd = open(garmin->device, O_NOCTTY | O_RDWR);
	if (garmin->fd == -1)
		error("open: %s: %s", garmin->device, strerror(errno));
	if (tcflush(garmin->fd, TCIOFLUSH) == -1)
		error("tcflush: %s: %s", garmin->device, strerror(errno));
	struct termios termios;
	memset(&termios, 0, sizeof termios);
	termios.c_iflag = IGNPAR;
	termios.c_cflag = CLOCAL | CREAD | CS8;
	cfsetispeed(&termios, B9600);
	cfsetospeed(&termios, B9600);
	if (tcsetattr(garmin->fd, TCSANOW, &termios) == -1)
		error("tcsetattr: %s: %s", garmin->device, strerror(errno));
	garmin->logfile = logfile;
	garmin_packet_t packet;
	packet.id = Pid_Product_Rqst;
	packet.size = 0;
	garmin_write_packet_ack(garmin, &packet);
	garmin_expect_packet_ack(garmin, &packet, Pid_Product_Data);
	garmin->product_data = alloc(packet.size + 1);
	memcpy(garmin->product_data, packet.data, packet.size);
	((char *) garmin->product_data)[packet.size] = 0;
	garmin_read_packet_ack(garmin, &packet);
	if (packet.id == Pid_Ext_Product_Data) {
		garmin_read_packet_ack(garmin, &packet);
	} else if (packet.id != EOF) {
		error("%s: unexpected packet %d", garmin->device, packet.id);
	}
	if (packet.id == Pid_Protocol_Array) {
		garmin->nprotocols = packet.size / sizeof(Protocol_Data_Type);
		garmin->protocols = alloc(packet.size);
		memcpy(garmin->protocols, packet.data, packet.size);
		garmin_read_packet_ack(garmin, &packet);
	} else if (packet.id != EOF) {
		error("%s: unexpected packet %d", garmin->device, packet.id);
	}
	if (!garmin_grep_protocol(garmin, Tag_Link_Prot_Id, 1))
		error("%s: device does not support Link Protocol L001", garmin->device);
	if (!garmin_grep_protocol(garmin, Tag_Appl_Prot_Id, 10))
		error("%s: device does not support Device Command Protocol A010", garmin->device);
	return garmin;
}

int garmin_has_barometric_altimeter(garmin_t *garmin)
{
	const char *p = garmin->product_data->product_description;
	while (*p && !isdigit(*p))
		++p;
	while (*p && isdigit(*p))
		++p;
	while (*p && !isspace(*p)) {
		if (*p == 'S' || *p == 's')
			return 1;
		++p;
	}
	return 0;
}

void garmin_delete(garmin_t *garmin)
{
	if (garmin) {
		if (close(garmin->fd) == -1)
			DIE("close", errno);
		free(garmin->product_data);
		free(garmin->protocols);
		free(garmin);
	}
}

void garmin_each(garmin_t *garmin, int command, void (*callback)(void *, int, int, const garmin_packet_t *), void *data)
{
	garmin_packet_t packet;
	packet.id = Pid_Command_Data;
	packet.size = 2;
	*((uint16_t *) packet.data) = command;
	garmin_write_packet_ack(garmin, &packet);
	garmin_expect_packet_ack(garmin, &packet, Pid_Records);
	int records = *((uint16_t *) packet.data);
	int i;
	for (i = 0; i < records; ++i) {
		garmin_read_packet_ack(garmin, &packet);
		callback(data, i, records, &packet);
	}
	garmin_expect_packet_ack(garmin, &packet, Pid_Xfer_Cmplt);
}

void garmin_turn_off_pwr(garmin_t *garmin)
{
	garmin_packet_t packet;
	packet.id = Pid_Command_Data;
	packet.size = 2;
	*((uint16_t *) packet.data) = Cmnd_Turn_Off_Pwr;
	garmin_write_packet(garmin, &packet);
}

typedef struct {
	garmin_t *garmin;
	Protocol_Data_Type trk_hdr;
	Protocol_Data_Type trk_data;
	clock_t clock;
	int remaining_sec;
	int percentage;
	void (*callback)(void *, const garmin_trk_point_t *);
	void *data;
} garmin_transfer_trk_data_t;

static void garmin_transfer_trk_callback(void *data, int i, int records, const garmin_packet_t *packet)
{
	garmin_transfer_trk_data_t *transfer_trk_data = data;
	if (!quiet) {
		struct tms tms;
		clock_t clock = times(&tms);
		if (clock == -1)
			DIE("times", errno);
		int remaining_sec = (records - i - 1) * (clock - transfer_trk_data->clock) / (_sc_clk_tck * (i + 1));
		if (remaining_sec < 1)
			remaining_sec = 1;
		int percentage = 100 * i / records;
		if (remaining_sec != transfer_trk_data->remaining_sec || percentage != transfer_trk_data->percentage) {
			fprintf(stderr, "%3d%% (ETA %2d:%02d)\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", 100 * i / records, remaining_sec / 60, remaining_sec % 60);
			transfer_trk_data->remaining_sec = remaining_sec;
			transfer_trk_data->percentage = percentage;
		}
	}
	switch (packet->id) {
		case Pid_Trk_Data:
			{
				garmin_trk_point_t trk_point;
				memset(&trk_point, 0, sizeof trk_point);
				switch (transfer_trk_data->trk_data.data) {
					case 300:
						{
							D300_Trk_Point_Type *d300_trk_point = (D300_Trk_Point_Type *) packet->data;
							trk_point.posn = d300_trk_point->posn;
							trk_point.time = d300_trk_point->time;
							trk_point.alt = 0;
							trk_point.validity = 'V';
						}
						break;
					case 301:
						{
							D301_Trk_Point_Type *d301_trk_point = (D301_Trk_Point_Type *) packet->data;
							trk_point.posn = d301_trk_point->posn;
							trk_point.time = d301_trk_point->time;
							trk_point.alt = d301_trk_point->alt;
							trk_point.validity = 'A';
						}
						break;
					case 302:
						{
							D302_Trk_Point_Type *d302_trk_point = (D302_Trk_Point_Type *) packet->data;
							trk_point.posn = d302_trk_point->posn;
							trk_point.time = d302_trk_point->time;
							trk_point.alt = d302_trk_point->alt;
							trk_point.validity = 'A';
						}
						break;
					case 303:
						{
							D303_Trk_Point_Type *d303_trk_point = (D303_Trk_Point_Type *) packet->data;
							trk_point.posn = d303_trk_point->posn;
							trk_point.time = d303_trk_point->time;
							trk_point.alt = d303_trk_point->alt;
							trk_point.validity = 'A';
						}
						break;
					case 304:
						{
							D304_Trk_Point_Type *d304_trk_point = (D304_Trk_Point_Type *) packet->data;
							trk_point.posn = d304_trk_point->posn;
							trk_point.time = d304_trk_point->time;
							trk_point.alt = d304_trk_point->alt;
							trk_point.validity = 'A';
						}
						break;
					default:
						abort();
						break;
				}
				transfer_trk_data->callback(transfer_trk_data->data, &trk_point);
				break;
			}
		case Pid_Trk_Hdr:
			break;
	}
}

void garmin_transfer_trk(garmin_t *garmin, void (*callback)(void *, const garmin_trk_point_t *), void *data)
{
	if (_sc_clk_tck == -1) {
		_sc_clk_tck = sysconf(_SC_CLK_TCK);
		if (_sc_clk_tck == -1)
			DIE("sysconf", errno);
	}
	garmin_transfer_trk_data_t transfer_trk_data;
	memset(&transfer_trk_data, 0, sizeof transfer_trk_data);
	transfer_trk_data.garmin = garmin;
	Protocol_Data_Type *protocol_data = garmin->protocols;
	Protocol_Data_Type *protocol_data_end = protocol_data + garmin->nprotocols;
	while (protocol_data < protocol_data_end) {
		switch (protocol_data->tag) {
			case Tag_Appl_Prot_Id:
				switch (protocol_data->data) {
					case 300:
						++protocol_data;
						if (protocol_data == protocol_data_end)
							goto _error;
						transfer_trk_data.trk_data = *protocol_data++;
						break;
					case 301: case 302:
						++protocol_data;
						if (protocol_data == protocol_data_end)
							goto _error;
						transfer_trk_data.trk_hdr = *protocol_data++;
						if (protocol_data == protocol_data_end)
							goto _error;
						transfer_trk_data.trk_data = *protocol_data++;
						break;
					default:
						++protocol_data;
						break;
				}
				break;
			default:
				++protocol_data;
				break;
		}
	}
	if (!transfer_trk_data.trk_data.tag) {
		static const int supported_product_ids[] = { 13, 18, 22, 23, 24, 25, 29, 31, 35, 36, 39, 41, 42, 44, 45, 47, 48, 49, 50, 53, 55, 56, 59, 61, 62, 71, 72, 73, 74, 76, 77, 87, 88, 95, 96, 97, 100, 105, 106, 112 };
		unsigned i;
		for (i = 0; i < sizeof supported_product_ids / sizeof supported_product_ids[0]; ++i) {
			if (garmin->product_data->product_id == supported_product_ids[i]) {
				transfer_trk_data.trk_data.tag = Tag_Data_Prot_Id;
				transfer_trk_data.trk_data.data = 300;
				break;
			}
		}
	}
	if (!transfer_trk_data.trk_data.tag)
		goto _error;
	if (transfer_trk_data.trk_data.tag != Tag_Data_Prot_Id)
		goto _error;
	switch (transfer_trk_data.trk_data.data) {
		case 300:
		case 301:
		case 302:
		case 303:
		case 304:
			break;
		default:
			goto _error;
	}
	if (transfer_trk_data.trk_hdr.tag) {
		if (transfer_trk_data.trk_hdr.tag != Tag_Data_Prot_Id)
			goto _error;
		switch (transfer_trk_data.trk_hdr.data) {
			case 310:
			case 311:
			case 312:
				break;
			default:
				goto _error;
		}
	}
	if (!quiet) {
		struct tms tms;
		transfer_trk_data.clock = times(&tms);
		if (transfer_trk_data.clock == -1)
			DIE("times", errno);
		transfer_trk_data.remaining_sec = -1;
		transfer_trk_data.percentage = -1;
		fprintf(stderr, "Downloading track log: ");
	}
	transfer_trk_data.callback = callback;
	transfer_trk_data.data = data;
	garmin_each(garmin, Cmnd_Transfer_Trk, garmin_transfer_trk_callback, &transfer_trk_data);
	if (!quiet)
		fprintf(stderr, "100%%            \n");
	return;
_error:
	error("%s: unsupported track transfer protocol", garmin->device);
}

typedef struct {
	int capacity;
	garmin_trk_point_t *begin;
	garmin_trk_point_t *end;
} garmini_track_t;

garmini_track_t *garmini_track_new(int capacity, garmin_trk_point_t *begin, garmin_trk_point_t *end)
{
	garmini_track_t *track = alloc(sizeof(garmini_track_t));
	track->capacity = capacity;
	track->begin = begin;
	track->end = end;
	if (track->capacity) {
		track->begin = alloc(track->capacity * sizeof(garmin_trk_point_t));
		track->end = track->begin;
	}
	return track;
}

void garmini_track_delete(garmini_track_t *track)
{
	if (track) {
		if (track->capacity)
			free(track->begin);
		free(track);
	}
}

void garmini_track_push(garmini_track_t *track, const garmin_trk_point_t *trk_point)
{
	if (track->end - track->begin == track->capacity) {
		int old_capacity = track->capacity;
		track->capacity *= 2;
		track->begin = realloc(track->begin, track->capacity * sizeof(garmin_trk_point_t));
		if (!track->begin)
			DIE("realloc", errno);
		track->end = track->begin + old_capacity;
	}
	*track->end++ = *trk_point;
}

static int int_compare(const void *p1, const void *p2)
{
	return *(int *) p1 - *(int *) p2;
}

void garmini_track_split(garmini_track_t *track, int nsplits, int *splits)
{
	qsort(splits, nsplits, sizeof(int), int_compare);
	garmin_trk_point_t *begin = track->begin;
	int i;
	for (i = 0; i < nsplits; ++i) {
		garmin_trk_point_t *end = track->begin + splits[i];
		if (begin != end) {
			garmini_track_new(0, begin, end);
			begin = end;
		}
	}
}

garmini_track_t *garmini_transfer_trk(garmin_t *garmin)
{
	garmini_track_t *track = garmini_track_new(16384, 0, 0);
	garmin_transfer_trk(garmin, (void (*)(void *, const garmin_trk_point_t *)) garmini_track_push, track);
	return track;
}

void garmini_write_igc(FILE *file, garmin_t *garmin, const garmin_trk_point_t *begin, const garmin_trk_point_t *end)
{
	fprintf(file, "A%s%03d\r\n", manufacturer, serial_number);
	time_t time = (begin == end ? 0 : begin->time) + 631065600;
	struct tm *tm = gmtime(&time);
	fprintf(file, "HFDTE%02d%02d%02d\n", tm->tm_mday, tm->tm_mon + 1, (tm->tm_year + 1900) % 100);
	struct tm last_tm = *tm;
	fprintf(file, "HFFXA100\r\n");
	if (pilot)
		fprintf(file, "HPPLTPILOT:%s\r\n", pilot);
	if (glider_type)
		fprintf(file, "HPGTYGLIDERTYPE:%s\r\n", glider_type);
	if (glider_id)
		fprintf(file, "HPGIDGLIDERID:%s\r\n", glider_id);
	fprintf(file, "HDTM100GPSDATUM:WGS-1984\r\n");
	fprintf(file, "HFRFWFIRMWAREREVISION:%d.%02d\r\n", garmin->product_data->software_version / 100, garmin->product_data->software_version % 100);
	fprintf(file, "HFFTYFRTYPE:GARMIN,%s\r\n", garmin->product_data->product_description);
	if (competition_id)
		fprintf(file, "HPCIDCOMPETITIONID:%s\r\n", competition_id);
	if (competition_class)
		fprintf(file, "HPCCLCOMPETITIONCLASS:%s\r\n", competition_class);
	const garmin_trk_point_t *trk_point;
	for (trk_point = begin; trk_point != end; ++trk_point) {
		if ((trk_point->posn.lat == 0x7fffffff && trk_point->posn.lon == 0x7fffffff) || trk_point->alt == 1.0e25)
			continue;
		time = trk_point->time + 631065600;
		struct tm *tm = gmtime(&time);
		if (tm->tm_year != last_tm.tm_year || tm->tm_mon != last_tm.tm_mon || tm->tm_mday != last_tm.tm_mday) {
			fprintf(file, "HFDTE%02d%02d%02d\n", tm->tm_mday, tm->tm_mon + 1, (tm->tm_year + 1900) % 100);
			last_tm = *tm;
		}
		float lat = fabs(180.0 * trk_point->posn.lat / 2147483648.0) + 0.5 / 60000.0;
		float lon = fabs(180.0 * trk_point->posn.lon / 2147483648.0) + 0.5 / 60000.0;
		int int_alt = trk_point->alt <= 0.0 ? 0 : trk_point->alt + 0.5;
		int pressure_alt;
		int gnss_alt;
		if (barometric_altimeter) {
			pressure_alt = int_alt;
			gnss_alt = 0;
		} else {
			pressure_alt = 0;
			gnss_alt = int_alt;
		}
		fprintf(file, "B%02d%02d%02d%02d%05d%c%03d%05d%c%c%05d%05d\r\n", tm->tm_hour, tm->tm_min, tm->tm_sec, (int) lat, (int) (60000 * (lat - (int) lat)), trk_point->posn.lat > 0 ? 'N' : 'S', (int) lon, (int) (60000 * (lon - (int) lon)), trk_point->posn.lon > 0 ? 'E' : 'W', trk_point->validity, pressure_alt, gnss_alt);
	}
}

void garmini_id(garmin_t *garmin)
{
	printf("--- \n");
	printf("product_id: %d\n", garmin->product_data->product_id);
	printf("software_version: %d.%02d\n", garmin->product_data->software_version / 100, garmin->product_data->software_version % 100);
	printf("product_description: \"");
	print_string(stdout, garmin->product_data->product_description, -1);
	printf("\"\n");
	printf("protocols: \"");
	if (garmin->nprotocols) {
		printf("%c%03d", garmin->protocols[0].tag, garmin->protocols[0].data);
		int i;
		for (i = 1; i < garmin->nprotocols; ++i)
			printf(",%c%03d", garmin->protocols[i].tag, garmin->protocols[i].data);
	}
	printf("\"\n");
}

void garmini_igc(garmin_t *garmin)
{
	garmini_track_t *track = garmini_transfer_trk(garmin);
	garmini_write_igc(stdout, garmin, track->begin, track->end);
	garmini_track_delete(track);
}

void garmini_download(garmin_t *garmin)
{
	if (directory && chdir(directory) == -1)
		error("chdir: %s: %s", directory, strerror(errno));
	garmini_track_t *track = garmini_transfer_trk(garmin);
	struct tm last_tm;
	memset(&last_tm, 0, sizeof last_tm);
	int track_number = 0;
#if 0
	garmini_track_t *segment;
	for (segment = begin; segment != end; ++segment) {
		if (segment->end - segment->begin < minimum_trk_points)
			continue;
		if (segment->end->time - segment->begin->time < minimum_duration)
			continue;
		time_t time = segment->begin->time + 631065600;
		struct tm *tm = gmtime(&time);
		if (tm->tm_year == last_tm.tm_year && tm->tm_mon == last_tm.tm_mon && tm->tm_mday == last_tm.tm_mday)
			++track_number;
		else
			last_tm = *tm;
		char filename[1024];
		snprintf(filename, sizeof filename, "%04d-%02d-%02d-%s-%d-%02d.IGC", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, manufacturer, serial_number, track_number);
		FILE *file = fopen(filename, "w");
		if (!file)
			error("%s: %s", filename, strerror(errno));
		garmini_write_igc(file, garmin, segment->begin, segment->end);
		if (!fclose(file))
			error("%s: %s", filename, strerror(errno));
	}
#endif
}

static void usage(void)
{
	printf("%s - download track log from Garmin GPSs\n"
			"Usage: %s [options] [command]\n"
			"Options:\n"
			"\t-h, --help\t\t\tshow some help\n"
			"\t-q, --quiet\t\t\tsuppress output\n"
			"\t-d, --device=DEVICE\t\tselect device (default is /dev/ttyS0)\n"
			"\t-D, --directory=DIR\t\tdownload tracklogs to DIR\n"
			"\t-l, --log=FILENAME\t\tlog communication to FILENAME\n"
			"\t-o, --power-off\t\t\tpower off GPS\n"
			"IGC options:\n"
			"\t-m, --manufacturer=STRING\toverride manufacturer\n"
			"\t-n, --serial-number=NUMBER\toverride serial number\n"
			"\t-p, --pilot=PILOT\t\tset pilot\n"
			"\t-t, --glider-type=TYPE\t\tset glider type\n"
			"\t-g, --glider-id=ID\t\tset glider id\n"
			"\t-c, --competition-class=CLASS\tset competition class\n"
			"\t-i, --competition-id=ID\t\tset competition id\n"
			"\t-b, --barometric-altimeter=0|1\tset whether GPS has a barometric altimeter\n"
			"Commands:\n"
			"\tid\t\tidentify GPS\n"
			"\tdo, download\tdownload tracklogs\n"
			"\tig, igc\t\twrite entire track log to stdout\n"
			"Supported GPSs:\n"
			"\tAll modern Garmin GPSs and the following older models:\n"
			"\tGPS 12, GPS 12 XL, GPS 12 XL Chinese, GPS 12 XL Japanese, GPS 120,\n"
			"\tGPS 120 Chinese, GPS 120 XL, GPS 125 Sounder, GPS 126, GPS 126 Chinese,\n"
			"\tGPS 128, GPS 128 Chinese, GPS 38, GPS 38 Chinese, GPS 38 Japanese,\n"
			"\tGPS 40, GPS 40 Chinese, GPS 40 Japanese, GPS 45, GPS 45 Chinese,\n"
			"\tGPS 45 XL, GPS 48, GPS 65, GPS 75, GPS 85, GPS 89, GPS 90, GPS 92,\n"
			"\tGPS 95, GPS 95 AVD, GPS 95 XL, GPS II, GPS II Plus, GPS III,\n"
			"\tGPS III Pilot, GPSCOM 170, GPSCOM 190, GPSMAP 130, GPSMAP 130 Chinese,\n"
			"\tGPSMAP 135 Sounder, GPSMAP 175, GPSMAP 195, GPSMAP 205, GPSMAP 210,\n"
			"\tGPSMAP 215, GPSMAP 220, GPSMAP 225, GPSMAP 230, GPSMAP 230 Chinese,\n"
			"\tGPSMAP 235\n",
		program_name, program_name);
}

int main(int argc, char *argv[])
{
	program_name = strrchr(argv[0], '/');
	program_name = program_name ? program_name + 1 : argv[0];

	device = getenv("GARMINI_DEVICE");
	if (!device)
		device = "/dev/ttyS0";

	setenv("TZ", "UTC", 1);
	tzset();

	opterr = 0;
	while (1) {
		static struct option options[] = {
			{ "help",                 no_argument,       0, 'h' },
			{ "quiet",                no_argument,       0, 'q' },
			{ "device",               required_argument, 0, 'd' },
			{ "directory",            required_argument, 0, 'D' },
			{ "log",                  required_argument, 0, 'l' },
			{ "power-off",            no_argument,       0, 'o' },
			{ "manufacturer",         required_argument, 0, 'm' },
			{ "serial-number",        required_argument, 0, 's' },
			{ "pilot",                required_argument, 0, 'p' },
			{ "glider-type",          required_argument, 0, 't' },
			{ "glider-id",            required_argument, 0, 'g' },
			{ "competition-class",    required_argument, 0, 'c' },
			{ "competition-id",       required_argument, 0, 'i' },
			{ "barometric-altimeter", required_argument, 0, 'b' },
			{ 0,                      0,                 0, 0 },
		};
		int c = getopt_long(argc, argv, ":hqd:D:l:om:s:p:t:g:c:i:b:", options, 0);
		if (c == -1)
			break;
		char *endptr;
		switch (c) {
			case 'D':
				directory = optarg;
				break;
			case 'c':
				competition_class = optarg;
				break;
			case 'd':
				device = optarg;
				break;
			case 'g':
				glider_id = optarg;
				break;
			case 'h':
				usage();
				exit(EXIT_SUCCESS);
			case 'i':
				competition_id = optarg;
				break;
			case 'l':
				if (strcmp(optarg, "-") == 0)
					logfile = stdout;
				else {
					logfile = fopen(optarg, "a");
					if (!logfile)
						error("fopen: %s: %s", optarg, strerror(errno));
				}
				break;
			case 'm':
				manufacturer = optarg;
				break;
			case 'n':
				serial_number = strtol(optarg, &endptr, 10);
				if (endptr == optarg || *endptr != '\0')
					error("invalid serial number '%s'", optarg);
				break;
			case 'o':
				power_off = 1;
				break;
			case 'p':
				pilot = optarg;
				break;
			case 'q':
				quiet = 1;
				break;
			case 's':
				barometric_altimeter = strtol(optarg, &endptr, 10);
				if (endptr == optarg || *endptr != '\0' || barometric_altimeter < 0 || 1 < barometric_altimeter)
					error("invalid argument '%s'", optarg);
				break;
			case 't':
				glider_type = optarg;
				break;
			case ':':
				error("option '%c' requires an argument", optopt);
			case '?':
				error("invalid option '%c'", optopt);
		}
	}

	garmin_t *garmin = garmin_new(device, logfile);

	if (barometric_altimeter == -1)
		barometric_altimeter = garmin_has_barometric_altimeter(garmin);

	if (optind == argc || strcmp(argv[optind], "do") == 0 || strcmp(argv[optind], "download") == 0) {
		garmini_download(garmin);
	} else {
		if (optind + 1 != argc)
			error("excess arguments on command line");
		if (strcmp(argv[optind], "id") == 0) {
			garmini_id(garmin);
		} else if (strcmp(argv[optind], "ig") == 0 || strcmp(argv[optind], "igc") == 0) {
			garmini_igc(garmin);
		} else {
			error("invalid command '%s'", argv[optind]);
		}
	}

	if (power_off)
		garmin_turn_off_pwr(garmin);

	garmin_delete(garmin);
	if (logfile && logfile != stdout)
		fclose(logfile);

	return 0;
}
