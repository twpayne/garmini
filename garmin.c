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

#include <ctype.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "garmin.h"
#include "garmini.h"

#if __BYTE_ORDER != __LITTLE_ENDIAN
#error Only little-endian machines are currently supported
#endif

#define GARMINI_TIMEOUT (10 * 1000)

enum {
	DLE = 16,
	ETX =  3
};

enum {
	Pid_Ack_Byte =  6,
	Pid_Nak_Byte = 21
};

enum {
	Pid_Protocol_Array   = 253,
	Pid_Product_Rqst     = 254,
	Pid_Product_Data     = 255,
	Pid_Ext_Product_Data = 248
};

enum {
	Pid_Command_Data = 10,
	Pid_Xfer_Cmplt   = 12,
	Pid_Records      = 27,
	Pid_Trk_Data     = 34,
	Pid_Trk_Hdr      = 99
};

enum {
	Tag_Phys_Prot_Id = 'P',
	Tag_Link_Prot_Id = 'L',
	Tag_Appl_Prot_Id = 'A',
	Tag_Data_Prot_Id = 'D'
};

enum {
	Cmnd_Abort_Transfer = 0,
	Cmnd_Transfer_Trk   = 6,
	Cmnd_Turn_Off_Pwr   = 8
};

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
	while (garmin_read_packet_ack(garmin, packet) != id)
		warning("%s: unexpected packet %d", garmin->device, packet->id);
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
	void (*callback)(void *, const garmin_trk_point_t *, int, int);
	void *data;
} garmin_transfer_trk_data_t;

static void garmin_transfer_trk_callback(void *data, int i, int records, const garmin_packet_t *packet)
{
	garmin_transfer_trk_data_t *transfer_trk_data = data;
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
				transfer_trk_data->callback(transfer_trk_data->data, &trk_point, i, records);
				break;
			}
		case Pid_Trk_Hdr:
			break;
	}
}

void garmin_transfer_trk(garmin_t *garmin, void (*callback)(void *, const garmin_trk_point_t *, int, int), void *data)
{
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
	transfer_trk_data.callback = callback;
	transfer_trk_data.data = data;
	garmin_each(garmin, Cmnd_Transfer_Trk, garmin_transfer_trk_callback, &transfer_trk_data);
	return;
_error:
	error("%s: unsupported track transfer protocol", garmin->device);
}
