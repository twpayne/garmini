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

#ifndef GARMIN_H
#define GARMIN_H

#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define GARMIN_TIME_OFFSET 631065600

#define DIE(syscall, _errno) die(__FILE__, __LINE__, __FUNCTION__, (syscall), (_errno))

typedef struct {
	int id;
	int size;
	unsigned char data[255];
} garmin_packet_t;

typedef struct {
	uint16_t product_id;
	int16_t software_version;
	char product_description[1];
} Product_Data_Type;

typedef struct {
	uint8_t tag;
	uint16_t data __attribute__ ((packed));
} Protocol_Data_Type;

typedef struct {
	int32_t lat;
	int32_t lon;
} position_t;

typedef struct {
	time_t time;
	position_t posn;
	float alt;
	char validity;
} garmin_trk_point_t;

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

int garmin_read_packet(garmin_t *, garmin_packet_t *);
void garmin_write_packet(garmin_t *, garmin_packet_t *);
int garmin_read_packet_ack(garmin_t *, garmin_packet_t *);
int garmin_expect_packet_ack(garmin_t *, garmin_packet_t *, int);
void garmin_write_packet_ack(garmin_t *, garmin_packet_t *);
Protocol_Data_Type *garmin_grep_protocol(garmin_t *, int, int);
garmin_t *garmin_new(const char *, FILE *);
int garmin_has_barometric_altimeter(garmin_t *);
void garmin_delete(garmin_t *);
void garmin_each(garmin_t *, int, void (*)(void *, int, int, const garmin_packet_t *), void *);
void garmin_turn_off_pwr(garmin_t *);
void garmin_transfer_trk(garmin_t *, void (*)(void *, const garmin_trk_point_t *, int, int), void *);

#endif
