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
#include <sys/times.h>
#include <time.h>
#include <unistd.h>

#include "garmin.h"

const char *program_name = 0;
int _sc_clk_tck = -1;
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

void print_string(FILE *file, const char *s, int len)
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

typedef struct {
	garmini_track_t *track;
	clock_t clock;
	int remaining_sec;
	int percentage;
} garmini_transfer_trk_data_t;

static void garmini_transfer_trk_callback(void *data, const garmin_trk_point_t *trk_point, int i, int records)
{
	garmini_transfer_trk_data_t *transfer_trk_data = data;
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
	garmini_track_push(transfer_trk_data->track, trk_point);
}

garmini_track_t *garmini_transfer_trk(garmin_t *garmin)
{
	garmini_transfer_trk_data_t transfer_trk_data;
	memset(&transfer_trk_data, 0, sizeof transfer_trk_data);
	transfer_trk_data.track = garmini_track_new(16384, 0, 0);
	if (!quiet) {
		if (_sc_clk_tck == -1) {
			_sc_clk_tck = sysconf(_SC_CLK_TCK);
			if (_sc_clk_tck == -1)
				DIE("sysconf", errno);
		}
		struct tms tms;
		transfer_trk_data.clock = times(&tms);
		if (transfer_trk_data.clock == -1)
			DIE("times", errno);
		transfer_trk_data.remaining_sec = -1;
		transfer_trk_data.percentage = -1;
		fprintf(stderr, "Downloading track log: ");
	}
	garmin_transfer_trk(garmin, garmini_transfer_trk_callback, &transfer_trk_data);
	if (!quiet)
		fprintf(stderr, "100%%            \n");
	return transfer_trk_data.track;
}

void garmini_write_igc(FILE *file, garmin_t *garmin, const garmin_trk_point_t *begin, const garmin_trk_point_t *end)
{
	fprintf(file, "A%s%03d\r\n", manufacturer, serial_number);
	time_t time = (begin == end ? 0 : begin->time) + GARMIN_TIME_OFFSET;
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
		time = trk_point->time + GARMIN_TIME_OFFSET;
		struct tm *tm = gmtime(&time);
		if (tm->tm_year != last_tm.tm_year || tm->tm_mon != last_tm.tm_mon || tm->tm_mday != last_tm.tm_mday) {
			fprintf(file, "HFDTE%02d%02d%02d\n", tm->tm_mday, tm->tm_mon + 1, (tm->tm_year + 1900) % 100);
			last_tm = *tm;
		}
		double lat = fabs(180.0 * trk_point->posn.lat / 2147483648.0) + 0.5 / 60000.0;
		double lon = fabs(180.0 * trk_point->posn.lon / 2147483648.0) + 0.5 / 60000.0;
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
		time_t time = segment->begin->time + GARMIN_TIME_OFFSET;
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
			"\tig, igc\t\twrite entire track log to stdout\n",
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
