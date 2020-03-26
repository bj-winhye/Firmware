/*
 * winhye_name_gps_receiver.c
 *
 * read gps through uart
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdlib.h>

#include <poll.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/vehicle_gps_heading.h>


#include <parameters/param.h>


#ifndef M_PI_F
# define M_PI_F 3.14159265358979323846f
#endif

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define ASH_UNUSED(x) (void)x;

//#define ASH_DEBUG(...)		{GPS_WARN(__VA_ARGS__);}
#define ASH_DEBUG(...)		{/*GPS_WARN(__VA_ARGS__);*/}


__EXPORT int winhye_name_gps_receiver_main(int argc, char *argv[]);


static bool thread_should_exit = false; /*Ddemon exit flag*/
static bool thread_running = false;  /*Daemon status flag*/
static int daemon_task;

/**
 * Main loop
 */
int winhye_name_gps_receiver_thread_main(int argc, char *argv[]);

static int uart_init(const char *uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		warnx("ERR: baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);
	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}


int uart_init(const char *uart_name)
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
		return false;
	}

	return serial_fd;
}

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: winhye_name_gps_receiver {start|stop|status} [param]\n\n");
	exit(1);
}

int winhye_name_gps_receiver_main(int argc, char *argv[])
{


	if (argc < 2) {
		usage("[winhye_name_gps_receiver]missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("[winhye_name_gps_receiver]already running\n");
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("winhye_name_gps_receiver",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 winhye_name_gps_receiver_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("[winhye_name_gps_receiver]running");
			return 0;

		} else {
			warnx("[winhye_name_gps_receiver]stopped");
			return 1;
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int winhye_name_gps_receiver_thread_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "[winhye_name_gps_receiver]need a serial port name as argument");
		usage("eg:");
	}

	const char *uart_name = argv[1];

	warnx("[winhye_name_gps_receiver]opening port %s", uart_name);
	char data = '0';
	char check,getCheck;
	char *endp;
	int i;
	float get_heading = 0.f;

	struct vehicle_gps_position_s gpspos;

	memset(&gpspos, 0, sizeof(gpspos));
	orb_advert_t _gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &gpspos);

	struct heading_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(heading), &att);

	att.timestamp = hrt_absolute_time();
	att.q[0] = 0;
	att.q[1] = 0;
	att.q[2] = 0;
	att.q[3] = 0;
	orb_publish(ORB_ID(heading), att_pub, &att);

	int uart_read = uart_init(uart_name);

	if (false == uart_read) { return -1; }

	if (false == set_uart_baudrate(uart_read, 115200)) {
		printf("[winhye_name_gps_receiver]set_uart_baudrate is failed\n");
		return -1;
	}

	printf("[winhye_name_gps_receiver]uart init is successful\n");

	thread_running = true;

	while (!thread_should_exit) {

		read(uart_read, &data, 1);
		//printf("%c",data);

		if (data == '$') {
			char buffer[100] = "";
			char *bufptr = (char *)(buffer + 6);
			buffer[0] = data;
			check = 0x00;
			for(i = 1; data != '*'; i++) {
				read(uart_read, &data, 1);
				buffer[i] = data;
				//printf("%c",data);
				check ^= data=='*'?0x00:data;
			}
			read(uart_read, &data, 1);
			buffer[i++] = data;
			getCheck = (data<='9'?data-'0':(data<'G'?data-'A'+10:data-'a'+10))*16;
			read(uart_read, &data, 1);
			buffer[i++] = data;
			getCheck += (data<='9'?data-'0':(data<'G'?data-'A'+10:data-'a'+10));
			buffer[i] = '\n';
			//printf("%sgetCheck:%x   check:%x  ok?:%d\n", buffer,getCheck,check,getCheck==check);

			if (getCheck==check && memcmp(buffer + 3, "GGA,", 3) == 0) {
				//write(uart_read,&buffer,strlen(buffer));
				double ashtech_time = 0.0, lat = 0.0, lon = 0.0;//, alt = 0.0;
				int num_of_sv = 0, fix_quality = 0;
				double hdop = 99.9;
				char ns = '?', ew = '?';

				ASH_UNUSED(ashtech_time);
				ASH_UNUSED(num_of_sv);
				ASH_UNUSED(hdop);

				if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

				if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

				if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

				//if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

				if (ns == 'S') {
					lat = -lat;
				}

				if (ew == 'W') {
					lon = -lon;
				}

				if (fix_quality <= 0) {
					gpspos.fix_type = 0;

				} else {
					/*
					* in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
					* and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position.fix_type
					*/
					if (fix_quality == 5) { fix_quality = 3; }
					/*
					* fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
					*/
					gpspos.fix_type = 3 + fix_quality - 1;
				}

				gpspos.timestamp = hrt_absolute_time();
				/* convert from degrees, minutes and seconds to degrees * 1e7 */
				gpspos.lat = (int32_t)(((int)(lat * 0.01) + (lat * 0.01 - (int)(lat * 0.01)) * 100.0 / 60.0) * 10000000);
				gpspos.lon = (int32_t)(((int)(lon * 0.01) + (lon * 0.01 - (int)(lon * 0.01)) * 100.0 / 60.0) * 10000000);
				gpspos.alt = 0;
				gpspos.satellites_used = num_of_sv;
				gpspos.s_variance_m_s = 0;
				gpspos.c_variance_rad = 0;
				gpspos.hdop = hdop;
				gpspos.vdop = hdop;// has no "vdop".
				gpspos.vel_m_s = 0;
				gpspos.vel_n_m_s = 0;
				gpspos.vel_e_m_s = 0;
				gpspos.vel_d_m_s = 0;
				gpspos.cog_rad = 0;
				gpspos.vel_ned_valid = false;
				gpspos.noise_per_ms = 0;
				gpspos.jamming_indicator = 0;
				gpspos.timestamp_time_relative = 0;
				gpspos.time_utc_usec = 0;
				gpspos.heading = NAN;
				int gpsinstance;
				orb_publish_auto(ORB_ID(vehicle_gps_position), &_gps_pub, &gpspos, &gpsinstance, ORB_PRIO_HIGH);

			} else if (getCheck==check && memcmp(buffer + 3, "HDT,", 3) == 0) {

				if (bufptr && *(++bufptr) != ',') {
					get_heading = strtof(bufptr, &endp); bufptr = endp;
					get_heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
					//heading -= M_PI_F; // range: [-pi, 3pi]
					if (get_heading > M_PI_F) {
						get_heading -= 2.f * M_PI_F; // final range is [-pi, pi]
					}
				}
				att.timestamp = hrt_absolute_time();
				att.q[0] = cos(get_heading / 2);
				att.q[1] = 0;
				att.q[2] = 0;
				att.q[3] = sin(get_heading / 2);
				//printf("%2.6f\n",(double)get_heading);
				orb_publish(ORB_ID(heading), att_pub, &att);
			}

		}
	}

	warnx("[winhye_name_gps_receiver]exiting");
	thread_running = false;
	close(uart_read);

	fflush(stdout);
	return 0;
}
