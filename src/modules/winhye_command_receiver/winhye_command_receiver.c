/*
 * winhye_command_receiver.c
 *
 * read winhye_command_receiver through uart
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

#include <uORB/topics/winhye_command.h>




#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define ASH_UNUSED(x) (void)x;

//#define ASH_DEBUG(...)		{GPS_WARN(__VA_ARGS__);}
#define ASH_DEBUG(...)		{/*GPS_WARN(__VA_ARGS__);*/}



__EXPORT int winhye_command_receiver_main(int argc, char *argv[]);


static bool thread_should_exit = false; /*Ddemon exit flag*/
static bool thread_running = false;  /*Daemon status flag*/
static int daemon_task;

/**
 * Main loop
 */
int winhye_command_receiver_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
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


int uart_init(const char * uart_name)
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

    fprintf(stderr, "usage: winhye_command_receiver {start|stop|status} [param]\n\n");
    exit(1);
}

int winhye_command_receiver_main(int argc, char *argv[])
{

    if (argc < 2) {
        usage("[winhye_command_receiver]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[winhye_command_receiver]already running\n");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("winhye_command_receiver",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         winhye_command_receiver_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[winhye_command_receiver]running");
            return 0;

        } else {
            warnx("[winhye_command_receiver]stopped");
            return 1;
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int read_thread_main(int argc, char *argv[])
{
    if (argc < 2) {
        errx(1, "[winhye_command_receiver]need a serial port name as argument");
        usage("eg:");
    }
    const char *uart_name = argv[1];

    warnx("[winhye_command_receiver]opening port %s", uart_name);
    char data = '0';
    char buffer[100] = "";
    char *bufptr = (char *)(buffer);

    char *endp;

    int uart_read = uart_init(uart_name);
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,115200)){
        printf("[winhye_command_receiver]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[winhye_command_receiver]uart init is successful\n");

    /* advertise gps topic */
    struct uart_get_s uartdata;
    memset(&uartdata, 0, sizeof(uartdata));
    orb_advert_t uartdata_pub = orb_advertise(ORB_ID(uart_get), &uartdata);

    /* subscribe to vehicle_gps_position topic */
    int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));

    /*limit the update rate to 20 Hz */
    orb_set_interval(gps_sub_fd, 500);

    thread_running = true;
    uartdata.type = 0;
    orb_publish(ORB_ID(uart_get), uartdata_pub, &uartdata);
    while(!thread_should_exit){

        read(uart_read,&data,1);
        if(data == '$'){
            int i = 0;
            while(data!='\n'){
                buffer[i] = data;
                read(uart_read,&data,1);
                i++;
            }
            buffer[i] = '\0';
            //printf("%s\n",buffer);

            bufptr = (char *)(buffer + 1);
            int num = 0;
            char ns = '?';

            if (bufptr) { num = strtol(bufptr, &endp, 10); bufptr = endp; }

            if (bufptr) { ns = *(bufptr++); }
            if (ns == 'E') {
                uartdata.type = num;
                //printf("%d\n",num);
                orb_publish(ORB_ID(uart_get), uartdata_pub, &uartdata);
            }

        }

    }

    warnx("[winhye_command_receiver]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}

