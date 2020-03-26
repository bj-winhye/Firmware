/*
 * winhye_speed_controller.c
 *
 * winhye_speed_controller
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
#include <string.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/uart_get.h>
#include <parameters/param.h>


__EXPORT int winhye_speed_controller_main(int argc, char *argv[]);


static bool thread_should_exit = false; /*Ddemon exit flag*/
static bool thread_running = false;  /*Daemon status flag*/
static int daemon_task;
static void usage(const char *reason);

int winhye_speed_controller_thread_main(int argc, char *argv[]);

int winhye_speed_controller_main(int argc, char *argv[])
{
    if (argc < 2) {
       usage("[winhye_speed_controller]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[winhye_speed_controller]already running\n");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("winhye_speed_controller",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         winhye_speed_controller_thread_main,
                         (argv) ? (char * const *)&argv[1] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[winhye_speed_controller]running");
            return 0;

        } else {
            warnx("[winhye_speed_controller]stopped");
            return 1;
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}
static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: winhye_speed_controller {start|stop|status} [param]\n\n");
    exit(1);
}

bool setspeed(float speed){
    while(param_set(param_find("GND_THR_MAX"),&(speed)) == 1){;}
    while(param_set(param_find("GND_THR_MIN"),&(speed)) == 1){;}
    return true;
}

int winhye_speed_controller_thread_main(int argc, char *argv[])
{
    PX4_INFO("Hello winhye_speed_controller!\n");

    float speed = 0.f;

    /* subscribe to uart_get topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(uart_get));

    /*limit the update rate to 20 Hz */
    //orb_set_interval(sensor_sub_fd, 200);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    thread_running = true;
    while(!thread_should_exit){ // infinite loop
        /* wait for sensor update of 1 file descriptor for 200 ms  */
        int poll_ret = poll(fds, 1, 200);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            //printf("[px4_test] Got no data within a second\n");
            //setspeed(0);

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            setspeed(0);
        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct uart_get_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(uart_get), sensor_sub_fd, &raw);
                speed = 0.f;
                if(raw.type == 1){
                    speed = 0.1;
                }else if(raw.type == 0){
                    speed = 0.2;
                }
                setspeed(speed);
                //PX4_INFO("type:\t%d\tspeed:\t%f\n",raw.type,(double)speed);
            }

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }


    }

    warnx("[winhye_speed_controller]exiting");
	thread_running = false;

    fflush(stdout);
    return 0;
}

