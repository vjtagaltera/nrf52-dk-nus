/*
 * to compile: 
 *      $CC -I ./ -O3 -Og -Wall -Werror uart-cat-linux.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <stdint.h>


static uint32_t ms_converted(struct timespec *now) {
    if ( now == NULL ) {
        return (uint32_t)0;
    }
    uint32_t tsec = (uint32_t)now->tv_sec;
    uint32_t msec = (uint32_t)((now->tv_nsec/1000/1000)%1000);
    msec += ((uint32_t)(tsec * 1000));
    if ( msec == 0 ) {
        msec = 1; /* avoid 0 */
    }
    return msec;
}

static uint32_t ms_stamp() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return ms_converted(&now);
}


int main(int argc, char *argv[])
{
    struct termios options;

    static int serial_fd = -1;
    serial_fd = open("/dev/ttyAMA1", O_RDWR | O_NDELAY | O_NOCTTY);
    if (serial_fd < 0) {
        printf("Failed open ttyAMA1\n");
        return 1;
    }

    fcntl(serial_fd, F_SETFL, 0);

    // Get the current options for the port...
    tcgetattr(serial_fd, &options);

    // Set the baud rates
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Enable the receiver and set local mode...
    // Turn off charachter processing, set to 8N1
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= (CLOCAL | CREAD | CS8);

    // turn off output processing
    options.c_oflag = 0;

    // Turn off input processing
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    // One byte is enough to read, no inter-charachter timer
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    // Set the new options for the port...
    tcsetattr(serial_fd, TCSANOW, &options);

    uint32_t ms0 = ms_stamp();
    uint32_t ms_last = 0;
    while(1) {
        char buf[256];
        memset(buf, 0, sizeof(buf));
        int len = read(serial_fd, buf, 256);
        if (len > 0) {
            uint32_t ms1 = ms_stamp() - ms0;
            uint32_t msdelta = ms1 - ms_last;
            printf("Received data  at ms %u %-5u  len %d\n", ms1, msdelta, len);
            ms_last = ms1;
        }
        usleep(1000);
    }
    return 0;
}


