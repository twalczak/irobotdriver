#include "serial.h"
#include <iostream>
#include <string.h>
#include <libplayercore/playercore.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>   /* Standard types */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <string>
#if !defined (WIN32)
    #include <unistd.h>
#endif

int serialport_writebyte(int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}


int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    // Open port
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    // Read current termios settings
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

    // Set baud rate variable
    speed_t brate = baud;
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // Setup termios for 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    // Reccomended settings "BSD RAW MODE" ??
    toptions.c_cflag &= ~CRTSCTS;   // no flow control
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on read & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // Setting when read() releases ---> Something's wrong with this. read() is still non-blocking
    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html (Still a little confusing)
    toptions.c_cc[VMIN]  = 0; // was 0
    toptions.c_cc[VTIME] = 20; // was 20
    
    // Apply settings
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes :'(");
        return -1;
    }

    return fd;
}
