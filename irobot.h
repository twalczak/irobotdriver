#ifndef IROBOT_H
#define IROBOT_H
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
#if !defined (WIN32)
    #include <unistd.h>
#endif
#include "serial.h"

#define CREATE_TVEL_MAX_MM_S       500     
#define CREATE_RADIUS_MAX_MM       2000
#define CREATE_AXLE_LENGTH         0.258

int irobot_init(int* fd);
int irobot_setspeed(int* fd,double dvdt, double drdt);
int irobot_responding(int* fd);

#endif
