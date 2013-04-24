#include "irobot.h"
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

int irobot_init(int* fd){

}
int irobot_setspeed(int* fd, double dvdt, double drdt){
	int16_t tv_mm, rad_mm;

	tv_mm = (int16_t)rint(dvdt * 1e3); // Meter to Milimeter
	tv_mm = MAX(tv_mm, -500);
	tv_mm = MIN(tv_mm,  500);

	if(drdt == 0) {
		rad_mm = 0x8000;
	}
	else if(dvdt == 0) {
		if(drdt > 0)
			rad_mm = 1;
		else
			rad_mm = -1;
		tv_mm = (int16_t)rint(0.258 * fabs(drdt) * 1e3);
	} else {
		rad_mm = (int16_t)rint(tv_mm / drdt); // rint probably not needed
		rad_mm = MAX(rad_mm, -2000);
		rad_mm = MIN(rad_mm, 2000);
	}
	uint8_t vel_h = (tv_mm & 0xFF00) >> 8;
	uint8_t vel_l = (tv_mm & 0x00FF);
	uint8_t deg_h = (rad_mm & 0xFF00) >> 8;
	uint8_t deg_l = (rad_mm & 0x00FF);

        serialport_writebyte(*fd,137);
        serialport_writebyte(*fd,vel_h);
        serialport_writebyte(*fd,vel_l);
        serialport_writebyte(*fd,deg_h);
        serialport_writebyte(*fd,deg_l);
}

int irobot_responding(int* fd) {

}
