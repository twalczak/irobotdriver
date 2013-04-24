#ifndef RMBDRIVER_H
#define RMBDRIVER_H
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

/* -------------------------------------------------------------- */
#include "serial.h"
#include "rmbdriver.h"

/*-----------------------------------------------
		GLOBAL VARIABLES
-------------------------------------------------*/

bool client_connected;
double dvdt_global;
double drdt_global;
int _irobot_count;
char serial_port_str[13] = "/dev/ttyUSBx"; // <---- NOT NEEDED ANYMORE

class rmbDriver : public ThreadedDriver {
public:
	rmbDriver(ConfigFile *cf, int section);
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void* data);
	static void* usonic_thread(void* arg);
	static void* nav_thread(void* arg);
	static void* screen_thread(void* arg);
	static void* irobot_control_thread(void* arg);
	static void* server_thread(void* arg);
	
private:
	double adcdata2m(int adc);
	int serialport_read_until(int fd, char* buf, char until, uint16_t* sensors);
	int parse_packet(uint8_t* packet,uint16_t* sensors);
	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
	int irobot_port;
	int nav_port;
	int usonic_port;
	player_devaddr_t positionID;
	player_devaddr_t sonarID;
	player_devaddr_t laserID;
};

Driver* rmbDriver_Init(ConfigFile* cf, int section) {		// Return driver instance to Player
	return( (Driver*)(new rmbDriver(cf, section)) );	
}

void rmbDriver_Register(DriverTable* table) {			// Register driver with Player
	table->AddDriver("rmbdriver", rmbDriver_Init);
}

// Needed for older versions of Player server
extern "C" {
	int player_driver_init(DriverTable* table) {
		rmbDriver_Register(table);
		return 0;
	}
}

void mdelay(int d) {
	int i;
	for(i=0; i<d; i++) {
		usleep(1000);
	}
}

#endif
