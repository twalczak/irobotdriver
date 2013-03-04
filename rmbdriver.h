#ifndef RMBDRIVER_H
#define RMBDRIVER_H


/*-----------------------------------------------
		GLOBAL VARIABLES
-------------------------------------------------*/

bool client_connected;



void mdelay(int d) {
    int i;
    for(i=0; i<d; i++) {
        usleep(1000);
    }
}

class rmbDriver : public ThreadedDriver {
public:
	rmbDriver(ConfigFile *cf, int section);
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void* data);
	static void* screen_thread(void* arg);
	static void* nav_thread(void* arg);
    static void* irobot_control_thread(void* arg);
	
private:
    //int serialport_init(const char* serialport, int baud);
    int serialport_writebyte( int fd, uint8_t b);
    int serialport_writebyte( int fd, uint8_t b, uint8_t sonic);
    double adcdata2m(int adc);
    int serialport_read_until(int fd, char* buf, char until, uint16_t* sensors);
    int parse_packet(uint8_t* packet,uint16_t* sensors);
    uint8_t rmbOPEN;
	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
    int fd;
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

#endif