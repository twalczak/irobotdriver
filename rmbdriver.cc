// TESTING GIT TOMASZ WALCZAK

#include <iostream>
#include <string.h>
#include <libplayercore/playercore.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
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



using namespace std;

class rmbDriver : public ThreadedDriver {
public:
	rmbDriver(ConfigFile *cf, int section);
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void* data);
    int fd;
private:
    int serialport_init(const char* serialport, int baud);
    int serialport_writebyte( int fd, uint8_t b);
	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
	int testVar;
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

rmbDriver::rmbDriver(ConfigFile* cf, int section) : ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN/*, PLAYER_POSITION2D_CODE*/) {

    // Check if the configuration file asks us to provide a Limb interface
    if(cf->ReadDeviceAddr(&positionID, section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
    {
        // If the interface failed to correctly register
        if(AddInterface(positionID) != 0)
        {
            // Post an error string and quit the constructor
            PLAYER_ERROR("Error adding position2d interface\n");
            SetError(-1);
            return;
        }
    }
    
    // Check if the configuration file asks us to provide a Limb interface
    if(cf->ReadDeviceAddr(&sonarID, section, "provides", PLAYER_SONAR_CODE, -1, NULL) == 0)
    {
        // If the interface failed to correctly register
        if(AddInterface(sonarID) != 0)
        {
            // Post an error string and quit the constructor
            PLAYER_ERROR("Error adding sonar interface\n");
            SetError(-1);
            return;
        }
    } else {
        cout << "No sonar\n";
    }
    
        // Check if the configuration file asks us to provide a Limb interface
    if(cf->ReadDeviceAddr(&laserID, section, "provides", PLAYER_LASER_CODE, -1, NULL) == 0)
    {
        // If the interface failed to correctly register
        if(AddInterface(laserID) != 0)
        {
            // Post an error string and quit the constructor
            PLAYER_ERROR("Error adding laser interface\n");
            SetError(-1);
            return;
        }
    } else {
        cout << "No laser\n";
    }

	this->testVar = cf->ReadInt(section, "testVar", 0); // Read variable in .cfg file
	//return 
	
}

int rmbDriver::MainSetup() {
	cout << "---> Main Setup Request <---\n";
    int baudrate = B57600;  // default
    fd = serialport_init((char*)"/dev/ttyUSB0", baudrate);
        if(fd==-1) return -1;
    serialport_writebyte(fd,128);
    serialport_writebyte(fd,130);
	return 0;
}

void rmbDriver::MainQuit() {
	cout << "---> Player Quit Request <---\n";
}

int rmbDriver::ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void * data) {
	printf("---> Message Received <---\n        TYPE: %d SUBTYPE: %d SIZE: %d\n", hdr->type, hdr->subtype, hdr->size);
	
	// VELOCITY COMMAND
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL))
	{
		player_position2d_cmd_vel_t* vel_cmd = (player_position2d_cmd_vel_t*) data;
		player_pose2d_t pose_data = vel_cmd->vel;
		
		int16_t vel = pose_data.px * 500;
		int16_t deg = (pose_data.pa < 0.1 && pose_data.pa > -0.1) ? 32768 : 
			      (pose_data.pa < 0) ? (-1-pose_data.pa) * 2000 : (1-pose_data.pa) * 2000 ;
		
		uint8_t vel_h = (vel & 0xFF00) >> 8;
		uint8_t vel_l = (vel & 0x00FF);
		uint8_t deg_h = (deg & 0xFF00) >> 8;
		uint8_t deg_l = (deg & 0x00FF);
        serialport_writebyte(fd,137);
        serialport_writebyte(fd,vel_h);
        serialport_writebyte(fd,vel_l);
        serialport_writebyte(fd,deg_h);
        serialport_writebyte(fd,deg_l);
		printf("        VEL: %x %x\n", vel_h, vel_l);
        printf("        DEG: %x %x\n", deg_h, deg_l);
		return 0;
	}
	
	return -1;
}

void rmbDriver::Main() {
	bool run = true;
	while(run) {
		pthread_testcancel();		// Check Player for cancel flag
		ProcessMessages();		// Tell Player to invoke ProcessMessage if new messages exist
		usleep(100);			// Sleep thread
	}
}

// Needed for older versions of Player server
extern "C" {
	int player_driver_init(DriverTable* table) {
		rmbDriver_Register(table);
		return 0;
	}
}

int rmbDriver::serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}

int rmbDriver::serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        serialport,baud);

    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
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

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}














