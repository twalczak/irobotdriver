/*
    Custom iRobot Create plugin for Player/Stage v3.0+
    Controls:   iRobot Create
                Ultrasonic Sensors
                Stargazer navigation system
                Sony camera

    Project:    Swarm Robotics - Senior Design Team 161
    School:     University of Connecticut
    Engineer:   Tomasz Walczak
    Date:       04 Feb 2013
*/

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
#include "serial.h"
#if !defined (WIN32)
    #include <unistd.h>
#endif

#define streamsize 20
#define packetsize 40
#define CREATE_TVEL_MAX_MM_S        500     
#define CREATE_RADIUS_MAX_MM        2000
#define CREATE_AXLE_LENGTH          0.258

using namespace std;
bool running;
char packetn[packetsize];
uint8_t packet[4];
int fdd, nav_fd;
uint8_t pstream[streamsize];
uint16_t sensors[8];
pthread_t nav_thread_id;
double nav_pos[5];
void draw_screen(uint8_t* packet,uint16_t* sensors);
double adcdata2m(int adc);
int serialport_read_until2(int fd, char* buf, char until, uint16_t* sensors);
int parse_packet2(uint8_t* packet,uint16_t* sensors);

void mdelay(int d) {
    int i;
    for(i=0, i<d, i++)
        usleep(1000);
}

class rmbDriver : public ThreadedDriver {
public:
	rmbDriver(ConfigFile *cf, int section);
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void* data);
	static void* screen_thread(void* arg);
	static void* nav_thread(void* arg);
    static void* irobot_control_thread(void* arg);
	
private:
    int serialport_init(const char* serialport, int baud);
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
	int testVar;
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

void* rmbDriver::screen_thread(void* arg) {
	char* buffer;
	while(1) {
		serialport_read_until2(fdd,buffer, 'i', sensors);
	}

}

void* rmbDriver::irobot_control_thread(void* arg) {
    while(client_connected) {
        if(!irobot_responding())
            irobot_init();
        irobot_setspeed(dvdt_global, drdt_global);
        mdelay(100); // ms
    }
    pthread_exit(&irobot_control_thread_id);
}

void* rmbDriver::nav_thread(void* arg) {
    char b[1];
    int error = 0;
    int noread = 0;
    int count = 0;
    char* r_id_str;
    char* a_str;
    char* x_str;
    char* y_str;
    char* z_str;

    double id,a,x,y,z;

    memset(packetn,0,sizeof(packetn));
    do { 
        int n = read(nav_fd, b, 1);  //Read byte at a time
        if(n==-1) { 
        	//printf("Global Error: %d ", errno); fflush(stdout);
        	//perror(strerror(errno));
         }
        else if( n==0 ) {
        	//No data available, yet... Hurry up Mike
            	//usleep( 10 * 1000 ); // wait 10 msec try again
            	continue;
        } else if(n>0) {
        	//Reset error counts
		noread = 0;
		error  = 0;
		//printf("TEST: %c\n",*b);
		//Shift in byte

        if(*b=='~' || count>(packetsize-2)) {
            memset(packetn,0,sizeof(packetn));
            count=0;
        }


        if(*b=='`') {
            printf("packet: %s \n", packetn);
            r_id_str = strtok (packetn,"~^|I");
            a_str = strtok (NULL,"~^|I");
            x_str = strtok (NULL,"~^|I");
            y_str = strtok (NULL,"~^|I");
            z_str = strtok (NULL,"~^|I");

            if(!((r_id_str == NULL) || (a_str == NULL) || (x_str == NULL) || (y_str == NULL) || (z_str == NULL))) {
                nav_pos[0] = atof(r_id_str);
                nav_pos[1] = atof(a_str);
                nav_pos[2] = atof(x_str);
                nav_pos[3] = atof(y_str);
                nav_pos[4] = atof(z_str);

                printf("DATA: id: %f a: %f x: %f y: %f z: %f\n",id,a,x,y,z);
            } else {
                printf("DATA: NULL\n");fflush(stdout);
            }

            memset(packetn,0,sizeof(packetn));
            count=0;
        }
        packetn[count] = *b;
        count++;

        }
        if(!running) pthread_exit(&nav_thread_id);
    } while(1);


    return 0;
}


rmbDriver::rmbDriver(ConfigFile* cf, int section) : ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN) {

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
    this->irobot_port = cf->ReadInt(section, "irobot_port", 0); // Read variable in .cfg file
    this->nav_port = cf->ReadInt(section, "nav_port", 0); // Read variable in .cfg file
    this->usonic_port = cf->ReadInt(section, "usonic_port", 0); // Read variable in .cfg file
	return; 
}

int rmbDriver::MainSetup() {
	cout << "---> Main Setup Request <---\n";
	
    int baudrate = B57600;  // default
    fd = serialport_init((char*)"/dev/ttyUSB0", baudrate);
    if(fd==-1) {
        PLAYER_ERROR("Fatal Error: Serial port failed to initialize");
        //return -1; 
        rmbOPEN = 0;
    } else {
    	rmbOPEN = 1;
    }
	serialport_writebyte(fd,128);   // TODO: Define opcodes in header
	serialport_writebyte(fd,132); usleep(50*1000);
	//serialport_writebyte(fd,131); usleep(50*1000);
	//serialport_writebyte(fd,132);
	
	//START
	pthread_t screen_thread_id;
	


	baudrate = B115200; //NAVIGATION
    nav_fd = serialport_init((char*)"/dev/ttyUSB1", baudrate);
    if(nav_fd==-1) {
        PLAYER_ERROR("NAVIGATION PORT FAILED");
        return -1; 
    }
    running = true;
    pthread_create(&nav_thread_id, NULL, &rmbDriver::nav_thread, (void*)NULL);


	int flags;
	baudrate = B2400;  // default
    	fdd = serialport_init((char*)"/dev/ttyUSB2", baudrate);
    	/*if(fdd==-1) {
    		cout << "Open port: error\n";
        	return -1;
        }
        
        
 	if (-1 == (flags = fcntl(fdd, F_GETFL, 0)))
 	       flags = 0;
    	fcntl(fdd, F_SETFL, flags | O_NONBLOCK);
        
        pthread_create(&screen_thread_id, NULL, &rmbDriver::screen_thread, (void*)NULL);/**/
        
         char* buffer;
	//serialport_read_until(fdd,buffer, 'i', sensors);
	//END
	return 0;
}

void rmbDriver::MainQuit() {
	cout << "---> Player Quit Request <---\n";
	running = false;
	//pthread_terminate(nav_thread_id);
	close(fd);
    // TODO: Close gracefuly
}

int rmbDriver::ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void * data) {
	printf("---> Message Received <---\n        TYPE: %d SUBTYPE: %d SIZE: %d\n", hdr->type, hdr->subtype, hdr->size);
	
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM, this->sonarID)) {
		cout << "They want geometry D:\n";
		
		//START
		player_pose3d_t poses[8];
		uint32_t poses_count = 8;
		poses[0].px = 0.15;
		poses[0].py = 0.00;
		poses[0].pz = 0.05;
		poses[0].proll = 0.0;
		poses[0].ppitch = 0.0;
		poses[0].pyaw = 0.0;
		
		poses[1].px = 0.15;
		poses[1].py = 0.15;
		poses[1].pz = 0.05;
		poses[1].proll = 0.0;
		poses[1].ppitch = 0.0;
		poses[1].pyaw = 0.785;
		
		poses[2].px = 0.0;
		poses[2].py = 0.15;
		poses[2].pz = 0.05;
		poses[2].proll = 0.0;
		poses[2].ppitch = 0.0;
		poses[2].pyaw = 1.57;
		
		poses[3].px = -0.15;
		poses[3].py = 0.15;
		poses[3].pz = 0.05;
		poses[3].proll = 0.0;
		poses[3].ppitch = 0.0;
		poses[3].pyaw = 2.36;
		
		poses[4].px = -0.15;
		poses[4].py = 0.0;
		poses[4].pz = 0.05;
		poses[4].proll = 0.0;
		poses[4].ppitch = 0.0;
		poses[4].pyaw = 3.14;
		
		poses[5].px = -0.15;
		poses[5].py = -0.15;
		poses[5].pz = 0.05;
		poses[5].proll = 0.0;
		poses[5].ppitch = 0.0;
		poses[5].pyaw = 3.926;
		
		poses[6].px = 0.0;
		poses[6].py = -0.15;
		poses[6].pz = 0.05;
		poses[6].proll = 0.0;
		poses[6].ppitch = 0.0;
		poses[6].pyaw = 4.712;
		
		poses[7].px = 0.15;
		poses[7].py = -0.15;
		poses[7].pz = 0.05;
		poses[7].proll = 0.0;
		poses[7].ppitch = 0.0;
		poses[7].pyaw = 5.498;
		
		player_sonar_geom_t sg;
		sg.poses_count = poses_count;
		sg.poses = poses;
		
		//END
		
		Publish(this->sonarID, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SONAR_REQ_GET_GEOM,(void*)&sg);
		return 0;
	}
	
	// VELOCITY COMMAND
	  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL))
	{
		player_position2d_cmd_vel_t* vel_cmd = (player_position2d_cmd_vel_t*) data;
		player_pose2d_t pose_data = vel_cmd->vel;
		
		double tv = pose_data.px;
		double rv = pose_data.pa;
/*--------- START COPYnPASTE*/
  int16_t tv_mm, rad_mm;

  //printf("tv: %.3lf rv: %.3lf\n", tv, rv);

  tv_mm = (int16_t)rint(tv * 1e3);
  tv_mm = MAX(tv_mm, -CREATE_TVEL_MAX_MM_S);
  tv_mm = MIN(tv_mm, CREATE_TVEL_MAX_MM_S);

  if(rv == 0)
  {
    // Special case: drive straight
    rad_mm = 0x8000;
  }
  else if(tv == 0)
  {
    // Special cases: turn in place
    if(rv > 0)
      rad_mm = 1;
    else
      rad_mm = -1;

    tv_mm = (int16_t)rint(CREATE_AXLE_LENGTH * fabs(rv) * 1e3);
  }
  else
  {
    // General case: convert rv to turn radius
    rad_mm = (int16_t)rint(tv_mm / rv);
    // The robot seems to turn very slowly with the above
    //rad_mm /= 2;
    //printf("real rad_mm: %d\n", rad_mm);
    rad_mm = MAX(rad_mm, -CREATE_RADIUS_MAX_MM);
    rad_mm = MIN(rad_mm, CREATE_RADIUS_MAX_MM);
  }
		uint8_t vel_h = (tv_mm & 0xFF00) >> 8;
		uint8_t vel_l = (tv_mm & 0x00FF);
		uint8_t deg_h = (rad_mm & 0xFF00) >> 8;
		uint8_t deg_l = (rad_mm & 0x00FF);
/*------------------*/
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
	cout << "---> Main Request <---\n";
	bool run = true;
	while(run) {
		pthread_testcancel();		// Check Player for cancel flag
		ProcessMessages();		// Tell Player to invoke ProcessMessage if new messages exist
		
		player_pose2d_t current_pos;
		player_pose2d_t current_vel;
		player_position2d_data_t message_data;

		//TEST VALUES
		current_pos.px = nav_pos[2];
		current_pos.py = nav_pos[3];
		current_pos.pa = nav_pos[1];
		current_vel.px = 0;
		current_vel.py = 0;
		current_vel.pa = 0;
		message_data.pos = current_pos;
		message_data.vel = current_vel;
		message_data.stall = 0;
		this->Publish(this->positionID, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,(void*)&message_data);

		     player_sonar_data_t irdata;
		     memset(&irdata,0,sizeof(irdata));

		     irdata.ranges_count = 8;
		     irdata.ranges = new float [irdata.ranges_count];
		     for(int i=0; i<8; i++)
		     	irdata.ranges[i] = (float)adcdata2m(sensors[i]);


		     this->Publish(this->sonarID,
			 PLAYER_MSGTYPE_DATA, PLAYER_SONAR_DATA_RANGES,
			 (void*)&irdata);
		     delete [] irdata.ranges;
		     

		
		
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
	if(rmbOPEN) {
		int n = write(fd,&b,1);
		if( n!=1)
			return -1;
		return 0;
	} else {
		cout << "---> CREATE DOESN'T EXIST D:<---\n";
		return 0;
	}
}

int rmbDriver::serialport_writebyte( int fd, uint8_t b, uint8_t sonic)
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

    // Reccomended settings
    toptions.c_cflag &= ~CRTSCTS;   // no flow control
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on read & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // Setting when read() releases
    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html (Still a little confusing)
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    // Apply settings
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}



/////////////////////////////////////
int rmbDriver::serialport_read_until(int fd, char* buf, char until, uint16_t* sensors)
{
    char b[1];
    uint8_t bb;
    int state = 0;
    int c=0;
    uint16_t data=0;
    uint8_t dh,dl;
    int error = 0;
    int noread = 0;
    do { 
        int n = read(fd, b, 1);  //Read byte at a time
        if(n==-1) { 
        	//printf("Global Error: %d ", errno); fflush(stdout);
        	//perror(strerror(errno));
         }
        else if( n==0 ) {
        	//No data available, yet... Hurry up Mike
            	//usleep( 10 * 1000 ); // wait 10 msec try again
            	continue;
        } else if(n>0) {
        	//Reset error counts
		noread = 0;
		error  = 0;
		
		//Shift in byte
		for(int i=3; i>0;i--) packet[i]=packet[i-1];
		*packet = 0xFF & *b;
		
		for(int i=(streamsize-1); i>0;i--) pstream[i]=pstream[i-1];
		*pstream = 0xFF & *b;
		
		//Parse data packet
		parse_packet(packet, sensors);

        }
    } while(1);


    return 0;
}

int rmbDriver::parse_packet(uint8_t* packet,uint16_t* sensors) {
	uint16_t high_byte = packet[0];
	uint16_t  low_byte = packet[1];
	uint16_t   payload = (0xFF00 & (high_byte << 8)) | (0x00FF & low_byte);
	
	if(  packet[3] == 'T' &&
	    (packet[2] >= '0' && packet[2] <= '9') &&
	      (payload >=  0  &&  payload  <= 4095) ) {
		sensors[packet[2]-0x30] = payload;
	       	return 0;
	} else {
		return -1;
	}
}



int parse_packet2(uint8_t* packet,uint16_t* sensors) {
	uint16_t high_byte = packet[0];
	uint16_t  low_byte = packet[1];
	uint16_t payload = (0xFF00 & (high_byte << 8)) | (0x00FF & low_byte);
	
	if(  packet[3] == 'T' &&
	    (packet[2] >= '0' && packet[2] <= '9') &&
	      (payload >=  0  &&  payload  <= 4095) ) {
		sensors[packet[2]-0x30] = payload;
	       	return 0;
	} else {
		return -1;
	}
}

void draw_screen(uint8_t* packet,uint16_t* sensors) {
	
	printf("\033[1JDATA STREAM: ");
	for(int i=0; i<streamsize; i++)
		printf("%x ", packet[i]);
	printf("\n");
	for(int i=0; i<8; i++) {
		printf("sensor %i: %f Meters\n", i, adcdata2m(sensors[i]));
	}
}

double adcdata2m(int adc){
	double m;
	m = adc*0.00049837;	// ((adc/4095)/0.0049)/100
					//adc/4095 -> volts    /0.0049 -> cm   /100 -> m
	return m;
}

double rmbDriver::adcdata2m(int adc){
	double m;
	m = adc*0.00049837;	// ((adc/4095)/0.0049)/100
					//adc/4095 -> volts    /0.0049 -> cm   /100 -> m
	return m;
}


int serialport_read_until2(int fd, char* buf, char until, uint16_t* sensors)
{
    char b[1];
    uint8_t bb;
    int state = 0;
    int c=0;
    uint16_t data=0;
    uint8_t dh,dl;
    int error = 0;
    int noread = 0;
    do { 
        int n = read(fd, b, 1);  //Read byte at a time
        if(n==-1) { 
        	//printf("Global Error: %d ", errno); fflush(stdout);
        	//perror(strerror(errno));
         }
        else if( n==0 ) {
        	//No data available, yet... Hurry up Mike
            	//usleep( 10 * 1000 ); // wait 10 msec try again
            	continue;
        } else if(n>0) {
        	//Reset error counts
		noread = 0;
		error  = 0;
		
		//Shift in byte
		for(int i=3; i>0;i--) packet[i]=packet[i-1];
		*packet = 0xFF & *b;
		
		for(int i=(streamsize-1); i>0;i--) pstream[i]=pstream[i-1];
		*pstream = 0xFF & *b;
		
		//Parse data packet
		parse_packet2(packet, sensors);

        }
    } while(1);


    return 0;
}














