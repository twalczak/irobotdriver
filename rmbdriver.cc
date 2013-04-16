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
#if !defined (WIN32)
    #include <unistd.h>
#endif

/* -------------------------------------------------------------- */
#include "serial.h"
#include "rmbdriver.h"
#include "irobot.h"
#define streamsize 20
#define packetsize 40

using namespace std;
char packetn[packetsize];
uint8_t packet[4];
int iro_fd, uso_fd, nav_fd;
uint8_t pstream[streamsize];
uint16_t sensors[10];
pthread_t nav_thread_id;
pthread_t irobot_control_thread_id;
pthread_t usonic_thread_id;
pthread_t screen_thread_id;
double nav_pos[5];
void draw_screen(uint8_t* packet,uint16_t* sensors);
double adcdata2m(int adc);
int return_i(int _i);
double adcdata2m_temp(int adc);
int serialport_read_until2(int fd, char* buf, char until, uint16_t* sensors);
int parse_packet2(uint8_t* packet,uint16_t* sensors);


int return_i(int _i) {
	     if(_i==7) return 9;
	else if(_i==9) return 7;
	return _i;
}


void* rmbDriver::screen_thread(void* arg) {
	while(client_connected) {
		printf("\033[1J Client Connected");
		for(int i=0; i<8; i++) {
			printf("sensor %i: %f Meters\n", i, adcdata2m_temp(sensors[i]));
		}

		printf("\nLOCAL:\n   x: %f\n   y: %f\n   a: %f\n",nav_pos[2],nav_pos[3],nav_pos[1]);
		printf("\nSPEED:\n  dV: %f\n  dR: %f\n",dvdt_global,drdt_global);
		mdelay(250);
	}
	printf("\nClient Disconnected\n");
	pthread_exit(&screen_thread_id);
}


void* rmbDriver::usonic_thread(void* arg) {
	char* buffer;
	while(client_connected) {
		serialport_read_until2(uso_fd,buffer, 'i', sensors);
	}
	pthread_exit(&usonic_thread_id);
}

void* rmbDriver::irobot_control_thread(void* arg) {
    while(client_connected) {
        //if(!irobot_responding())
        //    irobot_init();
    	if(_irobot_count>5) {
    		_irobot_count = 0;
    		serialport_writebyte(iro_fd, 132); //printf("Sending OP_CODE_FULL ");
    		mdelay(60);
    	}
    	_irobot_count++;


        irobot_setspeed(&iro_fd, dvdt_global, drdt_global);
        //printf("Sending dVdt: %f dRdt: %f\n",dvdt_global,drdt_global);
        mdelay(100); // ms
    }
	printf("Exiting irobot thread...\n");fflush(stdout);
	irobot_setspeed(&iro_fd,0.0,0.0);
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
            //printf("packet: %s \n", packetn);
            r_id_str = strtok (packetn,"~^|I");
            a_str = strtok (NULL,"~^|I");
            x_str = strtok (NULL,"~^|I");
            y_str = strtok (NULL,"~^|I");
            z_str = strtok (NULL,"~^|I");

            if(!((r_id_str == NULL) || (a_str == NULL) || (x_str == NULL) || (y_str == NULL) || (z_str == NULL))) {
                nav_pos[0] = atof(r_id_str);
                nav_pos[1] = -M_PI/180 * atof(a_str);
                nav_pos[2] = 0.01 * atof(x_str);
                nav_pos[3] = 0.01 * atof(y_str);
                nav_pos[4] = 0.01 * atof(z_str);

                //printf("DATA: id: %f a: %f x: %f y: %f z: %f\n",id,a,x,y,z);
            } else {
                //printf("DATA: NULL\n");fflush(stdout);
            }

            memset(packetn,0,sizeof(packetn));
            count=0;
        }
        packetn[count] = *b;
        count++;

        }
    } while(client_connected);
    pthread_exit(&nav_thread_id);

    return 0;
}

rmbDriver::rmbDriver(ConfigFile* cf, int section) : ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN) {
    
    if(cf->ReadDeviceAddr(&positionID, section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0) {
        if(AddInterface(positionID) != 0) {
            PLAYER_ERROR("Error adding position2d interface\n");
            SetError(-1);
            return;
        }
    }
    
    if(cf->ReadDeviceAddr(&sonarID, section, "provides", PLAYER_SONAR_CODE, -1, NULL) == 0) {
        if(AddInterface(sonarID) != 0) {
            PLAYER_ERROR("Error adding sonar interface\n");
            SetError(-1);
            return;
        }
    } 
    
    if(cf->ReadDeviceAddr(&laserID, section, "provides", PLAYER_LASER_CODE, -1, NULL) == 0) {
        if(AddInterface(laserID) != 0) {
            PLAYER_ERROR("Error adding laser interface\n");
            SetError(-1);
            return;
        }
    } 
    
    this->irobot_port = cf->ReadInt(section, "irobot_port", 0); // Read variable in .cfg file
    this->nav_port = cf->ReadInt(section, "nav_port", 0); // Read variable in .cfg file
    this->usonic_port = cf->ReadInt(section, "usonic_port", 0); // Read variable in .cfg file

    client_connected = false;
    _irobot_count = 0;

    serial_port_str[11] = 0x30 + 5; /*Update port number*/
    printf("iROBOT: %d\nLocal: %d\nuSonic: %d\n", irobot_port, nav_port,usonic_port);
	
    return; 
}

int rmbDriver::MainSetup() {
    int baudrate = B57600;  // iROBOT CREATE
    serial_port_str[11] = 0x30 + irobot_port;
    //iro_fd = serialport_init((char*)serial_port_str, baudrate);
    iro_fd = serialport_init("/dev/tty_irobot", baudrate);
    if(iro_fd==-1) {
        PLAYER_ERROR("Fatal Error: Serial port failed to initialize");
        return -1; 
    }

    baudrate = B115200; // LOCALIZATION SYSTEM
    serial_port_str[11] = 0x30 + nav_port;
    //nav_fd = serialport_init((char*)serial_port_str, baudrate);
    nav_fd = serialport_init("/dev/tty_nav", baudrate);
    if(nav_fd==-1) {
        PLAYER_ERROR("NAVIGATION PORT FAILED");
        cout << "Localization Port ERROR!\n";
        return -1; 
    }

    baudrate = B9600;  // ULTRA-SONIC SENSORS
    serial_port_str[11] = 0x30 + usonic_port;
    //uso_fd = serialport_init((char*)serial_port_str, baudrate);
    uso_fd = serialport_init("/dev/tty_usonic", baudrate);
    if(uso_fd==-1) {
        cout << "uSonic port: error\n";
        return -1;
    }

    client_connected = true;
    _irobot_count = 0;

    printf("START SCI: 128\n");
	serialport_writebyte(iro_fd,128);   // TODO: Define opcodes in header
	mdelay(500);
	printf("START FULL: 132\n");
	serialport_writebyte(iro_fd,132); 
	mdelay(500);
	printf("ready\n");

	pthread_create(&irobot_control_thread_id, NULL, &rmbDriver::irobot_control_thread, (void*)NULL);
    pthread_create(&nav_thread_id, NULL, &rmbDriver::nav_thread, (void*)NULL);
    pthread_create(&usonic_thread_id, NULL, &rmbDriver::usonic_thread, (void*)NULL);
    pthread_create(&screen_thread_id, NULL, &rmbDriver::screen_thread, (void*)NULL);

	return 0;
}

void rmbDriver::MainQuit() {
    // TODO: Close gracefuly
	client_connected = false;
	mdelay(400);	
	printf("Trying to stop... "); fflush(stdout);
	irobot_setspeed(&iro_fd,0.0,0.0); mdelay(300);
	printf("done\nClosing all ports... "); fflush(stdout);
	close(iro_fd);
	close(uso_fd);
	close(nav_fd);
	printf("done\n   [DRIVER QUIT]\n"); fflush(stdout);
}

int rmbDriver::ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void * data) {
	//printf("---> Message Received <---\n        TYPE: %d SUBTYPE: %d SIZE: %d\n", hdr->type, hdr->subtype, hdr->size);
	
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM, this->sonarID)) {
		cout << "They want geometry D:\n";
		
		player_pose3d_t poses[10];
		uint32_t poses_count = 10;
		poses[0].px = 0.34;
		poses[0].py = 0.247;
		poses[0].pz = 0.05;
		poses[0].proll = 0.0;
		poses[0].ppitch = 0.0;
		poses[0].pyaw = 0.628;
		
		poses[1].px = 0.13;
		poses[1].py = 0.399;
		poses[1].pz = 0.05;
		poses[1].proll = 0.0;
		poses[1].ppitch = 0.0;
		poses[1].pyaw = 1.2566;
		
		poses[2].px = -0.13;
		poses[2].py = 0.399;
		poses[2].pz = 0.05;
		poses[2].proll = 0.0;
		poses[2].ppitch = 0.0;
		poses[2].pyaw = 1.885;
		
		poses[3].px = -0.34;
		poses[3].py = 0.247;
		poses[3].pz = 0.05;
		poses[3].proll = 0.0;
		poses[3].ppitch = 0.0;
		poses[3].pyaw = 2.513;
		
		poses[4].px = -0.42;
		poses[4].py = 0.0;
		poses[4].pz = 0.05;
		poses[4].proll = 0.0;
		poses[4].ppitch = 0.0;
		poses[4].pyaw = 3.14;
		
		poses[5].px = -0.34;
		poses[5].py = -0.247;
		poses[5].pz = 0.05;
		poses[5].proll = 0.0;
		poses[5].ppitch = 0.0;
		poses[5].pyaw = 3.77;
		
		poses[6].px = -0.13;
		poses[6].py = -0.399;
		poses[6].pz = 0.05;
		poses[6].proll = 0.0;
		poses[6].ppitch = 0.0;
		poses[6].pyaw = 4.398;
		
		poses[7].px = 0.13;
		poses[7].py = -0.399;
		poses[7].pz = 0.05;
		poses[7].proll = 0.0;
		poses[7].ppitch = 0.0;
		poses[7].pyaw = 5.03;

		poses[8].px = 0.34;
		poses[8].py = -0.247;
		poses[8].pz = 0.05;
		poses[8].proll = 0.0;
		poses[8].ppitch = 0.0;
		poses[8].pyaw = 5.655;

		poses[9].px = 0.420;
		poses[9].py = 0.0;
		poses[9].pz = 0.05;
		poses[9].proll = 0.0;
		poses[9].ppitch = 0.0;
		poses[9].pyaw = 6.283;
		
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
		dvdt_global = pose_data.px;		//UPDATE GLOBALS
		drdt_global = pose_data.pa;
		return 0;
	}
	return -1;
}

void rmbDriver::Main() {
	cout << "---> Main Request <---\n";
	bool run = true;
	while(client_connected) {
		pthread_testcancel();	// Check Player for cancel flag
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
	    irdata.ranges_count = 10;
	    irdata.ranges = new float [irdata.ranges_count];
	    for(int i=0; i<10; i++)
	     	irdata.ranges[i] = (float)adcdata2m(sensors[return_i(i)]);
	    this->Publish(this->sonarID,PLAYER_MSGTYPE_DATA, PLAYER_SONAR_DATA_RANGES,(void*)&irdata);
	    delete [] irdata.ranges;
		    
		usleep(100);			// Sleep thread
	}
}









/*-----------------------------------------------------------------
        STUFF FOR CLEAN UP
-------------------------------------------------------------------*/



/*int rmbDriver::serialport_writebyte( int fd, uint8_t b)
{
	int n = write(fd,&b,1);
	if( n!=1)
		return -1;
	return 0;
}*/




/////////////////////////////////////
/*int rmbDriver::serialport_read_until(int fd, char* buf, char until, uint16_t* sensors)
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
}*/



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


double adcdata2m(int adc){
	double m;
	m = adc*0.0025;	// ((adc/4095)/0.0049)/100
					//adc/4095 -> volts    /0.0049 -> cm   /100 -> m
	return m;
}

double adcdata2m_temp(int adc){
	double m;
	m = adc*0.0025;	// ((adc/4095)/0.0049)/100
					//adc/4095 -> volts    /0.0049 -> cm   /100 -> m
	return m;
}

double rmbDriver::adcdata2m(int adc){
	double m;
	m = adc*0.0025;	// ((adc/4095)/0.0049)/100
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
    } while(client_connected);


    return 0;
}














