#include <iostream>
#include <string.h>
#include <cstdio>

#include <sys/types.h>
#include <pthread.h>
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

#define READT 0
#define READS 1
#define	READH 2
#define READL 3
#define IDLE 5
#define streamsize 20
#define packetsize 40

using namespace std;

int fdd;
uint8_t packet[packetsize];
uint8_t pstream[streamsize];
uint16_t sensors[8];
int serialport_init(const char* serialport, int baud);
int serialport_read_until(int fd, char* buf, char until, uint16_t* sensors);
int parse_packet(uint8_t* packet,uint16_t* sensors);
void draw_screen(uint8_t* packet,uint16_t* sensors);

void* screen_thread(void* arg) {
	while(1) {
		draw_screen(pstream,sensors);
		usleep(100*1000);
	}
}

double adcdata2m(int adc){
	double m;
	//m = adc*0.00049837;	
	m = adc*0.01;
	return m;
}

int main(void) {

	pthread_t screen_thread_id;
	int flags;
	int baudrate = B9600;  // default
    	fdd = serialport_init((char*)"/dev/ttyUSB0", baudrate);
    	if(fdd==-1) {
    		cout << "Open port: error\n";
        	return -1;
        }
        
        
 	if (-1 == (flags = fcntl(fdd, F_GETFL, 0)))
 	       flags = 0;
    	fcntl(fdd, F_SETFL, flags | O_NONBLOCK);
        
        //pthread_create(&screen_thread_id, NULL, screen_thread, (void*)NULL);
        
         char* buffer;
	serialport_read_until(fdd,buffer, 'i', sensors);
	return 0;
}

int serialport_read_until(int fd, char* buf, char until, uint16_t* sensors)
{
    char b[1];
    uint8_t bb;
    int state = 0;
    int c=0;
    uint16_t data=0;
    uint8_t dh,dl;
    int error = 0;
    int noread = 0;
    int count = 0;

    char* r_id_str;
    char* a_str;
    char* x_str;
    char* y_str;
    char* z_str;

    double id,a,x,y,z;

    memset(packet,0,sizeof(packet));
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
		printf("TEST: %c\n",*b);
		//Shift in byte

        if(*b=='~' || count>(packetsize-2)) {
            memset(packet,0,sizeof(packet));
            count=0;
        }


        if(*b=='`') {
            printf("packet: %s \n", packet);
            r_id_str = strtok (packet,"~^|I");
            a_str = strtok (NULL,"~^|I");
            x_str = strtok (NULL,"~^|I");
            y_str = strtok (NULL,"~^|I");
            z_str = strtok (NULL,"~^|I");

            if(!((r_id_str == NULL) || (a_str == NULL) || (x_str == NULL) || (y_str == NULL) || (z_str == NULL))) {
                id = atof(r_id_str);
                a  = atof(a_str);
                x  = atof(x_str);
                y  = atof(y_str);
                z  = atof(z_str);

                printf("DATA: id: %f a: %f x: %f y: %f z: %f\n",id,a,x,y,z);
            } else {
                printf("DATA: NULL\n");fflush(stdout);
            }

            memset(packet,0,sizeof(packet));
            count=0;
        }
        packet[count] = *b;
        count++;

        }
    } while(1);


    return 0;
}

int parse_packet(uint8_t* packet,uint16_t* sensors) {
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
		printf("sensor %i: %f Meters | INT=%d\n", i, adcdata2m(sensors[i]), sensors[i]);
	}
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
    case 2400:   brate=B2400;   break;
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


