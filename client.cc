/*-------------------------------------
       Name:    Tomasz Walczak
       Date:    Sept. 15, 2012
 
       File:    main.c
       Desc:    Main Function
    Project:    iRobot Server
--------------------------------------*/

#include <iostream>
#include <pthread.h>        //POSIX Threads Library

#include "network.h"        //TCP-IP Netowrk Class
#include "serial.h"         //UART Serial Class
#include "workerThread.h"   

#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <signal.h>

#define MAXRISE 255

using namespace std;
int readData( char* request, string* msg );
string _errormsg;
bool disconnect;

void sigint_call(int signum) {
	disconnect = true;
}

int main(int argc, char *argv[])
{
    disconnect = false;
    signal(SIGPIPE,SIG_IGN);
    signal(SIGINT, sigint_call);
    if(argc != 2)
    {
        cout << "Argument Error\nExiting...\n";
        exit(0);
    }
    
    
    string msg="";
    char* request = (char*)"hello";
    //cout << "n: ";
    //cout << flush;
	while(!disconnect) {
	    	readData(argv[1],&msg);
		printf("reconnect...\n");fflush(stdout);
		usleep(100*1000);
	}
	printf("Quit\n");
    //pthread_t networkThread;    // Start network worker thread
    //pthread_create(&networkThread,
     //              NULL, netWorker, (void *)NULL);
    
   // pthread_exit(NULL);         // Wait for threads
    return 0;
}

void setErrorMsg( char* msg )
{
    _errormsg = msg;
    cout << _errormsg;
}


int readData( char* request, string* msg )
{
    
    int _sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[MAXRISE];
    int count = 0;
    *msg = (char*)"";
    
    //CREATE SOCKET DISCRIPTOR-------------------------------------------
    portno = 8080;
    _sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (_sockfd < 0)
    {
        setErrorMsg((char*)"ERROR opening socket");
        return -1;
    }
    
    //SETUP ADDRESS INFORMATION-------------------------------------------
    server = gethostbyname("localhost");
    if ( server==NULL )
    {
        setErrorMsg((char*)"ERROR, no such host");
        return -1;
    }
    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy((char *)&serv_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    
    //SET SOCKET AS NON_BLOCKING--------------------------------------------
    fd_set fdset;
    struct timeval tv;
    int flags = fcntl(_sockfd, F_GETFL);
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);
    FD_ZERO(&fdset);
    FD_SET(_sockfd, &fdset);
    tv.tv_sec = 1;
    tv.tv_usec = 1000*1000;
    n=-1;
	int error_count = 0;
	while((!n==0) && !disconnect && error_count < 5) {
		n=connect(_sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr));
		if(n<0) error_count++;
		printf("CONNECT: %d\n",n);fflush(stdout);
		usleep(10*1000);
	}



	FD_ZERO(&fdset);
	FD_SET(_sockfd, &fdset);
	stringstream temp;
	memset(buffer,0,MAXRISE); n = 1;
	char message[10];
	*message = 'a';
	int n_write = 0;
	while((n_write > -1) && !disconnect){
		int test = 0;
		n=0;
		while(n<1 && test<250) {
			n = read(_sockfd,buffer,MAXRISE);
			usleep(1*1000); test++;
		}
		if (!FD_ISSET(_sockfd, &fdset))
			printf("   ---> fdset error\n");
			
		    // Set null character
		buffer[n > MAXRISE ? MAXRISE : n] = '\0';
		if(n>0) printf("received: %s\n", buffer);
		
		//usleep(1000*1000);
		n = write(_sockfd, (char*)"ton" , 3);
		if(n<0) perror("what");
		n_write = n;
		(*message)++;
		if (!FD_ISSET(_sockfd, &fdset))
			printf("   ---> fdset error\n");
	}
	if(n_write==-1) printf(" ---> WRITE ERROR\n");
   
	printf("closing socket...\n"); fflush(stdout);
	usleep(500*1000);
    close(_sockfd);

    return count;
}
