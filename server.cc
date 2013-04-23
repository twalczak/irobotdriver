//
//  server.c
//  
//
//  Created by Tomasz Walczak on 10/24/12.
//  QUICK EXAMPLE FOR GUPTA
//

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>

int SocketFD;
int ConnectFD;

void signal_callback_handler(int signum) {
    printf("\n\n    RECEIVED SIGINT: Attempting to close sockets D:\n\n");
    close(ConnectFD);
    close(SocketFD);
    printf("    BYE!!!\n\n");
    exit(signum);
    
}

void sigpipe_callback(int signum) {
	printf("  ---> SIGPIPE ERROR <---\n"); fflush(stdout);
}

int main(void)
{
	signal(SIGINT, signal_callback_handler);
	signal(SIGPIPE, sigpipe_callback);
    	char buff[50] = "nothing";

	uint16_t vel = 234;
	double   dvel = 4.5675;
	char output[100]; 
	memset(output, 0, sizeof(output));
	
	bool failed = true;
	while(failed) {
		failed = false;
		struct sockaddr_in stSockAddr;
		SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

		fd_set fdset;
		struct timeval tv;
		int flags = fcntl(SocketFD, F_GETFL);
		fcntl(SocketFD, F_SETFL, O_NONBLOCK);
		FD_ZERO(&fdset);
		FD_SET(SocketFD, &fdset);
		tv.tv_sec = 1;
		tv.tv_usec = 1000*1000;

		if(-1 == SocketFD)
		{
			perror("^[31mcan not create socket^[0m");
			failed = true;
			//exit(EXIT_FAILURE);
		}

		memset(&stSockAddr, 0, sizeof(stSockAddr));

		stSockAddr.sin_family = AF_INET;
		stSockAddr.sin_port = htons(8080);
		stSockAddr.sin_addr.s_addr = INADDR_ANY;

		if(-1 == bind(SocketFD,(struct sockaddr *)&stSockAddr, sizeof(stSockAddr)))
		{
			perror("error bind failed");
			failed = true;
			close(SocketFD);
			//exit(EXIT_FAILURE);
		}

		if(-1 == listen(SocketFD, 10))
		{
			perror("error listen failed");
			failed = true;
			close(SocketFD);
			//exit(EXIT_FAILURE);
		}
		printf("Creating socket: %s\e[0m\n", (char*)(failed ? "\e[1;31mfailed" : "\e[1;32msuccessful"));
		usleep(500*1000);
	}

   	int n;
	int read_err = 0;
    for(;;)
    {
		printf("waiting for client:\n"); fflush(stdout);
		ConnectFD=-1;
		while(ConnectFD<0) {
			ConnectFD = accept(SocketFD, NULL, NULL);
			usleep(250000);
		}

		fd_set fdsetc;
		struct timeval tvc;
		int flagsc = fcntl(ConnectFD, F_GETFL);
		fcntl(ConnectFD, F_SETFL, O_NONBLOCK);
		FD_ZERO(&fdsetc);
		FD_SET(ConnectFD, &fdsetc);
		tvc.tv_sec = 1;
		tvc.tv_usec = 1000*1000;
		std::string str2;
		std::stringstream temp_out;
		read_err = 0;
		while( read_err < 3 ){
			printf("waiting for message: ");fflush(stdout);
			do {
				n=read(ConnectFD,buff,50); 
				usleep(10*1000);
			} while(n<0);

			if(n==0) read_err++;
			else read_err = 0;
			buff[ n<0 ? 0 : (n>49 ? 49 : n) ] = 0;
			str2 = buff;
			printf("%s\n", n==0 ? (char*)"EOF" : "Parsing...");fflush(stdout);

			if(str2.compare(0,3,(char*)"tom")==0) { 
				n=write(ConnectFD, (char*)"heyheyhey",6);
			}
			else {
				temp_out.str(std::string());
				temp_out << dvel << "|";
				temp_out << vel  << "|e";
				memset(output, 0, sizeof(output));
				memcpy(output,temp_out.str().c_str(),(temp_out.str().size() > (sizeof(output)-1) ? sizeof(output)-1 : temp_out.str().size()));
				printf("output: %s\n", output);
				n=write(ConnectFD, output,sizeof(output));
			}
			memset(buff,0,sizeof buff);
		}

		printf("CLOSING CONNECTION\n");fflush(stdout);
		if (-1 == shutdown(ConnectFD, SHUT_RDWR)) {
			perror("can not shutdown socket");
		}
		close(ConnectFD);
    }
    return EXIT_SUCCESS;  
}
