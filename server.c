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
#include <unistd.h>

int SocketFD;

unsigned char c2h(char c1, char c2)
{
    uint16_t hex_high,hex_low,out;
    
    if(c1>='a' && c1<='f')
        hex_high=c1-87;
    else if(c1>='0' && c1<='9')
        hex_high=c1-48;
    
    if(c2>='a' && c2<='f')
        hex_low=c2-87;
    else if(c2>='0' && c2<='9')
        hex_low=c2-48;
    
    return (uint16_t)(hex_high<<4)|(uint16_t)(hex_low);
}

int valid(char c)
{
    if((c>='a' && c<='f') || (c>='0' && c<='9'))
        return 1;
    else
        return 0;
}

int parseCommand(char* msg)
{
    unsigned char hex;          //HEX VALUE
    char hc1,hc2;               //HEX CHAR
    int len = strlen(msg);
    uint8_t serial_message[20];
    uint8_t sm_count=-1;         //serial_message pointer position
    
    
    memset(serial_message,0,sizeof serial_message);
    
    printf("\n    L:%i",len);
    
    for (int i=0; i<len; i++) {
        
        // LOOK FOR HEX START
        if(msg[i]=='x')
        {
            //CHECK+READ NEXT TWO CHARS
            if(!((i+2)<len)) return 1;
            hc1=msg[i+1];
            hc2=msg[i+2];
            
            //CHECK FOR VALID CHAR
            if(!valid(hc1)) return 1;
            if(!valid(hc2)) return 1;
            
            //GET HEX VALUE FROM STRING
            hex = c2h(hc1,hc2);
            
            //ADD TO SERIAL COMMAND STRING
            sm_count++;
            serial_message[sm_count]=hex;
        }
    }
    
    printf("\n");
    for(int i=0;i<=sm_count;i++)
        printf("    c%d: %d\n",i,serial_message[i]);
    
    return 0;
}


void signal_callback_handler(int signum)
{
    
    printf("\n\n    RECEIVED SIGINT: Attempting to close sockets D:\n\n");
    close(SocketFD);
    printf("    BYE!!!\n\n");
    exit(signum);
    
}

    


int main(void)
{
    signal(SIGINT, signal_callback_handler);
    char buff[50] = "nothing";
    struct sockaddr_in stSockAddr;
    SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    if(-1 == SocketFD)
    {
        perror("can not create socket");
        exit(EXIT_FAILURE);
    }
    
    memset(&stSockAddr, 0, sizeof(stSockAddr));
    
    stSockAddr.sin_family = AF_INET;
    stSockAddr.sin_port = htons(8080);
    stSockAddr.sin_addr.s_addr = INADDR_ANY;
    
    if(-1 == bind(SocketFD,(struct sockaddr *)&stSockAddr, sizeof(stSockAddr)))
    {
        perror("error bind failed");
        close(SocketFD);
        exit(EXIT_FAILURE);
    }
    
    if(-1 == listen(SocketFD, 10))
    {
        perror("error listen failed");
        close(SocketFD);
        exit(EXIT_FAILURE);
    }
    
    for(;;)
    {
        printf("waiting for message: ");
        fflush(stdout);
        int ConnectFD = accept(SocketFD, NULL, NULL);
        
        if(0 > ConnectFD)
        {
            perror("error accept failed");
            close(SocketFD);
            exit(EXIT_FAILURE);
        }
    
        read(ConnectFD,buff,50);
        printf("\e[1;33m%s\e[0m parsing...",buff);fflush(stdout);
        
        
        if(!parseCommand(buff))
            printf("    \e[1;32mdone\e[0m\n");
        else
            printf("    \e[1;31mfailed\e[0m\n");
        
        memset(buff,0,sizeof buff);
        
        if (-1 == shutdown(ConnectFD, SHUT_RDWR))
        {
            perror("can not shutdown socket");
            close(ConnectFD);
            exit(EXIT_FAILURE);
        }
        close(ConnectFD);
    }
    return EXIT_SUCCESS;  
}
