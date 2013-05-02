#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include "monitor.h"
#include <QPainter>
#include <QtGui>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <string>
#define fontSize 8
#define shipSize 6

bool disconnect;

using namespace std;

monitor::monitor(QWidget *parent) : QWidget(parent, Qt::FramelessWindowHint) {
	timer = new QTimer(this);
	timer->setInterval(100);
	connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	timer->start(); running=true;
	X=100;
	Y=100;
	for(int i=0; i<CONSOLE_LINES; i++)
		memset((main_console)[i],0,sizeof(main_console[i]));
	std::stringstream ipaddress;
	for(int i=0; i<NUMBER_OF_ROBOTS; i++) {
		ipaddress.str(std::string());
		ipaddress << "192.168.1.10" << (i+1);
		robot[i].start(); //67.221.76.236
		robot[0].set_addr((char*)ipaddress.str().c_str());
	}
	robot[0].set_addr((char*)"67.221.76.236");
	int id = QFontDatabase::addApplicationFont(":/gui_img/consolas.ttf");
	cout << "id: " << id << '\n';
    QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    this->font = QFont(family);
    calculate_graphics();
    recalculate_timer = 0;

    grid_darker = QImage(":/gui_img/grid_darker.png");
    background = QImage(":/gui_img/sprites/background.png");
    robot_img = QImage(":/gui_img/sprites/robot.png");
}

void monitor::calculate_graphics(void) {
	/*     (0,0)
		-|-------
		 | .  .  .
		 | .  .  .
	*/
	right_column_width = 250;
	bottom_column_height = 100;

	// ARENA
	aX = 5; aY = 35;
	aW = (width() - right_column_width) - aX;
	aH = (height() - bottom_column_height) - aY;
	y_axis_X = (aW*0.5) + aX;
	x_axis_Y = (aH*0.5) + aY;
	arena_center_X = y_axis_X;
	arena_center_Y = x_axis_Y;
	ppm = 100; // pixels per meter

	// ROBOTS
	robot_radius = 5;

}

int monitor::translate_x(double x) {
	return (arena_center_X+(x*ppm));
}

int monitor::translate_y(double y) {
	return (arena_center_Y-(y*ppm));
	// Y-axis is reversed ?
}

// Arena: Meters-to-pixles
QPoint monitor::am2p(double x, double y) {
	return QPoint(translate_x(x),translate_y(y));
}

// Paints graphics: Trigered by timer and system calls
void monitor::paintEvent(QPaintEvent *event) {
	//QFont font("Consolas");
	QPainter p(this);
	p.setFont(font);
	printf("FONT(): %s\n", font.exactMatch() ? "true" : "false");
	p.setRenderHint(QPainter::Antialiasing, false);
	QColor cb(222,250,255,255);
	
	//	Background Rectangle
	p.setPen(QPen(Qt::black, 1));
	p.setBrush(QBrush(Qt::black));
	p.drawRect(0,0,width(),height());
	p.drawImage(0,0,background);

	// Title
	/*
	font.setPixelSize(14);
	p.setFont(font);
	p.setPen(QPen(cb, 1));
	p.drawText(5, 18, "monitor: SENSOR 0");
	p.setPen(QPen(Qt::gray, 10));
	p.drawLine(0,27,width(),27);/**/

	// Arena
	/*
	p.setPen(QPen(cb, 1));
	p.drawRect(aX,aY,aW,aH);
	p.setPen(QPen(Qt::gray, 1, Qt::DashDotDotLine));
	p.drawLine(y_axis_X,aY,y_axis_X,aY+aH);
	p.drawLine(aX,x_axis_Y,aX+aW,x_axis_Y);/**/

	// Arena Ticks
	/*
	for (int i=-2; i<=2; i++) {
		for (int j=-2; j<=2; j++) 
			p.drawLine(translate_x(j),translate_y(i),translate_x(j),translate_y(i)+3);
	}
	/**/

	// Robots
	p.setPen(QPen(Qt::white, 3, Qt::SolidLine));
	for(int i=0; i<NUMBER_OF_ROBOTS; i++) {
		double r_xM = robot[i].getx();
		double r_yM = robot[i].gety();
		QPoint r_loc = am2p(r_xM,r_yM);
		
		p.save();
		p.translate(r_loc);
		p.rotate(45);
		p.drawImage(QPoint(0,0),robot_img);
		p.drawText(r_loc.x()+robot_radius,r_loc.y()-robot_radius,"i");
		p.restore();
	}

	// Draw main_console
	font.setPixelSize(10);
	//font.setStyleStrategy(QFont::NoAntialias);
	p.setPen(QPen(cb, 1, Qt::SolidLine));
	p.setFont(font);
	for(int i=0; i<CONSOLE_LINES; i++)
		p.drawText(937, 105+(i*11)+8, main_console[i]);

	// Recalculate graphics variables
	recalculate_timer++;
	if(recalculate_timer > 10) {
		recalculate_timer = 0;
		calculate_graphics();
	}
}


void monitor::print_main(char* str) {
	for(int i=1; i<CONSOLE_LINES; i++)
		strncpy(main_console[i-1],main_console[i], STR_LEN-1);
	strncpy(main_console[CONSOLE_LINES-1],str, STR_LEN-1);
}

void monitor::timerTimeout() {
	X = robot[0].getx();
	Y = robot[0].gety();
	stringstream temp;
	temp << "[DATA] x:" << X << " y: " << Y;
	print_main((char*)temp.str().c_str());
	this->update();
}


void monitor::keyPressEvent(QKeyEvent *event) {
	if(event->key()==Qt::Key_Space) shoot = true;
	if(event->key()==Qt::Key_Up) accel = true;
	if(event->key()==Qt::Key_Left) left = true;
	if(event->key()==Qt::Key_Right) right = true;
}

void monitor::keyReleaseEvent(QKeyEvent *event) {
	if((event->key()==Qt::Key_Up))    accel = false;
	if((event->key()==Qt::Key_Left))  left = false;
	if((event->key()==Qt::Key_Right)) right = false;
	if((event->key()==Qt::Key_Space)) shoot = false;
}






/*-----------------------------------------------------------------
			ROBOT MEMBER FUNCTIONS
-----------------------------------------------------------------*/
Robot::Robot() {
		_v = 0;
		_x = 0;
		_y = 0;
		_a = 0;
		_run = false;
		disconnect = false;
		_connected = false;
		memset(_addr,0,25);
    	signal(SIGPIPE,SIG_IGN);
    	signal(SIGINT, Robot::sigint_call);
	}

void* Robot::network_thread(void) {
	disconnect = false;
	std::string msg="";
	while(!disconnect) {
		_connected = false;
	    readData((char*)"all",&msg);
		printf("reconnect...\n");fflush(stdout);
		usleep(100*1000);
	}
	pthread_exit(&_tid);
	return 0;
}

int Robot::readData( char* request, string* msg ) {
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
        //setErrorMsg((char*)"ERROR opening socket");
        return -1;
    }
    
    //SETUP ADDRESS INFORMATION-------------------------------------------
    server = gethostbyname(_addr);
    if ( server==NULL )
    {
        //setErrorMsg((char*)"ERROR, no such host");
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
	int n_write = 0;
	while((n_write > -1) && !disconnect){
		_connected = true;
		int test = 0;
		n=0;
		while(n<1 && test<250) {
			n = read(_sockfd,buffer,MAXRISE);
			usleep(1*1000); test++;
		}

		// Set null character
		buffer[n > MAXRISE ? MAXRISE : n] = '\0';
		parse(buffer);
		
		n = write(_sockfd, (char*)"all" , 3);
		if(n<0) perror("oh no");
		n_write = n;
	}
	if(n_write==-1) printf(" ---> WRITE ERROR\n");
   
	printf("closing socket...\n"); fflush(stdout);
	usleep(500*1000);
    close(_sockfd);

    return count;
}

void Robot::parse(char* data) {
	// Assumes the order that data is arranged
	// TODO: Make it more dynamic
	char* x_str;
	char* y_str;
	char* a_str;
	char* v_str;
	usonic.setMeters(0,3.456);
	printf("parse: %s\n", data);
	x_str = strtok (data,"~^|Ie");
	y_str = strtok (NULL,"~^|Ie");
	a_str = strtok (NULL,"~^|Ie");
	v_str = strtok (NULL,"~^|Ie");
	if(!((a_str == NULL) || (x_str == NULL) || (y_str == NULL) || (v_str == NULL))) {
		_a = atof(a_str);
		_x = atof(x_str);
		_y = atof(y_str);
		_v = atoi(v_str);
	}
}

void Robot::set_addr(char* a) {
	strncpy(_addr,a, 25);
	_addr[24] = 0;
}

void Robot::sigint_call(int signum){ disconnect = true; exit(0); }	

void* sensor_points::math_thread(void) {
	double sa[10];
	int i;
	for(i=0; i<10; i++) {
		sa[i] = (double)i*0.62831;
	}
	while(1) { usleep(1000);
		for(i=0; i<10; i++) {
			/*     RANGER      |-------- X --------------|  |------------ Y ------------| */
			start[i] = QPoint( cos(sa[i])*_meters[i],        sin(sa[i])*_meters[i]        );
			  end[i] = QPoint( cos(sa[i])*(_meters[i]+SDP),  sin(sa[i])*(_meters[i]+SDP)  );
		}
	}
}














