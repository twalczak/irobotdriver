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

#define fontSize 8
#define shipSize 6
#define NUMBER_OF_ROBOTS 2;


monitor::monitor(QWidget *parent) : QWidget(parent, Qt::FramelessWindowHint)
{
	timer = new QTimer(this);
	timer->setInterval(100);
	connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	timer->start(); running=true;
	X=100;
	Y=100;
	for(int i=0; i<CONSOLE_LINES; i++)
		memset((main_console)[i],0,sizeof(main_console[i]));
	robot1.start();
	int id = QFontDatabase::addApplicationFont(":/gui_img/lucon.ttf");
    QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    QFont font(family);
    calculate_graphics();
    recalculate_timer = 0;
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
	QPainter p(this);
	p.setFont(font);
	p.setRenderHint(QPainter::Antialiasing, false);
	
	//	Background Rectangle
	p.setPen(QPen(Qt::black, 1));
	p.setBrush(QBrush(Qt::black, Qt::SolidPattern));
	p.drawRect(0,0,width(),height());

	// Title
	font.setPixelSize(14);
	p.setFont(font);
	p.setPen(QPen(Qt::white, 1));
	p.drawText(5, 18, "MONITOR");
	p.setPen(QPen(Qt::gray, 10));
	p.drawLine(0,27,width(),27);

	// Arena
	p.setPen(QPen(Qt::white/*QColor(95, 192, 206, 255)*/, 1));
	p.drawRect(aX,aY,aW,aH);
	p.setPen(QPen(Qt::gray, 1, Qt::DashDotDotLine));
	p.drawLine(y_axis_X,aY,y_axis_X,aY+aH);
	p.drawLine(aX,x_axis_Y,aX+aW,x_axis_Y);

	// Arena Ticks
	for (int i=-2; i<=2; i++) {
		for (int j=-2; j<=2; j++) 
			p.drawLine(translate_x(j),translate_y(i),translate_x(j),translate_y(i)+3);
	}

	// Robots
	p.setPen(QPen(Qt::white, 3, Qt::SolidLine));
	p.drawEllipse(am2p(X,Y),robot_radius,robot_radius);

	// Draw main_console
	font.setPixelSize(9);
	font.setStyleStrategy(QFont::NoAntialias);
	p.setPen(QPen(Qt::white, 1, Qt::SolidLine));
	p.setFont(font);
	for(int i=0; i<CONSOLE_LINES; i++)
		p.drawText(aX+aW+5, (aY+5)+(i*11)+8, main_console[i]);

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

	for(int i=0; i<CONSOLE_LINES; i++)
		printf("     %s\n", main_console[i]);
	printf("DONE\n");
}

void monitor::timerTimeout() {
	X = robot1.getx();
	Y = robot1.gety();
	print_main((char*)"CONNECT: -1");
	//REDRAW WINDOW
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

void* Robot::network_thread(void) {
	while(_run) { usleep(500000);
		parse((char*)"temp");
	}
	pthread_exit(&_tid);
	return 0;
}

void Robot::parse(char* data) {
	_x += 0.07;
	_y += 0.01;
	_a =  1.520;
	_v =  3760;
}


