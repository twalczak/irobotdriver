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


monitor::monitor(QWidget *parent) : QWidget(parent, Qt::FramelessWindowHint)
{
	timer = new QTimer(this);
	timer->setInterval(100);
	connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	timer->start(); running=true;
	X=100;
	Y=100;
	robot1.start();
}

void monitor::paintEvent(QPaintEvent *event)
{	
	QFont font("Monaco");
	font.setPixelSize(fontSize);
	font.setStyleStrategy(QFont::NoAntialias);
	
	QColor c(255,0,0,255);
	
	QPainter p(this);
	p.setFont(font);
	p.setRenderHint(QPainter::Antialiasing, false);
	
	p.setPen(QPen(Qt::black, 1));				//BACKGROUND RECTANGLE
	p.setBrush(QBrush(Qt::black, Qt::SolidPattern));
	p.drawRect(0,0,width(),height());
	
	p.setPen(QPen(Qt::red, 1));						
	//p.drawPolygon(points, 4);
	//p.setPen(QPen(Qt::gray, 1, Qt::DashDotLine));	
	
	p.setPen(QPen(c, 1));
	
	p.drawRect(100,10,width()-110, height()-20);
	p.setPen(QPen(Qt::white, 1));
	p.drawRect(X,Y,10,10);		// Robot1
	p.drawLine(X+10,Y-2,X+20,Y-15);
	p.drawText(X+20+2,Y-15-2, "ROBOT1: OFFLINE");
	std::stringstream str_temp;
	str_temp << robot1.getx();
	
	p.drawText(3,fontSize, "ROBOT1: OFFLINE");
	p.drawText(3,fontSize*2, "  192.168.1.101: No response...");
	p.drawText(3,fontSize*3, str_temp.str().c_str());

	
	p.setPen(QPen(Qt::white, 1));
	
	p.drawLine(((width()-10)*0.5)+50,10,((width()-10)*0.5)+50,height()-10);
	
	p.setPen(QPen(Qt::gray, 1, Qt::DashDotLine));						//DRAW ROCKS
	p.setBrush(QBrush(Qt::black, Qt::SolidPattern));
	
}

void monitor::timerTimeout()
{
	//SHAPE SHIP FIGURE
	monitor::points[0] = QPointF(X,Y);
	monitor::points[1] = QPointF(X-(sin(theta+(0.75*M_PI))*shipSize),Y-(cos(theta+(M_PI*0.75))*shipSize));
	monitor::points[2] = QPointF(X-(sin(theta)*shipSize*2),Y-(cos(theta)*2*shipSize));
	monitor::points[3] = QPointF(X-(sin(theta+(5*M_PI)/4)*shipSize),Y-(cos(theta+(5*M_PI)/4)*shipSize));
	
	X = robot1.getx();
	Y = robot1.gety();
	
	//REDRAW WINDOW
	this->update();

}


void monitor::keyPressEvent(QKeyEvent *event)
{
	if(event->key()==Qt::Key_Space) shoot = true;
	if(event->key()==Qt::Key_Up) accel = true;
	if(event->key()==Qt::Key_Left) left = true;
	if(event->key()==Qt::Key_Right) right = true;
}

void monitor::keyReleaseEvent(QKeyEvent *event)
{
	if((event->key()==Qt::Key_Up)) accel = false;
	if((event->key()==Qt::Key_Left)) left = false;
	if((event->key()==Qt::Key_Right)) right = false;
	if((event->key()==Qt::Key_Space)) shoot = false;
}

int monitor::translate_x(double x) {
	return x;
}

int monitor::translate_y(double y) {
	return y;
}



/*-----------------------------------------------------------------
			ROBOT MEMBER FUNCTIONS
-----------------------------------------------------------------*/

void* Robot::network_thread(void) {
	while(1) { usleep(500000);
		std::cout << "RUNNING\n";
		parse((char*)"temp");
	}
	return 0;
}

void Robot::parse(char* data) {
	_x += 1.5;
	_y += 2;
	_a =  1.520;
	_v =  3760;
}


