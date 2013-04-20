#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include "monitor.h"
#include <QPainter>
#include <QtGui>
#include <math.h>
#include <stdio.h>

#define fontSize 9
#define shipSize 6


monitor::monitor(QWidget *parent) : QWidget(parent, Qt::FramelessWindowHint)
{
	timer = new QTimer(this);
	timer->setInterval(10);
	connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	timer->start(); running=true;
	X=100;
	Y=100;
}

void monitor::paintEvent(QPaintEvent *event)
{	
	QFont font("Lucida Console");
	font.setPixelSize(fontSize);
	font.setStyleStrategy(QFont::NoAntialias);
	
	//QColor c(255,255,255,255);
	
	QPainter p(this);
	p.setFont(font);
	//p.setRenderHint(QPainter::Antialiasing, false);
	
	p.setPen(QPen(Qt::black, 1));						//BACKGROUND RECTANGLE
	p.setBrush(QBrush(Qt::black, Qt::SolidPattern));
	p.drawRect(0,0,width(),height());
	
	p.setPen(QPen(Qt::white, 1));						
	p.drawPolygon(points, 4);
	
	p.drawText(3,fontSize, "ROBOT1: OFFLINE");
	p.drawText(3,fontSize*2, "  192.168.1.101: No response...");


	
	p.setPen(QPen(Qt::white, 1));

	
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