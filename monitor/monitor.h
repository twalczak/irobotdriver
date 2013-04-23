#ifndef MONITOR_H
#define MONITOR_H

#include <QWidget>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>


class Robot {	// Robot data collector class
public:
	Robot() {
		_v = 0;
		_x = 0;
		_y = 0;
		_a = 0;
	}
	void start(void) {	// Start data collection
		pthread_t tid;
		pthread_create(&tid, 0, Robot::call_network_thread, this);
	}
	
	uint16_t getv(void){ return _v; }	// Battery voltage
	double   getx(void){ return _x; }	// Localization coordinates
	double   gety(void){ return _y; }
	double   geta(void){ return _a; }
	
private:
	   /* Thread caller functions */
	static void* call_network_thread(void *arg) { 
		return ((Robot*)arg)->network_thread();		// Connect thread to function with access to class
	}
	void* network_thread(void);	// Called by thread function on pthread creation
	void  parse(char* data);
	
	uint16_t _v;
	double   _x, _y, _a;
};

class monitor : public QWidget {	// QTCore window class
	Q_OBJECT
public:
	monitor(QWidget *parent = 0);
	Robot robot1;

protected:
	void paintEvent(QPaintEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	
private slots:
	void timerTimeout();

private:
	//KEYPRESS
	bool left, right, accel, shoot;
	QTimer *timer;
	QTimer *checkScore;
	QTimer *RESET;
	bool resetScreen;
	double X, Y;
	double theta;
	double shipSpeed, shipSpeedX, shipSpeedY;
	QString str, str2;
	QPointF points[4];
	QPointF rockShape[5];
	int delayFire;
	int countRocks;
	int lives;
	bool restart;
	bool running;
	
};



#endif
