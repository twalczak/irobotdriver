#ifndef MONITOR_H
#define MONITOR_H

#include <QWidget>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#define CONSOLE_LINES 20
#define STR_LEN 50


class Robot {	// Robot data collector class
public:
	Robot() {
		_v = 0;
		_x = 0;
		_y = 0;
		_a = 0;
		_run = false;
	}
	void start(void) {	// Start data collection
		_run = true;
		pthread_create(&_tid, 0, Robot::call_network_thread, this);
	}
	
	void stop(void) {
		_run = false;
		pthread_detach(_tid);
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
	bool _run;
	pthread_t _tid;
};

/* --------------------------------------------------------------------------------------  */

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
	QFont font;
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
	int recalculate_timer;
	bool restart;
	bool running;
	int translate_x(double x);
	int translate_y(double y);
	QPoint am2p(double x, double y); // Arena: Meters-to-pixles
	void calculate_graphics(void);
	char main_console[CONSOLE_LINES][STR_LEN];
	void print_main(char* str);

	// Graphics variables
	int aX, aY;
	int aW, aH;
	int x_axis_Y;
	int y_axis_X;
	int right_column_width;
	int bottom_column_height;
	int robot_radius;
	int ppm;
	int arena_center_X, arena_center_Y;
	
};



#endif
