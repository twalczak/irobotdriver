#ifndef MONITOR_H
#define MONITOR_H

#include <QWidget>
#include <iostream>
#include <pthread.h>
#include <unistd.h>


class Robot  {
public:
	Robot(){
	};
	static void*  call_network_thread(void *arg) { return ((Robot*)arg)->network_thread(); }
	void* network_thread(void);
	void start(void){
		pthread_t tid;
		int result;
		result = pthread_create(&tid, 0, Robot::call_network_thread, this);
		if (result == 0)
			pthread_detach(tid);
	};
};


class monitor : public QWidget
{
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