#include <QApplication>
#include "monitor.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	monitor *a = new monitor;
	if(argc > 1)
		if(!strcmp(argv[1],(char*)"max")) a->setWindowState(Qt::WindowMaximized);
	a->show();
	return app.exec();
}
