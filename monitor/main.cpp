#include <QApplication>
#include "monitor.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	monitor *a = new monitor;
	a->show();
	return app.exec();
}