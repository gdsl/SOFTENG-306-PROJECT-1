#include "mainwindow.h"
#include <QtGui/QApplication>
#include <pthread.h>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	KeyReceiver *key = new KeyReceiver();
	a.installEventFilter(key);
	MainWindow w;
	w.setKey(key);
	w.show();

	return a.exec();
}
