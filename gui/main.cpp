#include "mainwindow.h"
#include <QtGui/QApplication>
#include <pthread.h>

<<<<<<< HEAD
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	KeyReceiver *key = new KeyReceiver();
	a.installEventFilter(key);
	MainWindow w;
	w.setKey(key);
	w.show();

	return a.exec();
=======
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    KeyReceiver *key = new KeyReceiver();
    a.installEventFilter(key);
    MainWindow w;
    w.setKey(key);
    w.show();
    return a.exec();
>>>>>>> 002c2a1f3dc841f56b998f90719b2f96dade691f
}
