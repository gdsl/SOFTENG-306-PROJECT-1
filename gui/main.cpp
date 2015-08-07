#include "mainwindow.h"

#include <QtGui/QApplication>


int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	MainWindow mainWindow;
	mainWindow.setFixedSize(500,300);
	mainWindow.show();
	return app.exec();
} 
