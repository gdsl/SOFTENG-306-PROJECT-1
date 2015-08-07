#include "mainwindow.h"

#include <QtCore/QCoreApplication>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	// Create the button, make "this" the parent
	m_button = new QPushButton("Run simulator", this);
	// set size and location of the button
	m_button->setGeometry(QRect(QPoint(100, 100),
	QSize(200, 50)));
	move(200,200);
	// Connect button signal to appropriate slot
	connect(m_button, SIGNAL (released()), this, SLOT (handleButton()));
	isRunningSimulator = false;
}

void MainWindow::handleButton()
{
	if (!isRunningSimulator) {
		isRunningSimulator = true;
		// change the text
		m_button->setText("Stop simulator");
		//launch roslaunch
		system("roslaunch se306project orchard.launch &");
	} else {
		isRunningSimulator = false;
		// change the text
		m_button->setText("Run simulator");
		//close roslaunch
		system("pkill stage");
	}
}
