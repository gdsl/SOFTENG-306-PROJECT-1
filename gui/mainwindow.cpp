#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "worker.h"
#include <QThread>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QThread *thread = new QThread(this);
    Worker *worker = new Worker();
    worker->moveToThread(thread);
 	connect (thread, SIGNAL(started()), worker, SLOT(newLabel()));
    connect(worker, SIGNAL(requestNewLabel(QString)), this, SLOT(onUpdateGUI(QString)));
    connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));

	
	thread->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}


//method which calls the rostopic command
//void *print_message_function( void *ptr )
//{
//	exec("gui/getStatus.sh");
//}

void MainWindow::onUpdateGUI( QString str )
{
    ui->robotPanel1->setPlainText(str);
}

void MainWindow::on_launchButton_clicked()
{
	//launch roslaunch
	system("roslaunch se306project orchard.launch &");
	
	//create subscriber thread
	//pthread_t t1;
	//pthread_create (&t1, NULL, print_message_function, NULL);

	emit MainWindow::requestProcess();
	qDebug("THREAD2 STARTED@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
}

void MainWindow::on_closeButton_clicked()
{
	//close roslaunch
	system("pkill stage");
}

