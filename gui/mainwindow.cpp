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
	ui->robotList1->item(0)->setText("Type: Picker");
	ui->robotList2->item(0)->setText("Type: Carrier");

	//create a new thread
	QThread *thread = new QThread(this);
	//create a new worker (a bit like swingworker)
    Worker *worker = new Worker();
    worker->moveToThread(thread);
	worker->setId("robot_0");
 	connect (thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
    connect(worker, SIGNAL(requestNewLabel(QString, int)), this, SLOT(onUpdateGUI(QString, int))); //custom signal which calls the slot for onUpdateGUI
    connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
	
	thread->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::onUpdateGUI( QString str, int i )
{
	//update the gui for robots
    ui->robotList1->item(i)->setText(str);
}

void MainWindow::on_launchButton_clicked()
{
	//launch roslaunch
	system("roslaunch se306project orchard.launch &");

	emit MainWindow::requestProcess();
	qDebug("THREAD STARTED@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
}

void MainWindow::on_closeButton_clicked()
{
	//close roslaunch
	system("pkill stage");
}

