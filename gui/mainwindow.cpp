#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "worker.h"
#include <QThread>
#include <QListWidget>

using namespace std;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    /*MainWindow::uiList = {ui->robotList1, ui->robotList2, ui->obstacleList1, ui->obstacleList2}; */
   /* uiList[0] = ui->robotList1;
    uiList[1] = ui->robotList2;
    uiList[2] = ui->obstacleList1;
    uiList[3] = ui->obstacleList2; */
        
    //uiList.insert(0, ui->robotList1);
   // uiList.insert(1, ui->robotList2);
        
    ui->setupUi(this);
	ui->robotList1->item(0)->setText("Type: Picker");
	ui->robotList2->item(0)->setText("Type: Carrier");
    
    uiList[0] = ui->robotList1;
    uiList[1] = ui->robotList2;
    uiList[2] = ui->obstacleList1;
    uiList[3] = ui->obstacleList2;
        
  /* for(int i = 0; i < 2; i++) {
        QThread *thread = new QThread(this);
        Worker *worker = new Worker();
        worker->moveToThread(thread);
        worker->setId(i);
       
        connect (thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
        connect(worker, SIGNAL(requestNewLabel(QString, int)), this, SLOT(onUpdateGUI(QString, int))); //custom signal which calls the slot for onUpdateGUI
        connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
	
        thread->start();
    } */
        
    QThread *thread = new QThread(this);
    Worker *worker = new Worker();
    worker->moveToThread(thread);
    worker->setId("0");
       
    connect (thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
    connect(worker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int))); //custom signal which calls the slot for onUpdateGUI
    connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
	
    thread->start();
    
	/*//create a new thread
	QThread *thread = new QThread(this);
	//create a new worker (a bit like swingworker)
    Worker *worker = new Worker();
    worker->moveToThread(thread);
	worker->setId("robot_0"); */
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::onUpdateGUI( QString id, QString str, int i )
{
	//update the gui for robots
    bool ok;
    //qDebug(id.toUtf8().constData());
    uiList[id.toInt()]->item(i)->setText(str);
    //ui->robotList1->item(i)->setText(str);
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

