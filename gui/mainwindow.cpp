#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "worker.h"
#include <QThread>
#include <QListWidget>
#include <sstream>

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
        
}

void MainWindow::startReadingTopics() {
	QThread *thread = new QThread(this);
    QThread *thread2 = new QThread(this);
        
    Worker *worker = new Worker();
    Worker *secondWorker = new Worker();
    
    worker->moveToThread(thread);
    secondWorker->moveToThread(thread2);
        
    worker->setId("0");
    secondWorker->setId("1");
        
    connect(thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
    connect(thread2, SIGNAL(started()), secondWorker, SLOT(executeScript()));
        
    connect(worker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int))); //custom signal which calls the slot for onUpdateGUI
    connect(secondWorker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int)));
    
    connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
    connect(thread2, SIGNAL(destroyed()), secondWorker, SLOT(deleteLater()));
	
    thread->start();
    thread2->start();
        
/*    for(int i = 0; i < 2; i++) {
        
        std::string str;
        std:stringstream out;
        out << i;
        str = out.str();
        
        Worker theWorker = workerArray[i];
        
        theWorker->setId(str);
       
        connect (thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
        connect(worker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int))); //custom signal which calls the slot for onUpdateGUI
        connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
	
        thread->start(); 
    } */
    
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
    uiList[id.toInt()]->item(i)->setText(str);
    //ui->robotList1->item(i)->setText(str);
}

void MainWindow::on_launchButton_clicked()
{
	//launch roslaunch
	system("roslaunch se306project orchard.launch &");

	//emit MainWindow::requestProcess();
	qDebug("THREAD STARTED@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	startReadingTopics();
}

void MainWindow::on_closeButton_clicked()
{
	//close roslaunch
	system("pkill stage");
}

