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
    	ui->animalList1->item(0)->setText("Type: Dog");
	ui->humanList1->item(0)->setText("Type: Human");
    uiList[0] = ui->robotList1;
    uiList[1] = ui->robotList2;
    uiList[2] = ui->animalList1;
    uiList[3] = ui->humanList1;        
}

void MainWindow::startReadingTopics() {
	for (int i = 0; i < 4; i++) {
		QThread *thread = new QThread(this);
		//QThread *thread2 = new QThread(this);
		    
		Worker *worker = new Worker();
		//Worker *secondWorker = new Worker();
		
		worker->moveToThread(thread);
		//secondWorker->moveToThread(thread2);
	    stringstream out;
		out << i;
		worker->setId( out.str());
		//secondWorker->setId("1");
		    
		connect(thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
		//connect(thread2, SIGNAL(started()), secondWorker, SLOT(executeScript()));
		    
		connect(worker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int))); //custom signal which calls the slot for onUpdateGUI
		//connect(secondWorker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int)));
		
		connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
		//connect(thread2, SIGNAL(destroyed()), secondWorker, SLOT(deleteLater()));
	
		thread->start();
		//thread2->start();
    }
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
	startReadingTopics();
}

void MainWindow::on_closeButton_clicked()
{
	//close roslaunch
	system("pkill stage");
}

