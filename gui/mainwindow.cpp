#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "worker.h"
#include <QThread>
#include <QListWidget>
#include <sstream>
#include "../include/Markup.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    uiListRobots.reserve(50);  
    uiListAnimals.reserve(50); 
    
    ui->robotScroll->widget()->layout()->setAlignment(Qt::AlignLeft);
    ui->animalScroll->widget()->layout()->setAlignment(Qt::AlignLeft);
}

void MainWindow::startReadingTopics() {
	for (int i = 0; i < 4; i++) {
		QThread *thread = new QThread(this);    
		Worker *worker = new Worker();
		
		worker->moveToThread(thread);
	    stringstream out;
		out << i;
		worker->setId( out.str());
		    
		connect(thread, SIGNAL(started()), worker, SLOT(executeScript())); //started() signal is by default called by thread->start
		connect(worker, SIGNAL(requestNewLabel(QString, QString, int)), this, SLOT(onUpdateGUI(QString, QString, int))); //custom signal which calls the slot for onUpdateGUI
		connect(thread, SIGNAL(destroyed()), worker, SLOT(deleteLater()));
		thread->start();
    }
}


MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::onUpdateGUI( QString id, QString str, int i )
{
	//update the gui for robots
    //Select UI element based on ID. Then set the text for line i with message str.
    uiList[id.toInt()]->item(i)->setText(str);
	int idNum = id.toInt();


	//hardcoded for now
	if (idNum < 2) {
	    QListWidget *qlw = ((QListWidget*)ui->robotScroll->widget()->layout()->itemAt(idNum)->widget());
    	qlw->item(i)->setText(str);
    } else {
	    QListWidget *qlw = ((QListWidget*)ui->animalScroll->widget()->layout()->itemAt(idNum-2)->widget());
    	qlw->item(i)->setText(str);
    }
}

void MainWindow::on_launchButton_clicked()
{
    MainWindow::generate();
    
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

void MainWindow::on_generateButton_clicked()
{
    MainWindow::generate();
}

    
void MainWindow::generate() {
CMarkup xml;
    bool ok;
    xml.AddElem( "picker_number", ui->pickerRobotsField->text().toInt(&ok, 10) );
    xml.AddElem( "carrier_number", ui->carrierRobotsField->text().toInt(&ok, 10) );
    xml.AddElem( "worker_number", ui->workersField->text().toInt(&ok, 10) );
    xml.AddElem( "dog_number", ui->dogsField->text().toInt(&ok, 10) );
    xml.AddElem( "resolution", 2 );
    xml.AddElem( "row_width", ui->rowWidthField->text().toInt(&ok, 10) );
    xml.AddElem( "trunk_pole_spacing", ui->spacingField->text().toInt(&ok, 10) );
    xml.Save( "world/orchard.xml" );
    
    int numPicker = ui->pickerRobotsField->text().toInt(&ok, 10);
    int numCarrier = ui->carrierRobotsField->text().toInt(&ok, 10);
    int numWorkers = ui->workersField->text().toInt(&ok, 10);
    int numDogs = ui->dogsField->text().toInt(&ok, 10);
    
    int rowWidth = ui->rowWidthField->text().toInt(&ok, 10);
    int spacing = ui->spacingField->text().toInt(&ok, 10);

    uiListRobots.clear();
    uiListAnimals.clear();  
    for (int i = 0; i < numPicker; i++) {
        uiListRobots.push_back(createNewItem("Picker"));   
    }
    for (int i = 0; i < numCarrier; i++) {
        uiListRobots.push_back(createNewItem("Carrier"));   
    }
    for (int i = 0; i < numWorkers; i++) {
        uiListAnimals.push_back(createNewItem("Human_Worker"));   
    }
    for (int i = 0; i < numDogs; i++) {
        uiListAnimals.push_back(createNewItem("Animal_Dog"));   
    }
    //clear the layout
    QLayoutItem *item;
    while (( item = ui->robotScroll->widget()->layout()->takeAt(0)) != 0 ){
        delete item->widget();
        delete item;
    }
    while (( item = ui->animalScroll->widget()->layout()->takeAt(0)) != 0 ){
        delete item->widget();
        delete item;
    }
    //add all widgets back
    for (int i = 0; i < uiListRobots.size(); i++) {
        ui->robotScroll->widget()->layout()->addWidget(uiListRobots[i]);
    }
    for (int i = 0; i < uiListAnimals.size(); i++) {
        ui->animalScroll->widget()->layout()->addWidget(uiListAnimals[i]);
    }

}

QListWidget* MainWindow::createNewItem(string type) {
    QListWidget *list = new QListWidget;
    QListWidgetItem *item = new QListWidgetItem;
    string typeLabel("Type: ");
    item->setText((typeLabel + type).c_str());
    list->addItem(item);
    
    QListWidgetItem *item2 = new QListWidgetItem;
    list->addItem(item2);
    QListWidgetItem *item3 = new QListWidgetItem;
    list->addItem(item3);
    QListWidgetItem *item4 = new QListWidgetItem;
    list->addItem(item4);
    QListWidgetItem *item5 = new QListWidgetItem;
    list->addItem(item5);
    QListWidgetItem *item6 = new QListWidgetItem;
    list->addItem(item6);
    
    list->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    list->setFixedSize(180,150);
    return list;
}

