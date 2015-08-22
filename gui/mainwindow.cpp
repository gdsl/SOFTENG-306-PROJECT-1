#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "worker.h"
#include <QThread>
#include <QListWidget>
#include "unistd.h"
#include <QLayout>
#include "Generator.h"
#include <QDebug>
#include <vector>
#include "KeyReceiver.h"
#include <sstream>

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
    
    KeyReceiver* key = new KeyReceiver();
    ui->animalScroll->installEventFilter(key);
}

void MainWindow::startReadingTopics() {
    bool ok;
    int totalDynamicStuff = numPickers + numCarriers + numWorkers + numDogs + numCats;
    
	for (int i = numBeacons+numWeeds; i < numBeacons + numWeeds + totalDynamicStuff ; i++) {
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
	int idNum = id.toInt()-numBeacons-numWeeds;
	
	if (idNum < numCarriers+numPickers) {
	    QListWidget *qlw = ((QListWidget*)ui->robotScroll->widget()->layout()->itemAt(idNum)->widget());
    	qlw->item(i)->setText(str);
    } else {
	    QListWidget *qlw = ((QListWidget*)ui->animalScroll->widget()->layout()->itemAt(idNum-(numCarriers + numPickers))->widget());
    	qlw->item(i)->setText(str);
    }

}

void MainWindow::on_launchButton_clicked()
{
    MainWindow::generate();
	
	//launch roslaunch
	system("roslaunch se306project test.launch &");
    usleep(1000000); //1 second
    //qDebug("started reading!!!!!!!!!!!!!!!!!!!!!!!!!!1");
	//emit MainWindow::requestProcess();
	startReadingTopics();
}

void MainWindow::on_closeButton_clicked()
{  
	//close roslaunch and close all rostopics
	system("pkill stage");
	system("pkill rostopic");
	system("pkill roslaunch");	
}

void MainWindow::on_displayStatusButton_clicked()
{
	startReadingTopics();
    //MainWindow::generate();
}


void MainWindow::generate() {
    //writeXml();
    bool ok;
    numPickers = ui->pickerRobotsField->text().toInt(&ok, 10);
    numCarriers = ui->carrierRobotsField->text().toInt(&ok, 10);
    numWorkers = ui->workersField->text().toInt(&ok, 10);
    numDogs = ui->dogsField->text().toInt(&ok, 10);
    numCats = ui->catsField->text().toInt(&ok, 10);
    float rowWidth = ui->rowWidthField->text().toFloat(&ok);
    float spacing = ui->spacingField->text().toFloat(&ok);
    
    uiListRobots.clear();
    uiListAnimals.clear();
    launchFileEntityList.clear();
    
    for (int i = 0; i < numWeeds; i++) {
        launchFileEntityList.push_back("TallWeed");    
    }
    for (int i = 0; i < numRows; i++) {
        launchFileEntityList.push_back("Beacon");
        launchFileEntityList.push_back("Beacon");
    }
    for (int i = 0; i < numPickers; i++) {
        uiListRobots.push_back(createNewItem("Picker"));
        launchFileEntityList.push_back("PickerRobot");
    }
    for (int i = 0; i < numCarriers; i++) {
        uiListRobots.push_back(createNewItem("Carrier"));
        launchFileEntityList.push_back("CarrierRobot");
    }
    for (int i = 0; i < numWorkers; i++) {
        uiListAnimals.push_back(createNewItem("Human_Worker"));
        launchFileEntityList.push_back("AlphaPerson"); 
    }
    for (int i = 0; i < numDogs; i++) {
        uiListAnimals.push_back(createNewItem("Animal_Dog")); 
        launchFileEntityList.push_back("AlphaDog");  
    }
    for (int i = 0; i < numCats; i++) {
        uiListAnimals.push_back(createNewItem("Animal_Cat")); 
        launchFileEntityList.push_back("Cat");  
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
    string colourArray[14] = { "PeachPuff", "NavajoWhite", "LemonChiffon", "AliceBlue", "Lavender", "thistle", "LightSalmon", "PaleTurquoise", "PaleGreen", "beige", "plum", "LightGrey", "LightSkyBlue", "SpringGreen" };
    for (int i = 0; i < uiListRobots.size(); i++) {
        ui->robotScroll->widget()->layout()->addWidget(uiListRobots[i]);
        QListWidget *robotQL = ((QListWidget*)ui->robotScroll->widget()->layout()->itemAt(i)->widget());
        QString backgroundColour = "QListWidget {background: " + QString::fromStdString(colourArray[i]) + ";}";
        robotQL->setStyleSheet(backgroundColour);
    }
    for (int i = 0; i < uiListAnimals.size(); i++) {
        ui->animalScroll->widget()->layout()->addWidget(uiListAnimals[i]);
    }
    
    Generator generator("world/test.world", numPickers, numCarriers, numDogs, numCats, numWorkers, rowWidth, spacing);
	generator.loadWorld();
	generator.loadTallWeeds();
	generator.loadOrchard();
	generator.loadPickerRobots();
	generator.loadCarrierRobots();
	generator.loadPeople();
	generator.loadAnimals();
	generator.write();
	generator.writeLaunchFile();

}
/*
void MainWindow::writeXml() {
    CMarkup xml;
    bool ok;
    xml.ResetPos();
    xml.InsertNode( xml.MNT_PROCESSING_INSTRUCTION, "xml" );
    xml.SetAttrib( "version", "1.1" );
    xml.SetAttrib( "encoding", "UTF-8" );
    xml.AddElem("world");
    xml.IntoElem();
        xml.AddElem( "resolution", "0.02" );
        xml.AddElem( "interval_sim", 100 );
        xml.AddElem( "interval_real", 100 );
        xml.AddElem( "paused", 0 );
        xml.AddElem("models");
        xml.IntoElem();
            xml.AddElem("orchard");
            xml.IntoElem();
                xml.AddElem( "row_count", 7);
                xml.AddElem( "row_length", 70);
                xml.AddElem( "row_width", ui->rowWidthField->text().toStdString() );
                xml.AddElem( "trunk_pole_spacing", ui->spacingField->text().toStdString() );
            xml.OutOfElem();
            xml.AddElem("robots");
            xml.IntoElem();
                xml.AddElem( "picker_number", ui->pickerRobotsField->text().toInt(&ok, 10) );
                xml.AddElem( "carrier_number", ui->carrierRobotsField->text().toInt(&ok, 10) );
            xml.OutOfElem();
            xml.AddElem("people");   
            xml.IntoElem();             
                xml.AddElem( "worker_number", ui->workersField->text().toInt(&ok, 10) );
            xml.OutOfElem();
            xml.AddElem("animals"); 
            xml.IntoElem(); 
                xml.AddElem( "dog_number", ui->dogsField->text().toInt(&ok, 10) );
            xml.OutOfElem();
        xml.OutOfElem();
    xml.OutOfElem();
    xml.Save( "world/generatedOrchard.xml" );
}
*/



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

