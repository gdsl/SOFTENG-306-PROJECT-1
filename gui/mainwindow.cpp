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
#include <sstream>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
	system("pkill roslaunch");
    ui->setupUi(this);

    // Ask vector capacity to reserve atleast n elements
    uiListRobots.reserve(50);  
    uiListAnimals.reserve(50); 
    uiListPeoples.reserve(50);
    
    ui->robotScroll->widget()->layout()->setAlignment(Qt::AlignLeft);
    ui->peopleScroll->widget()->layout()->setAlignment(Qt::AlignLeft);
    ui->animalScroll->widget()->layout()->setAlignment(Qt::AlignLeft);
    
    key = new KeyReceiver();
    ui->animalScroll->installEventFilter(key);
    
}

void MainWindow::startReadingTopics() {
    bool ok;
    int totalDynamicStuff = numPickers + numCarriers + numWorkers + numDogs + numCats + numTractors;
    
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
    //close roslaunch and close all rostopics
	system("pkill roslaunch");
	system("pkill stage");
	system("pkill rostopic");
	system("pkill roscore");
    delete ui;
}


void MainWindow::onUpdateGUI( QString id, QString str, int i )
{
	//update the gui for robots
	int idNum = id.toInt()-numBeacons-numWeeds;

	if (idNum < numCarriers+numPickers) {
	    QListWidget *qlw = ((QListWidget*)ui->robotScroll->widget()->layout()->itemAt(idNum)->widget());
    	qlw->item(i)->setText(str);
    } else if (idNum < numCarriers+numPickers+numWorkers){
    	QListWidget *qlw = ((QListWidget*)ui->peopleScroll->widget()->layout()->itemAt(idNum-(numCarriers + numPickers))->widget());
    	qlw->item(i)->setText(str);
    } else {
    	QListWidget *qlw = ((QListWidget*)ui->animalScroll->widget()->layout()->itemAt(idNum-(numCarriers + numPickers+numWorkers))->widget());
    	qlw->item(i)->setText(str);
    }

}

void MainWindow::on_launchButton_clicked()
{
    MainWindow::generate();
	//launch roslaunch
	system("roslaunch se306project test.launch &");
    usleep(1000000); //1 second
	//emit MainWindow::requestProcess();
	startReadingTopics();

}

void MainWindow::on_displayStatusButton_clicked()
{
	startReadingTopics();
    //MainWindow::generate();
}

void MainWindow::on_closeButton_clicked() {
	system("pkill roslaunch");
	system("pkill stage");
	system("pkill rostopic");
	system("pkill roscore");
}


void MainWindow::generate() {
    //writeXml();

    // initialise variables
    model.pickerRobots = ui->pickerSpinner->value();
    model.carrierRobots = ui->carrierSpinner->value();
    model.workers = ui->workerSpinner->value();
    model.dogs = ui->dogSpinner->value();
    model.cats = ui->catSpinner->value();
    model.rowWidth = ui->rowWidthSpinner->value();
    model.poleTrunkSpacing = ui->spacingSpinner->value();
    model.rowCount = ui->rowNumberSpinner->value();
    model.blindPerson = ui->blindPersonSpinner->value();
    model.neighbors = ui->neighborSpinner->value();
    model.gardeners = ui->gardenerSpinner->value();
    //model.tractors = ui->tractorSpinner->value();
    model.beacons = model.rowCount*2;
    
    uiListPeoples.clear();
    uiListRobots.clear();
    uiListAnimals.clear();

    for (int i = 0; i < numPickers; i++) {
        uiListRobots.push_back(createNewItem("Picker"));
    }
    for (int i = 0; i < numCarriers; i++) {
        uiListRobots.push_back(createNewItem("Carrier"));
    }
    for (int i = 0; i < numWorkers; i++) {
        uiListPeoples.push_back(createNewItem("Human_Worker"));
    }
    for (int i = 0; i < numDogs; i++) {
        uiListAnimals.push_back(createNewItem("Animal_Dog"));
    }
    for (int i = 0; i < numCats; i++) {
        uiListAnimals.push_back(createNewItem("Animal_Cat")); 
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
    while (( item = ui->peopleScroll->widget()->layout()->takeAt(0)) != 0 ){
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
    for (int i = 0; i < uiListPeoples.size(); i++) {
    	ui->peopleScroll->widget()->layout()->addWidget(uiListPeoples[i]);
    }
    Generator generator("world/test.world", model);
    
	generator.loadWorld();
	generator.loadTallWeeds();
	generator.loadOrchard();
	generator.loadPickerRobots();
	generator.loadCarrierRobots();
	generator.loadPeople();
	generator.loadAnimals();
	
	generator.write();
    generator.writeLaunchFile();
	generator.loadTractor();
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
                xml.AddElem( "row_count", ui->rowNumberSpinner->value());
                xml.AddElem( "row_length", ui->rowLengthSpinner->value());
                xml.AddElem( "row_width", ui->rowWidthSpinner->value() );
                xml.AddElem( "trunk_pole_spacing", ui->poleTrunkSpacingSpinner->value() );
            xml.OutOfElem();
            xml.AddElem("robots");
            xml.IntoElem();
                xml.AddElem( "picker_number", ui->pickerSpinner->value() );
                xml.AddElem( "carrier_number", ui->carrierSpinner->value() );
            xml.OutOfElem();
            xml.AddElem("people");
            xml.IntoElem();
                xml.AddElem( "worker_number", ui->workerSpinner->value() );
            xml.OutOfElem();
            xml.AddElem("animals");
            xml.IntoElem();
                xml.AddElem( "dog_number", ui->dogSpinner->value() );
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

    list->setMinimumWidth(200);
    return list;
}

