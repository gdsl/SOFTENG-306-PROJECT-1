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
#include <QDebug>
#include <vector>
#include <sstream>
#include <math.h>

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
}

void MainWindow::setKey(KeyReceiver *k) {
	key = k;
}

int MainWindow::getLastKeyPressed() {
	return key->lastKeyPressed;
}

void MainWindow::startReadingTopics() {
	bool ok;
	int totalNodes = model.getTotalNodes();

	for (int i = model.beacons+model.weed; i < totalNodes ; i++) {
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
	system("pkill Tractor");
	system("pkill roslaunch");
	system("pkill stage");
	system("pkill rostopic");
	system("pkill roscore");
	delete ui;
}

void MainWindow::onUpdateGUI( QString id, QString str, int i )
{
	//update the gui for robots
	int idNum = id.toInt()-model.beacons-model.weed;
    int numRobots = model.carrierRobots + model.pickerRobots;
    int numRobotsPlusPeople = model.carrierRobots+model.pickerRobots+model.workers+model.gardeners+model.neighbours+model.blindPerson;
    
	if (idNum < numRobots) {
	    QListWidget *qlw = ((QListWidget*)ui->robotScroll->widget()->layout()->itemAt(idNum)->widget());
    	qlw->item(i)->setText(truncate(str));
    } else if (idNum < numRobotsPlusPeople){
    	QListWidget *qlw = ((QListWidget*)ui->peopleScroll->widget()->layout()->itemAt(idNum-numRobots)->widget());
    	qlw->item(i)->setText(truncate(str));
    } else {
    	QListWidget *qlw = ((QListWidget*)ui->animalScroll->widget()->layout()->itemAt(idNum-numRobotsPlusPeople)->widget());
    	qlw->item(i)->setText(truncate(str));
    }
}

QString MainWindow::truncate(QString str) {
	bool ok;
	QStringList pieces = str.split( " " );
	QString numString = pieces.value( pieces.length() -1 );
	double d = numString.toDouble(&ok);
	if (ok) {
		double scale = 0.01;  // i.e. round to nearest one-hundreth
		d = floor(d / scale + 0.5) * scale;
		return pieces.value(0) + " " + QString::number(d);
	} else {
		return str;
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

void MainWindow::on_testDriveButton_clicked()
{
	if (!startedTestDrive) {
		//start the sender to tractor in new thread
		Worker *worker = new Worker();
		worker->setMainWindow(this);
		QThread *thread = new QThread(this);
		worker->moveToThread(thread);
		connect(thread, SIGNAL(started()), worker, SLOT(sendToTractor()));
		thread->start();
		startedTestDrive = true;
		ui->robotScroll->setFocus();
	}
}

void MainWindow::on_closeButton_clicked() {
	system("pkill Tractor");
	system("pkill roslaunch");
	startedTestDrive = false;
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
	model.neighbours = ui->neighborSpinner->value();
	model.gardeners = ui->gardenerSpinner->value();
	//model.tractors = ui->tractorSpinner->value();
	model.beacons = model.rowCount*2;

	uiListPeoples.clear();
	uiListRobots.clear();
	uiListAnimals.clear();

	for (int i = 0; i < model.pickerRobots; i++) {
		uiListRobots.push_back(createNewItem("Picker"));
	}
	for (int i = 0; i < model.carrierRobots; i++) {
		uiListRobots.push_back(createNewItem("Carrier"));
	}
	for (int i = 0; i < model.workers; i++) {
		uiListPeoples.push_back(createNewItem("Human_Worker"));
	}
	for (int i = 0; i < model.gardeners; i++) {
		uiListPeoples.push_back(createNewItem("Gardener"));
	}
	for (int i = 0; i < model.blindPerson; i++) {
        uiListPeoples.push_back(createNewItem("Blind_Person"));
    }
	for (int i = 0; i < model.neighbours; i++) {
		uiListPeoples.push_back(createNewItem("Neighbour"));
	}
	for (int i = 0; i < model.dogs; i++) {
		uiListAnimals.push_back(createNewItem("Animal_Dog"));
	}
	for (int i = 0; i < model.cats; i++) {
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
		QListWidget *animalQL = ((QListWidget*)ui->animalScroll->widget()->layout()->itemAt(i)->widget());
		QString backgroundColour = "QListWidget {background: " + QString::fromStdString(colourArray[i]) + ";}";
		animalQL->setStyleSheet(backgroundColour);
	}
	for (int i = 0; i < uiListPeoples.size(); i++) {
		ui->peopleScroll->widget()->layout()->addWidget(uiListPeoples[i]);
		QListWidget *peopleQL = ((QListWidget*)ui->peopleScroll->widget()->layout()->itemAt(i)->widget());
		QString backgroundColour = "QListWidget {background: " + QString::fromStdString(colourArray[i]) + ";}";
		peopleQL->setStyleSheet(backgroundColour);
	}
	Generator generator(model);

	generator.loadWorld();
	generator.loadTallWeeds();
	generator.loadOrchard();
	generator.loadPickerRobots();
	generator.loadCarrierRobots();
	generator.loadPeople();
	generator.loadAnimals();
	generator.loadTractor();
	generator.loadBackdrop();
	generator.write();
	generator.writeLaunchFile();

}

int MainWindow::getTotalNodesFromModel() {
	return model.getTotalNodes();
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

