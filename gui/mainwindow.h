#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <QtGui/QMainWindow>
#include <QListWidget>
#include "KeyReceiver.h"
#include "Generator.h"

using namespace std;

namespace Ui {
	class MainWindow;
}

<<<<<<< HEAD
class MainWindow : public QMainWindow
{
	Q_OBJECT

	signals:
	void requestProcess();

	public slots:
	void onUpdateGUI( QString id, QString str, int i );

	public:
	explicit MainWindow(QWidget *parent = 0);
	void startReadingTopics();
	QListWidget* createNewItem(string type);
	void generate();
	void writeXml();
	void writeLaunchFile();
	void setKey(KeyReceiver *k);
	int getLastKeyPressed();
	int getTotalNodesFromModel();
	QString truncate(QString str);
	~MainWindow();

	// GeneratorModel
	GeneratorModel model;

	private slots:
	void on_launchButton_clicked();
	void on_displayStatusButton_clicked();
	void on_closeButton_clicked();
	void on_testDriveButton_clicked();

	private:
	Ui::MainWindow *ui;
	QListWidget *uiList[4] = {NULL, NULL, NULL, NULL};
	vector<QListWidget*> uiListRobots;
	vector<QListWidget*> uiListAnimals;
	vector<QListWidget*> uiListPeoples;
	vector<string> launchFileEntityList;

	KeyReceiver* key;
	bool startedTestDrive = false;
	//	void updateGUI(const QString buffer);
	//	void *print_message_function( void *ptr );
	//	void exec(string cmd);
=======
class MainWindow : public QMainWindow {
	Q_OBJECT

	signals:
		void requestProcess();

	public slots:
		void onUpdateGUI( QString id, QString str, int i );

	public:
	    explicit MainWindow(QWidget *parent = 0);
		void startReadingTopics();
		QListWidget* createNewItem(string type);
		void generate();
		void writeXml();
		void writeLaunchFile();
		void setKey(KeyReceiver *k);
		int getLastKeyPressed();
		int getTotalNodesFromModel();
	    QString truncate(QString str);
	    ~MainWindow();

	    // Generator Model
	    GeneratorModel model;

	private slots:
		void on_launchButton_clicked();
	    void on_displayStatusButton_clicked();
	    void on_closeButton_clicked();
	    void on_testDriveButton_clicked();

	private:
	    Ui::MainWindow *ui;
	    QListWidget *uiList[4] = {NULL, NULL, NULL, NULL};
	    vector<QListWidget*> uiListRobots;
	    vector<QListWidget*> uiListAnimals;
	    vector<QListWidget*> uiListPeoples;
	    vector<string> launchFileEntityList;
	    KeyReceiver* key;
	    bool startedTestDrive = false;
>>>>>>> 002c2a1f3dc841f56b998f90719b2f96dade691f
};

#endif // MAINWINDOW_H
