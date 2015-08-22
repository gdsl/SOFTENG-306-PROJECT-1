#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <QtGui/QMainWindow>
#include <QListWidget>


using namespace std;

namespace Ui {
class MainWindow;
}

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
    ~MainWindow();

private slots:
	void on_launchButton_clicked();
	void on_closeButton_clicked();
    void on_displayStatusButton_clicked();

private:
    Ui::MainWindow *ui;
    QListWidget *uiList[4] = {NULL, NULL, NULL, NULL};
    vector<QListWidget*> uiListRobots;
    vector<QListWidget*> uiListAnimals;
    vector<string> launchFileEntityList;
  
    int numPickers;
    int numCarriers;
    int numRows = 7;
    int numBeacons = 14;
    int numWeeds = 10;
    int numDogs;
    int numCats;
    int numWorkers;
    
//	void updateGUI(const QString buffer);
//	void *print_message_function( void *ptr );
//	void exec(string cmd);
};

#endif // MAINWINDOW_H
