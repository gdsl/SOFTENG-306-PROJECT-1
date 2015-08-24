#include <QObject>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mainwindow.h"

using namespace std;

class Worker : public QObject {
    Q_OBJECT

public:
    explicit Worker(QObject *parent = 0) : QObject(parent){}    
	void exec(string cmd);
	void setId(string id_string);
	void setMainWindow(MainWindow *m);
    
public slots:
    void executeScript();
    void sendToTractor();

private:
	QString id;
    string stringId;
    MainWindow *mw;

signals:    
    void requestNewLabel(QString, const QString &, int);
};
