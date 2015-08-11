#include <QObject>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

class Worker : public QObject {
    Q_OBJECT

public:
    explicit Worker(QObject *parent = 0) : QObject(parent){}    
	void exec(string cmd);
	void setId(string id_string);

public slots:
    void executeScript();

private:
	QString id;

signals:    
    void requestNewLabel(QString, const QString &, int);
};
