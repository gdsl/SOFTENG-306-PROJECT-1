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
	string id;

signals:    
    void requestNewLabel(const QString &, int);
};
