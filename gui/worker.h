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

public slots:
    void newLabel();

signals:    
    void requestNewLabel(const QString &);
};
