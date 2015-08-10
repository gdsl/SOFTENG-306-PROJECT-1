#include <QObject>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

class GUIUpdater : public QObject {
    Q_OBJECT

public:
    explicit GUIUpdater(QObject *parent = 0) : QObject(parent){}    
    void newLabel(const QString &image);
	void exec(string cmd);

signals:    
    void requestNewLabel(const QString &);
};
