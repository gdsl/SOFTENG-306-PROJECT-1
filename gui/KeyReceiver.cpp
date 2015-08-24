#include "KeyReceiver.h"
#include <QKeyEvent>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <QDebug>
#include <sstream>

KeyReceiver::KeyReceiver() {
	//timer = new QTimer(this);
	//QObject::connect(timer, SIGNAL(timeout()), this, SLOT(fireKeyData()));
	//timer->start(3000); //time specified in ms
}

bool KeyReceiver::eventFilter(QObject* obj, QEvent* event)
{
	if (event->type()==QEvent::KeyPress) {
		QKeyEvent* key = static_cast<QKeyEvent*>(event);
		if ( (key->key()==Qt::Key_Left) ) {
			lastKeyPressed = 1;
			//qDebug("left KEY");
		} else if ( (key->key()==Qt::Key_Right) ) {
			lastKeyPressed = 2;
			// qDebug("Right KEY");
		} else if ( (key->key()==Qt::Key_Up) ) {
			lastKeyPressed = 3;
			// qDebug("Up KEY");
		} else if ( (key->key()==Qt::Key_Down) ) {
			lastKeyPressed = 4;
			// qDebug("Down KEY");
		}
	} else {
		return QObject::eventFilter(obj, event);
	}
	return false;
}

