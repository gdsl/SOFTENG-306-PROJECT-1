#include "KeyReceiver.h"
#include <QKeyEvent>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <signal.h>

KeyReceiver::KeyReceiver() {
    timer = new QTimer(this);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(fireKeyData()));
    timer->start(3000); //time specified in ms
}

void KeyReceiver::fireKeyData(){
    if (lastKeyPressed == 0) {
            qDebug("none KEY");
        system("rostopic pub tractor std_msgs/String \"none\" --once &");
        timer->stop();
    } else if (lastKeyPressed == 1) {
            qDebug("left KEY");
        system("rostopic pub tractor std_msgs/String \"left\" --once &");
    } else if (lastKeyPressed == 2) {
            qDebug("right KEY");
        system("rostopic pub tractor std_msgs/String \"right\" --once &");
    } else if (lastKeyPressed == 3) {
        qDebug("up KEY");
        system("rostopic pub tractor std_msgs/String \"up\" --once &");
        qDebug("down KEY");
    } else if (lastKeyPressed == 4) {
        system("rostopic pub tractor std_msgs/String \"down\" --once &");
    }
    lastKeyPressed = 0;
}

bool KeyReceiver::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type()==QEvent::KeyPress) {
        timer->start(3000); //time specified in ms
        QKeyEvent* key = static_cast<QKeyEvent*>(event);
        if ( (key->key()==Qt::Key_Left) ) {
            lastKeyPressed = 1;
        } else if ( (key->key()==Qt::Key_Right) ) {
            lastKeyPressed = 2;
            qDebug("Right KEY");
        } else if ( (key->key()==Qt::Key_Up) ) {
            lastKeyPressed = 3;
            qDebug("Up KEY");
        } else if ( (key->key()==Qt::Key_Down) ) {
            lastKeyPressed = 4;
            qDebug("Down KEY");
        }
    } else {
        return QObject::eventFilter(obj, event);
    }
    return false;
}



