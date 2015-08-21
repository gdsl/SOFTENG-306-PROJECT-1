#include "KeyReceiver.h"
#include <QKeyEvent>

bool KeyReceiver::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type()==QEvent::KeyPress) {
        QKeyEvent* key = static_cast<QKeyEvent*>(event);
        if ( (key->key()==Qt::Key_Left) ) {
            qDebug("LEFT KEY");
        } else if ( (key->key()==Qt::Key_Right) ) {
            qDebug("Right KEY");
        } else if ( (key->key()==Qt::Key_Up) ) {
            qDebug("Up KEY");
        } else if ( (key->key()==Qt::Key_Down) ) {
            qDebug("Down KEY");
        }else {
            return QObject::eventFilter(obj, event);
        }
        return true;
    } else {
        return QObject::eventFilter(obj, event);
    }
    return false;
}
