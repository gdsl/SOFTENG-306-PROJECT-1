#include <QObject>
#include <QTimer>

class KeyReceiver : public QObject
{
    Q_OBJECT
    
public:
    KeyReceiver();
    int lastKeyPressed = 0; //nothing is 0, left is 1, right is 2, up is 3, down is 4
    
public slots:
    //void fireKeyData();
    //void sendToTractor();
    
protected:
    bool eventFilter(QObject* obj, QEvent* event);
    
    
private:
    QTimer *timer;
};
