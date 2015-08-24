#include <QObject>
#include <QTimer>

<<<<<<< HEAD
class KeyReceiver : public QObject
{
	Q_OBJECT

=======
class KeyReceiver : public QObject {
    Q_OBJECT
    
>>>>>>> 002c2a1f3dc841f56b998f90719b2f96dade691f
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
