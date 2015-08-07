#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

	public:
		explicit MainWindow(QWidget *parent = 0);

	private slots:
		void handleButton();

	private:
		QPushButton *m_button;
		bool isRunningSimulator;

};

#endif // MAINWINDOW_H
