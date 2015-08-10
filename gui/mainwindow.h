#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <QtGui/QMainWindow>


using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
	void requestProcess();

public slots:
	void onUpdateGUI( QString str );

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
//	static void updateGUI(const QString buffer);

private slots:
	void on_launchButton_clicked();

	void on_closeButton_clicked();

private:
    Ui::MainWindow *ui;
//	void updateGUI(const QString buffer);
//	void *print_message_function( void *ptr );
//	void exec(string cmd);
};

#endif // MAINWINDOW_H
