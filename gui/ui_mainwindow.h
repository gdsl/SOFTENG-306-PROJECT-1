/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *robotLabel;
    QTextEdit *robotPanel1;
    QTextEdit *robotPanel2;
    QLabel *obstacleLabel;
    QTextEdit *obstaclePanel1;
    QTextEdit *obstaclePanel2;
    QTextEdit *obstaclePanel3;
    QTextEdit *obstaclePanel4;
    QTextEdit *obstaclePanel5;
    QLabel *settingsLabel;
    QLabel *numRobotsLabel;
    QLabel *rowLengthLabel;
    QLabel *spacingLabel;
    QLineEdit *numRobotsField;
    QLineEdit *rowLengthField;
    QLineEdit *spacingField;
    QPushButton *closeButton;
    QPushButton *launchButton;
    QMenuBar *menuBar;
    QMenu *menuOrchard_Simulator;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(469, 362);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        robotLabel = new QLabel(centralWidget);
        robotLabel->setObjectName(QString::fromUtf8("robotLabel"));
        robotLabel->setGeometry(QRect(20, 0, 67, 17));
        robotPanel1 = new QTextEdit(centralWidget);
        robotPanel1->setObjectName(QString::fromUtf8("robotPanel1"));
        robotPanel1->setGeometry(QRect(20, 20, 91, 71));
        robotPanel2 = new QTextEdit(centralWidget);
        robotPanel2->setObjectName(QString::fromUtf8("robotPanel2"));
        robotPanel2->setGeometry(QRect(120, 20, 91, 71));
        obstacleLabel = new QLabel(centralWidget);
        obstacleLabel->setObjectName(QString::fromUtf8("obstacleLabel"));
        obstacleLabel->setGeometry(QRect(20, 100, 81, 17));
        obstaclePanel1 = new QTextEdit(centralWidget);
        obstaclePanel1->setObjectName(QString::fromUtf8("obstaclePanel1"));
        obstaclePanel1->setGeometry(QRect(20, 120, 61, 41));
        obstaclePanel2 = new QTextEdit(centralWidget);
        obstaclePanel2->setObjectName(QString::fromUtf8("obstaclePanel2"));
        obstaclePanel2->setGeometry(QRect(90, 120, 61, 41));
        obstaclePanel3 = new QTextEdit(centralWidget);
        obstaclePanel3->setObjectName(QString::fromUtf8("obstaclePanel3"));
        obstaclePanel3->setGeometry(QRect(160, 120, 61, 41));
        obstaclePanel4 = new QTextEdit(centralWidget);
        obstaclePanel4->setObjectName(QString::fromUtf8("obstaclePanel4"));
        obstaclePanel4->setGeometry(QRect(230, 120, 61, 41));
        obstaclePanel5 = new QTextEdit(centralWidget);
        obstaclePanel5->setObjectName(QString::fromUtf8("obstaclePanel5"));
        obstaclePanel5->setGeometry(QRect(300, 120, 61, 41));
        settingsLabel = new QLabel(centralWidget);
        settingsLabel->setObjectName(QString::fromUtf8("settingsLabel"));
        settingsLabel->setGeometry(QRect(20, 170, 67, 17));
        numRobotsLabel = new QLabel(centralWidget);
        numRobotsLabel->setObjectName(QString::fromUtf8("numRobotsLabel"));
        numRobotsLabel->setGeometry(QRect(40, 190, 101, 17));
        rowLengthLabel = new QLabel(centralWidget);
        rowLengthLabel->setObjectName(QString::fromUtf8("rowLengthLabel"));
        rowLengthLabel->setGeometry(QRect(40, 210, 101, 17));
        spacingLabel = new QLabel(centralWidget);
        spacingLabel->setObjectName(QString::fromUtf8("spacingLabel"));
        spacingLabel->setGeometry(QRect(40, 230, 131, 17));
        numRobotsField = new QLineEdit(centralWidget);
        numRobotsField->setObjectName(QString::fromUtf8("numRobotsField"));
        numRobotsField->setGeometry(QRect(180, 190, 113, 21));
        rowLengthField = new QLineEdit(centralWidget);
        rowLengthField->setObjectName(QString::fromUtf8("rowLengthField"));
        rowLengthField->setGeometry(QRect(180, 210, 113, 21));
        spacingField = new QLineEdit(centralWidget);
        spacingField->setObjectName(QString::fromUtf8("spacingField"));
        spacingField->setGeometry(QRect(180, 230, 113, 21));
        closeButton = new QPushButton(centralWidget);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));
        closeButton->setGeometry(QRect(110, 270, 99, 27));
        launchButton = new QPushButton(centralWidget);
        launchButton->setObjectName(QString::fromUtf8("launchButton"));
        launchButton->setGeometry(QRect(260, 270, 99, 27));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 469, 25));
        menuOrchard_Simulator = new QMenu(menuBar);
        menuOrchard_Simulator->setObjectName(QString::fromUtf8("menuOrchard_Simulator"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuOrchard_Simulator->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        robotLabel->setText(QApplication::translate("MainWindow", "Robots", 0, QApplication::UnicodeUTF8));
        obstacleLabel->setText(QApplication::translate("MainWindow", "Obstacles", 0, QApplication::UnicodeUTF8));
        settingsLabel->setText(QApplication::translate("MainWindow", "Settings", 0, QApplication::UnicodeUTF8));
        numRobotsLabel->setText(QApplication::translate("MainWindow", "Num robots", 0, QApplication::UnicodeUTF8));
        rowLengthLabel->setText(QApplication::translate("MainWindow", "Row length", 0, QApplication::UnicodeUTF8));
        spacingLabel->setText(QApplication::translate("MainWindow", "Pole/Trunk spacing", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("MainWindow", "Close", 0, QApplication::UnicodeUTF8));
        launchButton->setText(QApplication::translate("MainWindow", "Launch", 0, QApplication::UnicodeUTF8));
        menuOrchard_Simulator->setTitle(QApplication::translate("MainWindow", "Orchard Simulator", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
