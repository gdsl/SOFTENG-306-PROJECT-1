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
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *robotLabel;
    QLabel *obstacleLabel;
    QLabel *settingsLabel;
    QLabel *numRobotsLabel;
    QLabel *rowLengthLabel;
    QLabel *spacingLabel;
    QLineEdit *numRobotsField;
    QLineEdit *rowLengthField;
    QLineEdit *spacingField;
    QPushButton *closeButton;
    QPushButton *launchButton;
    QListWidget *robotList1;
    QListWidget *robotList2;
    QListWidget *obstacleList2;
    QListWidget *obstacleList1;
    QMenuBar *menuBar;
    QMenu *menuOrchard_Simulator;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(755, 485);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        robotLabel = new QLabel(centralWidget);
        robotLabel->setObjectName(QString::fromUtf8("robotLabel"));
        robotLabel->setGeometry(QRect(60, 0, 67, 17));
        obstacleLabel = new QLabel(centralWidget);
        obstacleLabel->setObjectName(QString::fromUtf8("obstacleLabel"));
        obstacleLabel->setGeometry(QRect(60, 150, 81, 17));
        settingsLabel = new QLabel(centralWidget);
        settingsLabel->setObjectName(QString::fromUtf8("settingsLabel"));
        settingsLabel->setGeometry(QRect(200, 290, 67, 17));
        numRobotsLabel = new QLabel(centralWidget);
        numRobotsLabel->setObjectName(QString::fromUtf8("numRobotsLabel"));
        numRobotsLabel->setGeometry(QRect(220, 310, 101, 17));
        rowLengthLabel = new QLabel(centralWidget);
        rowLengthLabel->setObjectName(QString::fromUtf8("rowLengthLabel"));
        rowLengthLabel->setGeometry(QRect(220, 330, 101, 17));
        spacingLabel = new QLabel(centralWidget);
        spacingLabel->setObjectName(QString::fromUtf8("spacingLabel"));
        spacingLabel->setGeometry(QRect(220, 350, 131, 17));
        numRobotsField = new QLineEdit(centralWidget);
        numRobotsField->setObjectName(QString::fromUtf8("numRobotsField"));
        numRobotsField->setGeometry(QRect(360, 310, 113, 21));
        rowLengthField = new QLineEdit(centralWidget);
        rowLengthField->setObjectName(QString::fromUtf8("rowLengthField"));
        rowLengthField->setGeometry(QRect(360, 330, 113, 21));
        spacingField = new QLineEdit(centralWidget);
        spacingField->setObjectName(QString::fromUtf8("spacingField"));
        spacingField->setGeometry(QRect(360, 350, 113, 21));
        closeButton = new QPushButton(centralWidget);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));
        closeButton->setGeometry(QRect(290, 390, 99, 27));
        launchButton = new QPushButton(centralWidget);
        launchButton->setObjectName(QString::fromUtf8("launchButton"));
        launchButton->setGeometry(QRect(440, 390, 99, 27));
        robotList1 = new QListWidget(centralWidget);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        robotList1->setObjectName(QString::fromUtf8("robotList1"));
        robotList1->setGeometry(QRect(60, 20, 181, 121));
        robotList2 = new QListWidget(centralWidget);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        robotList2->setObjectName(QString::fromUtf8("robotList2"));
        robotList2->setGeometry(QRect(250, 20, 181, 121));
        obstacleList2 = new QListWidget(centralWidget);
        new QListWidgetItem(obstacleList2);
        new QListWidgetItem(obstacleList2);
        new QListWidgetItem(obstacleList2);
        new QListWidgetItem(obstacleList2);
        new QListWidgetItem(obstacleList2);
        obstacleList2->setObjectName(QString::fromUtf8("obstacleList2"));
        obstacleList2->setGeometry(QRect(250, 170, 181, 101));
        obstacleList1 = new QListWidget(centralWidget);
        new QListWidgetItem(obstacleList1);
        new QListWidgetItem(obstacleList1);
        new QListWidgetItem(obstacleList1);
        new QListWidgetItem(obstacleList1);
        new QListWidgetItem(obstacleList1);
        obstacleList1->setObjectName(QString::fromUtf8("obstacleList1"));
        obstacleList1->setGeometry(QRect(60, 170, 181, 101));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 755, 25));
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

        const bool __sortingEnabled = robotList1->isSortingEnabled();
        robotList1->setSortingEnabled(false);
        robotList1->setSortingEnabled(__sortingEnabled);


        const bool __sortingEnabled1 = robotList2->isSortingEnabled();
        robotList2->setSortingEnabled(false);
        robotList2->setSortingEnabled(__sortingEnabled1);


        const bool __sortingEnabled2 = obstacleList2->isSortingEnabled();
        obstacleList2->setSortingEnabled(false);
        obstacleList2->setSortingEnabled(__sortingEnabled2);


        const bool __sortingEnabled3 = obstacleList1->isSortingEnabled();
        obstacleList1->setSortingEnabled(false);
        obstacleList1->setSortingEnabled(__sortingEnabled3);

        menuOrchard_Simulator->setTitle(QApplication::translate("MainWindow", "Orchard Simulator", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
