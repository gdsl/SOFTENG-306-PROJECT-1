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
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *robotLabel;
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
    QListWidget *robotList1;
    QListWidget *robotList2;
    QListWidget *robotList3;
    QMenuBar *menuBar;
    QMenu *menuOrchard_Simulator;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(570, 394);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        robotLabel = new QLabel(centralWidget);
        robotLabel->setObjectName(QString::fromUtf8("robotLabel"));
        robotLabel->setGeometry(QRect(60, 0, 67, 17));
        obstacleLabel = new QLabel(centralWidget);
        obstacleLabel->setObjectName(QString::fromUtf8("obstacleLabel"));
        obstacleLabel->setGeometry(QRect(60, 130, 81, 17));
        obstaclePanel1 = new QTextEdit(centralWidget);
        obstaclePanel1->setObjectName(QString::fromUtf8("obstaclePanel1"));
        obstaclePanel1->setGeometry(QRect(60, 150, 61, 41));
        obstaclePanel2 = new QTextEdit(centralWidget);
        obstaclePanel2->setObjectName(QString::fromUtf8("obstaclePanel2"));
        obstaclePanel2->setGeometry(QRect(130, 150, 61, 41));
        obstaclePanel3 = new QTextEdit(centralWidget);
        obstaclePanel3->setObjectName(QString::fromUtf8("obstaclePanel3"));
        obstaclePanel3->setGeometry(QRect(200, 150, 61, 41));
        obstaclePanel4 = new QTextEdit(centralWidget);
        obstaclePanel4->setObjectName(QString::fromUtf8("obstaclePanel4"));
        obstaclePanel4->setGeometry(QRect(270, 150, 61, 41));
        obstaclePanel5 = new QTextEdit(centralWidget);
        obstaclePanel5->setObjectName(QString::fromUtf8("obstaclePanel5"));
        obstaclePanel5->setGeometry(QRect(340, 150, 61, 41));
        settingsLabel = new QLabel(centralWidget);
        settingsLabel->setObjectName(QString::fromUtf8("settingsLabel"));
        settingsLabel->setGeometry(QRect(60, 200, 67, 17));
        numRobotsLabel = new QLabel(centralWidget);
        numRobotsLabel->setObjectName(QString::fromUtf8("numRobotsLabel"));
        numRobotsLabel->setGeometry(QRect(80, 220, 101, 17));
        rowLengthLabel = new QLabel(centralWidget);
        rowLengthLabel->setObjectName(QString::fromUtf8("rowLengthLabel"));
        rowLengthLabel->setGeometry(QRect(80, 240, 101, 17));
        spacingLabel = new QLabel(centralWidget);
        spacingLabel->setObjectName(QString::fromUtf8("spacingLabel"));
        spacingLabel->setGeometry(QRect(80, 260, 131, 17));
        numRobotsField = new QLineEdit(centralWidget);
        numRobotsField->setObjectName(QString::fromUtf8("numRobotsField"));
        numRobotsField->setGeometry(QRect(220, 220, 113, 21));
        rowLengthField = new QLineEdit(centralWidget);
        rowLengthField->setObjectName(QString::fromUtf8("rowLengthField"));
        rowLengthField->setGeometry(QRect(220, 240, 113, 21));
        spacingField = new QLineEdit(centralWidget);
        spacingField->setObjectName(QString::fromUtf8("spacingField"));
        spacingField->setGeometry(QRect(220, 260, 113, 21));
        closeButton = new QPushButton(centralWidget);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));
        closeButton->setGeometry(QRect(150, 300, 99, 27));
        launchButton = new QPushButton(centralWidget);
        launchButton->setObjectName(QString::fromUtf8("launchButton"));
        launchButton->setGeometry(QRect(300, 300, 99, 27));
        robotList1 = new QListWidget(centralWidget);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        new QListWidgetItem(robotList1);
        robotList1->setObjectName(QString::fromUtf8("robotList1"));
        robotList1->setGeometry(QRect(60, 20, 141, 101));
        robotList2 = new QListWidget(centralWidget);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        new QListWidgetItem(robotList2);
        robotList2->setObjectName(QString::fromUtf8("robotList2"));
        robotList2->setGeometry(QRect(210, 20, 141, 101));
        robotList3 = new QListWidget(centralWidget);
        new QListWidgetItem(robotList3);
        new QListWidgetItem(robotList3);
        new QListWidgetItem(robotList3);
        new QListWidgetItem(robotList3);
        new QListWidgetItem(robotList3);
        robotList3->setObjectName(QString::fromUtf8("robotList3"));
        robotList3->setGeometry(QRect(360, 20, 141, 101));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 570, 25));
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
        QListWidgetItem *___qlistwidgetitem = robotList1->item(0);
        ___qlistwidgetitem->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem1 = robotList1->item(1);
        ___qlistwidgetitem1->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem2 = robotList1->item(2);
        ___qlistwidgetitem2->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem3 = robotList1->item(3);
        ___qlistwidgetitem3->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem4 = robotList1->item(4);
        ___qlistwidgetitem4->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        robotList1->setSortingEnabled(__sortingEnabled);


        const bool __sortingEnabled1 = robotList2->isSortingEnabled();
        robotList2->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem5 = robotList2->item(0);
        ___qlistwidgetitem5->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem6 = robotList2->item(1);
        ___qlistwidgetitem6->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem7 = robotList2->item(2);
        ___qlistwidgetitem7->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem8 = robotList2->item(3);
        ___qlistwidgetitem8->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem9 = robotList2->item(4);
        ___qlistwidgetitem9->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        robotList2->setSortingEnabled(__sortingEnabled1);


        const bool __sortingEnabled2 = robotList3->isSortingEnabled();
        robotList3->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem10 = robotList3->item(0);
        ___qlistwidgetitem10->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem11 = robotList3->item(1);
        ___qlistwidgetitem11->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem12 = robotList3->item(2);
        ___qlistwidgetitem12->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem13 = robotList3->item(3);
        ___qlistwidgetitem13->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem14 = robotList3->item(4);
        ___qlistwidgetitem14->setText(QApplication::translate("MainWindow", "New Item", 0, QApplication::UnicodeUTF8));
        robotList3->setSortingEnabled(__sortingEnabled2);

        menuOrchard_Simulator->setTitle(QApplication::translate("MainWindow", "Orchard Simulator", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
