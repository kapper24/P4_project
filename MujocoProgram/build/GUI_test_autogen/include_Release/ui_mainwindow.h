/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.0.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *StartQuitButton;
    QPushButton *OpenCloseButton;
    QLabel *label;
    QFrame *frame;
    QMenuBar *menubar;
    QMenu *menuAutonomous_Prosthesis_Control;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(93, 93, 93);\n"
"alternate-background-color: rgb(255, 255, 255);\n"
"color: rgb(255, 255, 255);"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        StartQuitButton = new QPushButton(centralwidget);
        StartQuitButton->setObjectName(QString::fromUtf8("StartQuitButton"));
        StartQuitButton->setGeometry(QRect(30, 50, 151, 101));
        StartQuitButton->setMaximumSize(QSize(151, 16777215));
        StartQuitButton->setAutoFillBackground(false);
        StartQuitButton->setStyleSheet(QString::fromUtf8("font: 75 14pt \"MS Shell Dlg 2\";\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255, 255, 255);\n"
""));
        OpenCloseButton = new QPushButton(centralwidget);
        OpenCloseButton->setObjectName(QString::fromUtf8("OpenCloseButton"));
        OpenCloseButton->setGeometry(QRect(30, 210, 151, 101));
        OpenCloseButton->setStyleSheet(QString::fromUtf8("font: 75 14pt \"MS Shell Dlg 2\";\n"
"\n"
"color: rgb(255, 255, 255);"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(630, 510, 151, 31));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(270, 50, 471, 431));
        frame->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(255, 255, 255);"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 26));
        menuAutonomous_Prosthesis_Control = new QMenu(menubar);
        menuAutonomous_Prosthesis_Control->setObjectName(QString::fromUtf8("menuAutonomous_Prosthesis_Control"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuAutonomous_Prosthesis_Control->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        StartQuitButton->setText(QCoreApplication::translate("MainWindow", "START/QUIT", nullptr));
        OpenCloseButton->setText(QCoreApplication::translate("MainWindow", "OPEN/CLOSE", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Powered By GraspGorithm", nullptr));
        menuAutonomous_Prosthesis_Control->setTitle(QCoreApplication::translate("MainWindow", "Autonomous Prosthesis Control", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
