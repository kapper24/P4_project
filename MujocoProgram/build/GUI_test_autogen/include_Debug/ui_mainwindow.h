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
    QLabel *GraspGorithmLabel;
    QFrame *frame;
    QFrame *frame_2;
    QMenuBar *menubar;
    QMenu *menuAutonomous_Prosthesis_Control;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(40, 40, 40);\n"
"color: rgb(255, 255, 255);"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        StartQuitButton = new QPushButton(centralwidget);
        StartQuitButton->setObjectName(QString::fromUtf8("StartQuitButton"));
        StartQuitButton->setGeometry(QRect(30, 50, 151, 101));
        StartQuitButton->setMaximumSize(QSize(151, 16777215));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(255, 141, 1, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush1);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Active, QPalette::PlaceholderText, brush);
#endif
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush1);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush);
#endif
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush);
#endif
        StartQuitButton->setPalette(palette);
        StartQuitButton->setAutoFillBackground(false);
        StartQuitButton->setStyleSheet(QString::fromUtf8("font: 75 16pt \"MS Shell Dlg 2\";\n"
"background-color: rgb(255, 141, 1);\n"
"alternate-background-color: rgb(255, 141, 1);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255, 255, 255);\n"
""));
        OpenCloseButton = new QPushButton(centralwidget);
        OpenCloseButton->setObjectName(QString::fromUtf8("OpenCloseButton"));
        OpenCloseButton->setGeometry(QRect(30, 210, 151, 101));
        OpenCloseButton->setStyleSheet(QString::fromUtf8("font: 75 16pt \"MS Shell Dlg 2\";\n"
"background-color: rgb(255, 141, 1);\n"
"\n"
"color: rgb(255, 255, 255);"));
        GraspGorithmLabel = new QLabel(centralwidget);
        GraspGorithmLabel->setObjectName(QString::fromUtf8("GraspGorithmLabel"));
        GraspGorithmLabel->setGeometry(QRect(650, 520, 151, 31));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(290, 30, 491, 461));
        frame->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 141, 1);"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        frame_2 = new QFrame(frame);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(20, 20, 451, 421));
        frame_2->setStyleSheet(QString::fromUtf8("background-color: rgb(40, 40, 40);"));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
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
        GraspGorithmLabel->setText(QCoreApplication::translate("MainWindow", "Powered By GraspGorithm", nullptr));
        menuAutonomous_Prosthesis_Control->setTitle(QCoreApplication::translate("MainWindow", "Autonomous Prosthesis Control", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
