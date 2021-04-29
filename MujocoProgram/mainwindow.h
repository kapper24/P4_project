#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qtimer.h>
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "vector"
#include "iostream"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
class QPushButton;
class QLabel;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

	

	
	
    
private slots:
    void on_OpenCloseButton_clicked();
    void on_StartQuitButton_clicked();
    void PCLupdate();
private:
    Ui::MainWindow *ui;
    QPushButton* ui_OpenCloseButton;
    QPushButton* ui_StartQuitButton;
    QLabel* ui_GraspGorithmLabel;
};
#endif // MAINWINDOW_H
