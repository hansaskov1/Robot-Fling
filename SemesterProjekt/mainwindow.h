#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "Gripper.h"
#include <string.h>
#include <QMainWindow>
#include <QTcpSocket>
#include <QDebug>
#include <QHostAddress>
#include <QString>
#include <QTimer>
#include "calibration.h"
#include "objectdetection.h"
#include "showvideo.h"
#include "DetectCollision.h"
#include "RobotControl.h"
#include "sqlcontroller.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_bSend_clicked();

    void on_bOpenGrip_clicked();

    void on_bCloseGrip_clicked();

    void on_bCalibrate_clicked();

    void on_bSaveConnect_clicked();

    void on_bDisconnect_clicked();

private:
    Ui::MainWindow *ui;
    Calibration c;
    ObjectDetection o;
    cv::Mat image;
    cv::Mat worldCalImg[4];
    RobotControl RC;
    SQLController sql;
};

#endif // MAINWINDOW_H
