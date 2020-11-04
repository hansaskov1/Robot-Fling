#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "clientInfo.h"
#include "Gripper.h"
#include <string.h>
#include <QMainWindow>
#include <QTcpSocket>
#include <QDebug>
#include <QHostAddress>
#include <QString>
#include "calibration.h"
#include "objectdetection.h"
#include "showvideo.h"
#include "RobotControl.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onReadyRead();

private slots:
    void on_bSend_clicked();

    void on_bOpenGrip_clicked();

    void on_bCloseGrip_clicked();

    void on_bCalibrate_clicked();

    void on_bSaveConnect_clicked();

    void on_bDisconnect_clicked();

    static void grib(rw::math::Vector3D<> pos);

    private:
    Ui::MainWindow *ui;
    QTcpSocket _socket;
    quint32 newIP;
    double force = 0, speed = 0, size = 0;
    Calibration c;
    ObjectDetection o;
    cv::Mat image;
    cv::Mat worldCalImg[4];
    QPixmap pixmap;
    Gripper gripper;
    RobotControl RC;
};

#endif // MAINWINDOW_H
