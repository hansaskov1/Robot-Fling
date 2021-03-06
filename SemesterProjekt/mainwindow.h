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
//#include "DetectCollision.h"
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
    virtual ~MainWindow();

private:
    void showVideo();

private slots:
    void on_bSend_clicked();

    void on_bCalibrate_clicked();

    void on_bSaveConnect_clicked();

    void on_bDisconnect_clicked();

    void on_cbDB_currentIndexChanged(const QString&);

    void on_cbManual_currentIndexChanged(const QString &arg1);

    void on_pbManualApply_clicked();

    void on_pbManualStart_clicked();

    void on_pbGetBall_clicked();

    void on_pbOpenClose_clicked();

private:
    Ui::MainWindow *ui;
    Calibration c;
    ObjectDetection o;
    cv::Mat image;
    RobotControl RC;
    SQLController sql;
    std::thread cameraConnect;
    std::thread videoThread;
    std::thread RCthread;
    std::thread SQLThread;
    rw::math::Vector3D<> mBallPosition;
    rw::math::Vector3D<> mCupPosition;
    rw::math::Vector3D<> mReleasePosition;
    double mAngle;
    double mOffset;
    double mAcceleration;
    double mSpeed;
};

#endif // MAINWINDOW_H
