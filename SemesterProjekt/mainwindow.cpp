    #include "mainwindow.h"
#include "./ui_mainwindow.h"

using namespace std::chrono;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mBallPosition = rw::math::Vector3D<>(0.2, 0.2, 0.1);
    mCupPosition = rw::math::Vector3D<>(0.55, 0.5, 0.1);
    mReleasePosition = rw::math::Vector3D<>(0.6, 0.95, 0.7);
    mAngle = 3.1415/8;
    mOffset = 0.5;

    ui->lXValue->setText(QString::number(mBallPosition[0]));
    ui->lYValue->setText(QString::number(mBallPosition[1]));
    ui->lZValue->setText(QString::number(mBallPosition[2]));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showVideo()
{
    while(c.mRun) {
        image = c.getImage();
        ui->lImage->setPixmap(QPixmap::fromImage(QImage((const uchar*)image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped()));
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }
}

void MainWindow::on_bSend_clicked()
{
    if (cameraConnect.joinable())
        cameraConnect.join();

    cv::Point ballPos;
    cv::Mat camToBall;

    switch (ui->cbMethod->currentIndex()) {
    case 0:
        image = c.getImage();
        ballPos = o.colorMorphLineByLine(image);
        camToBall = c.calcTransMatCamToWorld(ballPos);
        std::cout << camToBall << std::endl;
        break;
    case 1:
        image = c.getImage();
        ballPos = o.hcBallCenterPosition(image,10,30);
        camToBall = c.calcTransMatCamToWorld(ballPos);
        std::cout << camToBall << std::endl;
        break;
    }

    if (ballPos.x && ballPos.y)
    {
        if (RCthread.joinable())
            RCthread.join();
        if (SQLThread.joinable())
            SQLThread.join();
        rw::math::Vector3D<> ballPosition(camToBall.at<float>(0,3)*0.01,camToBall.at<float>(1,3)*0.01,0.01);  //camToBall.at<float>(2,3)*0.01

        RCthread = std::thread([=] {RC.getBall(ballPosition); RC.circleThrow(rw::math::Vector3D<>(0.4, 0.2, 0.05), ui->sbAngle->value());});

        if (ui->cbDB->currentIndex()) {
            SQLThread = std::thread([=] {RCthread.join(); sql.insert(RC.getThrow());});
        }
    }
}

void MainWindow::on_bCalibrate_clicked()
{
    cameraConnect = std::thread([=] {c.calibrate();});
    ui->statusbar->showMessage("Calibrating", 3000);
}

void MainWindow::on_bSaveConnect_clicked()
{
    int celleNr = ui->cbCell->currentIndex()+1;

    cameraConnect = std::thread([=] {c.init(celleNr); c.connectToCam();});

    videoThread = std::thread([=] {cameraConnect.join(); showVideo();});

    RC.setParam(ui->liIP->text().toStdString(), ui->liGripperIP->text(), celleNr);

    ui->bSaveConnect->setDisabled(1);
}

void MainWindow::on_bDisconnect_clicked()
{
    c.mRun = false;
    ui->bDisconnect->setDisabled(1);
    RC.disconnect();
    if (RCthread.joinable())
        RCthread.join();
    if (SQLThread.joinable())
        SQLThread.join();
    if (videoThread.joinable())
        videoThread.join();
    ui->bSaveConnect->setEnabled(1);
}

void MainWindow::on_cbDB_currentIndexChanged(const QString &arg1)
{
    if (ui->cbDB->currentIndex())
        if (!sql.connect("192.168.221.1", "ubuntu", "Tarzan12!", "throwdb"))
            ui->cbDB->setCurrentIndex(0);
}

void MainWindow::on_cbManual_currentIndexChanged(const QString &arg1)
{
    switch (ui->cbManual->currentIndex()) {
    case 0:
        ui->lXValue->setText(QString::number(mBallPosition[0]));
        ui->lYValue->show();
        ui->lYValue->setText(QString::number(mBallPosition[1]));
        ui->lZValue->show();
        ui->lZValue->setText(QString::number(mBallPosition[2]));
        break;
    case 1:
        ui->lXValue->setText(QString::number(mCupPosition[0]));
        ui->lYValue->show();
        ui->lYValue->setText(QString::number(mCupPosition[1]));
        ui->lZValue->show();
        ui->lZValue->setText(QString::number(mCupPosition[2]));
        break;
    case 2:
        ui->lXValue->setText(QString::number(mReleasePosition[0]));
        ui->lYValue->show();
        ui->lYValue->setText(QString::number(mReleasePosition[1]));
        ui->lZValue->show();
        ui->lZValue->setText(QString::number(mReleasePosition[2]));
        break;
    case 3:
        ui->lXValue->setText(QString::number(mAngle));
        ui->lYValue->hide();
        ui->lZValue->hide();
        break;
    case 4:
        ui->lXValue->setText(QString::number(mOffset));
        ui->lYValue->hide();
        ui->lZValue->hide();
        break;
    }
}

void MainWindow::on_pbManualApply_clicked()
{
    switch (ui->cbManual->currentIndex()) {
    case 0:
        mBallPosition[0] = ui->lXValue->text().toDouble();
        mBallPosition[1] = ui->lYValue->text().toDouble();
        mBallPosition[2] = ui->lZValue->text().toDouble();
        break;
    case 1:
        mCupPosition[0] = ui->lXValue->text().toDouble();
        mCupPosition[1] = ui->lYValue->text().toDouble();
        mCupPosition[2] = ui->lZValue->text().toDouble();
        break;
    case 2:
        mReleasePosition[0] = ui->lXValue->text().toDouble();
        mReleasePosition[1] = ui->lYValue->text().toDouble();
        mReleasePosition[2] = ui->lZValue->text().toDouble();
        break;
    case 3:
        mAngle = ui->lXValue->text().toDouble();
        break;
    case 4:
        mOffset = ui->lXValue->text().toDouble();
        break;
    }
}

void MainWindow::on_pbManualStart_clicked()
{
    if (RCthread.joinable())
        RCthread.join();
    if (SQLThread.joinable())
        SQLThread.join();
    RCthread = std::thread([=] {RC.throwBallLinear(mCupPosition, mReleasePosition, mAngle, mOffset);});

    if (ui->cbDB->currentIndex()) {
        SQLThread = std::thread([=] {RCthread.join(); sql.insert(RC.getThrow());});
    }
}

void MainWindow::on_pbGetBall_clicked()
{
    if (RCthread.joinable())
        RCthread.join();
    RCthread = std::thread([=] {RC.getBall(mBallPosition, 0.05);});
}

void MainWindow::on_pbOpenClose_clicked()
{
    if (RCthread.joinable())
        RCthread.join();
    RCthread = std::thread([=] {RC.toggleGripper();});
}
