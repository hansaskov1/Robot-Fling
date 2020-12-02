#include "mainwindow.h"
#include "./ui_mainwindow.h"

using namespace std::chrono;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
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
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
}

void MainWindow::on_bSend_clicked()
{
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
        rw::math::Vector3D<> ballPosition(camToBall.at<float>(0,3)*0.01,camToBall.at<float>(1,3)*0.01,camToBall.at<float>(2,3)*0.01);
        //rw::math::Vector3D<> ballPosition(0.2,0.2,0.1);

        RC.getBall(ballPosition,0.05);

        if (ui->cbDB->currentIndex())
            if (sql.insert(RC.getThrow()))
                ui->statusbar->showMessage("Inserted to database", 3000);
    }
}

void MainWindow::on_bCalibrate_clicked()
{
    c.calibrate();
    ui->statusbar->showMessage("Calibrating", 3000);
}

void MainWindow::on_bSaveConnect_clicked()
{
    int celleNr = ui->cbCell->currentIndex()+1;

    c.init(celleNr);
    c.connectToCam();

    thread = std::thread(&MainWindow::showVideo, this);

    RC.setParam(ui->liIP->text().toStdString(), ui->liGripperIP->text(), celleNr);
}

void MainWindow::on_bDisconnect_clicked()
{
    RC.disconnect();
    c.mRun = false;
    if (thread.joinable())
        thread.join();
}

void MainWindow::on_cbDB_currentIndexChanged(const QString &arg1)
{
    if (ui->cbDB->currentIndex())
        if (sql.connect("192.168.221.1", "ubuntu", "Tarzan12!", "throwdb"))
            ui->statusbar->showMessage("Connected", 3000);
}
