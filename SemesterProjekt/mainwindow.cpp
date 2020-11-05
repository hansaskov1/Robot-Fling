#include "mainwindow.h"
#include "./ui_mainwindow.h"

using namespace std::chrono;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    c.calibrate2();
    c.connectToCam();

    for(int i = 0; i < 4; i++) {
        cv::String path = "";
        path = "../Images/BallWorldCordsROI/img" + std::to_string(i) + ".png";
        worldCalImg[i] = cv::imread(path, cv::IMREAD_COLOR);
    }

    c.createTranformMatrix(worldCalImg);
}

MainWindow::~MainWindow()
{
    delete ui;
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

    //const uchar *qImageBuffer = (const uchar*)image.data;
    //QImage img(qImageBuffer, image.cols, image.rows, image.step, QImage::Format_RGB888);
    //img.setColorTable(colorTable);
    //pixmap = QPixmap::fromImage(img);
    //ui->lImage->setPixmap(pixmap);
    /*
    QImage imgIn = QImage((uchar*)image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    ui->lImage->setPixmap(QPixmap::fromImage(imgIn));
    ui->lImage->setScaledContents(true);
*/
    if (ballPos.x && ballPos.y)
    {
        rw::math::Vector3D<> ballPosition(camToBall.at<float>(0,3)*0.01,camToBall.at<float>(1,3)*0.01,camToBall.at<float>(2,3)*0.01);
        //rw::math::Vector3D<> ballPosition(0.2,0.2,0.1);
        RC.getBall(ballPosition,0.2);
    }
}

void MainWindow::on_bOpenGrip_clicked()
{
    //gripper.open();
    ui->statusbar->showMessage("Opening Gripper", 2000);
}

void MainWindow::on_bCloseGrip_clicked()
{
    /*
    if (speed != 0)
        gripper.close(force, size, speed);
    else if (force != 0)
        gripper.close(force);
    else
        gripper.close();
    */
    ui->statusbar->showMessage("Closing Gripper", 2000);
}

void MainWindow::on_bCalibrate_clicked()
{
    c.calibrate();
    c.createTranformMatrix(worldCalImg);
    //gripper.home(); //Send code til server til at ændre status af gripperen
    //qDebug() << gripper.GetConnectStatus();
    ui->statusbar->showMessage("Calibrating Gripper", 3000);
}

void MainWindow::on_bSaveConnect_clicked()
{
    showVideo *video = new showVideo(ui->lImage, &c);
    QThreadPool::globalInstance()->start(video);

    rw::math::Vector3D<> PCal(0.400624689065891, 0.901530744085863, 0.042187492976487);
    rw::math::Rotation3D<double> RCal(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);

    RC.setParam("192.168.100.49", "192.168.100.10", PCal, RCal);

    //gripper.Init();
    //gripper.ToConnectToHost("192.168.100.10", 1000);
    ui->statusbar->showMessage("Connected", 3000);
}

void MainWindow::on_bDisconnect_clicked()
{
    //gripper.disconnect();
    c.mRun = false;
}
