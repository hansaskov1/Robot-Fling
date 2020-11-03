#include "mainwindow.h"
#include "./ui_mainwindow.h"

using namespace std::chrono;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
      _socket(this)
{
    ui->setupUi(this);

    c.calibrate2();
    c.connectToCam();

    //showVideo *video = new showVideo(ui->lImage, c);
    //QThreadPool::globalInstance()->start(video);

    for(int i = 0; i < 4; i++){
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

void MainWindow::onReadyRead()
{
    QByteArray datas = _socket.readAll();
}

void MainWindow::on_bSend_clicked()
{
    /*
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

    */
    rw::math::Vector3D<> PCal(0.400624689065891, 0.901530744085863, 0.042187492976487);
    rw::math::Rotation3D<double> RCal(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);

   RobotControl RC("127.0.0.1",PCal,RCal);

   rw::math::Vector3D<> ballPosition(0.2,0.2,0.1);
   RC.getBall(ballPosition,0.2);
}

void MainWindow::on_bOpenGrip_clicked()
{
    gripper.open();
    ui->statusbar->showMessage("Opening Gripper", 2000);
}

void MainWindow::on_bCloseGrip_clicked()
{
    if (speed != 0)
        gripper.close(force, size, speed);
    else if (force != 0)
        gripper.close(force);
    else
        gripper.close();
    ui->statusbar->showMessage("Closing Gripper", 2000);
}

void MainWindow::on_bCalibrate_clicked()
{
    c.calibrate();
    c.createTranformMatrix(worldCalImg);
    gripper.home(); //Send code til server til at ændre status af gripperen
    qDebug() << gripper.GetConnectStatus();
    ui->statusbar->showMessage("Calibrating Gripper", 3000);
}

void MainWindow::on_bSaveConnect_clicked()
{
    clientInfo client;
    //c.setIP(ui->liIP->text().toStdString());
    //c.setPort(ui->liPort->text().toInt());

    //QString newIP = QString::fromStdString(c.getIP());

    //_socket.connectToHost(QHostAddress(newIP), c.getPort());
    //connect(&_socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));

    gripper.Init();
    gripper.ToConnectToHost("192.168.100.10", 42069);
    ui->statusbar->showMessage("Connected", 3000);
}

void MainWindow::on_bDisconnect_clicked()
{
    gripper.disconnect();
    c.mRun = false;
}
