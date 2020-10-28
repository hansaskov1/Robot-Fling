#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
      _socket(this)
{
    ui->setupUi(this);
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
    if (ui->liSize->hasAcceptableInput())
        size = ui->liSize->text().toDouble();
    if (ui->liSpeed->hasAcceptableInput())
        speed = ui->liSpeed->text().toDouble();
    if (ui->liForce->hasAcceptableInput())
        force = ui->liForce->text().toDouble();
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
    gripper.home(); //Send code til server til at ændre status af gripperen
    qDebug() << gripper.GetConnectStatus();
    ui->statusbar->showMessage("Calibrating Gripper", 3000);
}

void MainWindow::on_bSaveConnect_clicked()
{
    clientInfo c;
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
}
