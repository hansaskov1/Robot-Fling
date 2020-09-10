#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "clientInfo.h"
#include <QDebug>
#include <QHostAddress>
#include <QString>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
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
    //qDebug() << datas;
    //QString datasAsString = QString(datas);
    //ui->lcdNumber->display(datasAsString);
//    ui->lineEdit_5->setText(_socket.readAll());
//    ui->lineEdit_6->setText(_socket.readAll
}



void MainWindow::on_bSaveConnect_clicked()
{
    clientInfo c;
    c.setIP(ui->lineEdit->text().toStdString());
    c.setPort(ui->lineEdit_2->text().toInt());

    QString newIP = QString::fromStdString(c.getIP());

    _socket.connectToHost(QHostAddress(newIP), c.getPort());
    connect(&_socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));

    ui->statusbar->showMessage("Connected", 3000);
}

void MainWindow::on_bDisconnect_clicked()
{
    qDebug() << _socket.write("BYE()\n");
   _socket.close();
}

void MainWindow::on_bOpenGrip_clicked()
{
    qDebug() << _socket.write("RELEASE()\n"); //Send code til server til at ændre status af gripperen
    ui->statusbar->showMessage("Opening Gripper", 2000);
}

void MainWindow::on_bCloseGrip_clicked()
{
    QByteArray grip;
    if (speed != 0)
        grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ", " + std::to_string(size) + ", " + std::to_string(speed) + ")\n");
    else if (force != 0)
        grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ")\n");
    else
        grip = QByteArray::fromStdString("GRIP()\n");
    qDebug() << _socket.write(grip); //Send code til server til at ændre status af gripperen
    ui->statusbar->showMessage("Closing Gripper " + grip, 2000);
}

void MainWindow::on_bSend_clicked()
{
    if (ui->lineEdit_3->hasAcceptableInput())
        size = ui->lineEdit_3->text().toDouble();
    if (ui->lineEdit_4->hasAcceptableInput())
        speed = ui->lineEdit_4->text().toDouble();
    if (ui->lineEdit_8->hasAcceptableInput())
        force = ui->lineEdit_8->text().toDouble();
}


void MainWindow::on_bCalibrate_clicked()
{
    qDebug() << _socket.write("HOME()\n"); //Send code til server til at ændre status af gripperen
    ui->statusbar->showMessage("Calibrating Gripper", 3000);
}
