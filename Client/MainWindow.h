#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <string.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onReadyRead();

private slots:
    void on_bSaveConnect_clicked();

    void on_bOpenGrip_clicked();

    void on_bCloseGrip_clicked();

    void on_bDisconnect_clicked();

    void on_bSend_clicked();


    void on_bCalibrate_clicked();

private:
    Ui::MainWindow *ui;
    QTcpSocket  _socket;

    quint32 newIP;
    double force = 0, speed = 0, size = 0;
};

#endif // MAINWINDOW_H
