#ifndef GRIPPER_H
#define GRIPPER_H

#include <iostream>
#include <QTcpSocket>
#include <QThread>
#include <QHostAddress>
#include <QDebug>
#include <QString>
#include <QObject>

class Gripper : public QObject
{
    Q_OBJECT

public:
    explicit Gripper (QObject *parent = nullptr) : QObject(parent) {
        connect(this,SIGNAL(ToConnectToHost(QString,quint16)),this,SLOT(ConnectToHost(QString,quint16)));
        connect(this, SIGNAL(ToSendData(QByteArray)), this, SLOT(SendData(QByteArray)));
        connect(this,SIGNAL(ToCloseSocket()),this,SLOT(CloseSocket()));
    }

    void Init() //Client initialization
    {
        if(!myThread)//Create thread start thread
            {
                myThread = new QThread;
                this->moveToThread(myThread);//Transfer to own thread
                myThread->start();
            }
    }

        bool GetConnectStatus()//Get the connection status with the server
        {
            return connectFlag;
        }

        void disconnect() {
            qDebug() << "Disconnect called";
            ToSendData("BYE()\n");
        }

        void open() {
            qDebug() << "Open called";
            ToSendData("RELEASE()\n");
        }

        void close() {
            qDebug() << "Close called";
            ToSendData("GRIP()\n");
        }

        void close(double force) {
            qDebug() << "Close called";
            QByteArray grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ")\n");
            ToSendData(grip);
        }

        void close(double force, double size, double speed) {
            qDebug() << "Close called";
            QByteArray grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ", " + std::to_string(size) + ", " + std::to_string(speed) + ")\n");
            ToSendData(grip);
        }

        void home() {
            qDebug() << "Home called";
            ToSendData("HOME()\n");
        }

    signals:
        //Signaling is used to ensure that slot functions are executed in child threads
        void ToConnectToHost(QString hostIp,quint16 hostPort); //Try to connect the server with random Ip port (no guarantee of successful connection)
        void ToSendData(QByteArray data);//Try to send data
        void ToCloseSocket();//Try to close Socket

private slots:
        void ConnectToHost(QString hostIp,quint16 hostPort)//Random Ip port connects to the server
        {
            if(!mySocket)//Create Socket
                {
                    mySocket = new QTcpSocket;
                    connect(mySocket, SIGNAL(readyRead()), this, SLOT(ReceiveData()));
                    connect(mySocket,&QTcpSocket::connected,this,[=]()
                    {
                        noAckTimes = 0;
                        connectFlag = true;
                    });
                    connect(mySocket,&QTcpSocket::disconnected,this,[=]()
                    {
                        connectFlag = false;
                    });
                }
            qDebug() << "Connecting";
                serverPort = hostPort;
                serverIp = hostIp;
                mySocket->abort();//Release connection
                mySocket->connectToHost(hostIp,hostPort);//Try to connect to the server
        }

        void ReceiveData() //Socket receive data slot function
        {
            noAckTimes = 0;
            if (mySocket->isReadable())
            {
                qDebug() << mySocket->readAll();
            }
        }

        void SendData(QByteArray data)//Send data to the server
        {
            if(connectFlag&&mySocket->isWritable())
                {
                    if (data.size() != mySocket->write(data))
                    {
                        qDebug() << "Noget gik gal";
                    }
                }
        }

        void CloseSocket()//Try to close Socket
        {
            if(mySocket)
            {
                SendData("BYE()\n");
                mySocket->close();
            }
        }

    private:
        QThread* myThread{ nullptr };//Own thread to prevent receiving and sending data from blocking other threads
        QTcpSocket* mySocket{ nullptr };
        bool connectFlag{false};//Connection status flag with server
        quint16 noAckTimes{0};//The number of times the server did not receive a reply
        quint16 serverPort{0};//Server port
        QString serverIp;//Server Ip
/*
    void connectGripper(std::string IP) {
        qDebug() << "Trying to connect";
        QString newIP = QString::fromStdString(IP);
        gripperSocket.connectToHost(QHostAddress(newIP), port);
        //connect(&gripperSocket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
        if (gripperSocket.waitForConnected(3000))
            qDebug() << "Connected";
        else {
            qDebug() << "Den gik ikke";
        }
    }

    void disconnect() {
       qDebug() << gripperSocket.write("BYE()\n");
       gripperSocket.close();
    }

    void open() {
        qDebug() << "Open called";
        qDebug() << gripperSocket.write("RELEASE()\n");
    }

    void close() {
        qDebug() << "Close called";
        qDebug() << gripperSocket.write("GRIP()\n");
    }

    void close(double force) {
        qDebug() << "Close called";
        QByteArray grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ")\n");
        qDebug() << gripperSocket.write(grip);
    }

    void close(double force, double size, double speed) {
        qDebug() << "Close called";
        QByteArray grip = QByteArray::fromStdString("GRIP(" + std::to_string(force) + ", " + std::to_string(size) + ", " + std::to_string(speed) + ")\n");
        qDebug() << gripperSocket.write(grip);
    }

    void home() {
        qDebug() << "Home called";
        qDebug() << gripperSocket.write("HOME()\n");
    }

    void setPort(int value) { port = value; }

private:
    int port = 42069;
    QTcpSocket gripperSocket;


public slots:
    void onReadyRead() { QByteArray datas = gripperSocket.readAll(); }*/
};

#endif // GRIPPER_H
