#ifndef CLIENTINFO_H
#define CLIENTINFO_H

#include <iostream>
#include <QHostAddress>

class clientInfo
{
public:
    clientInfo();

    void setIP(const std::string &mIP);
    std::string getIP() const;

    void setPort(int mPort);
    int getPort() const;


    std::string IP = "127.0.0.1";
    int port = 0;

};

#endif // CLIENTINFO_H
