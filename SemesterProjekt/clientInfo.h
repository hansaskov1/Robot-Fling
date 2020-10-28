#ifndef CLIENTINFO_H
#define CLIENTINFO_H
#include <iostream>
#include <QHostAddress>

class clientInfo
{
public:
    clientInfo(){}
    std::string getIP() const { return IP; }
    void setIP(const std::string &mIP) { IP = mIP; }

    int getPort() const { return port; }
    void setPort(int mPort) { port = mPort; }

private:
    std::string IP = "127.0.0.1";
    int port = 0;
};
#endif // CLIENTINFO_H
