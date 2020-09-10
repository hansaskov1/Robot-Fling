#include "clientInfo.h"

clientInfo::clientInfo()
{


}

void clientInfo::setIP(const std::string &mIP)
{
    IP = mIP;
}

std::string clientInfo::getIP() const
{
    return IP;
}

void clientInfo::setPort(int mPort)
{
    port = mPort;
}

int clientInfo::getPort() const
{
    return port;
}
