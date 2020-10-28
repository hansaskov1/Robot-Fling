#ifndef THROW_H
#define THROW_H

#include <QString>

class Throw
{
public:
    Throw(){}
    Throw(unsigned int kastId, QString objekt, double vinkel, double hastighed, bool success) {
        this->kastId = kastId;
        this->objekt = objekt;
        this->vinkel = vinkel;
        this->hastighed = hastighed;
        this->success = success;
    }

private:
    unsigned int kastId;
    QString objekt;
    double vinkel;
    double hastighed;
    bool success;
};

#endif // THROW_H
