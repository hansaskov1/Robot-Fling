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

    double speed(double vinkel, rw::math::Vector3D<> throwPose, rw::math::Vector3D<> cupPose)
    {
        double xDistanceToCup = throwPose[0] - cupPose[0];
        double yDistanceToCup = throwPose[1] - cupPose[1];
        double zDistanceToCup = throwPose[2] - cupPose[2];

        double den = gravity*(pow(xDistanceToCup,2)+pow(yDistanceToCup,2));
        double num = 2 * cos(pow(vinkel,2)) * tan(vinkel) * (sqrt(pow(xDistanceToCup,2)+pow(yDistanceToCup,2)))+zDistanceToCup;

        hastighed = sqrt(den/num);
        return hastighed;
    }

    rw::math::Vector3D<> rampPose(double hastighed, double acceleration, double vinkel, rw::math::Vector3D<> throwPose)
    {
        double timeFromThrowPose = hastighed/acceleration;
        double lenghtToRampPose = 0.5 * acceleration * pow(timeFromThrowPose,2);

        std::vector<double> startRampPose;

        double xLenght = lenghtToRampPose * cos(vinkel);
        double yLenght = 0;
        double zLenght = lenghtToRampPose * sin(vinkel);

        startRampPose.push_back(xLenght);
        startRampPose.push_back(yLenght);
        startRampPose.push_back(zLenght);

        return startRampPose;
    }


private:
    unsigned int kastId;
    QString objekt;
    double vinkel;
    double hastighed;
    bool success;
    double gravity = 9.82;
    double acceleration = 0.5;
};

#endif // THROW_H
