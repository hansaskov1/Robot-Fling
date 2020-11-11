#ifndef THROW_H
#define THROW_H

#include <QString>
#include <vector>
#include "path.h"


class Throw
{
public:
    Throw() {}

    Throw(unsigned int throwID, QString object, double angle, double speed, std::vector<Path> paths, bool success) {
        mThrowID = throwID;
        mObject = object;
        mAngle = angle;
        mSpeed = speed;
        mPaths = paths;
        mSuccess = success;
    }

    unsigned int getThrowID() const {
        return mThrowID;
    }

    void setThrowID(unsigned int throwID) {
        mThrowID = throwID;
    }

    QString getObject() const {
        return mObject;
    }

    void setObject(const QString &object) {
        mObject = object;
    }

    double getAngle() const {
        return mAngle;
    }

    void setAngle(double angle) {
        mAngle = angle;
    }

    double getSpeed() const {
        return mSpeed;
    }

    void setSpeed(double speed) {
        mSpeed = speed;
    }

    bool isSuccess() const {
        return mSuccess;
    }

    void setSuccess(bool success) {
        mSuccess = success;
    }

    void addPath (Path path) {
        mPaths.push_back(path);
    }

    std::vector<Path> getPaths () {
        mPaths.shrink_to_fit();
        return mPaths;
    }
/*
    double speed(double vinkel, rw::math::Vector3D<> throwPose, rw::math::Vector3D<> cupPose)
    {
        double xDistanceToCup = throwPose[0] - cupPose[0];
        double yDistanceToCup = throwPose[1] - cupPose[1];
        double zDistanceToCup = throwPose[2] - cupPose[2];

        double den = gravity*(pow(xDistanceToCup,2)+pow(yDistanceToCup,2));
        double num = 2 * cos(pow(vinkel,2)) * tan(vinkel) * (sqrt(pow(xDistanceToCup,2)+pow(yDistanceToCup,2)))+zDistanceToCup;

        mSpeed = sqrt(den/num);
        return mSpeed;
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
*/

private:
    unsigned int mThrowID;
    QString mObject;
    double mAngle;
    double mSpeed;
    std::vector<Path> mPaths;
    bool mSuccess;
    double gravity = 9.82;
    double acceleration = 0.5;
};

#endif // THROW_H
