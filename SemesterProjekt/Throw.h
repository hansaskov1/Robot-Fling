#ifndef THROW_H
#define THROW_H

#include <QString>
#include <vector>
#include "path.h"
#include <rw/math.hpp>


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




private:
    unsigned int mThrowID;
    QString mObject;
    double mAngle;
    double mSpeed;
    std::vector<Path> mPaths;
    bool mSuccess;
    double acceleration = 0.5;
};

#endif // THROW_H
