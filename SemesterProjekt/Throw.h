#ifndef THROW_H
#define THROW_H

#include <QString>
#include <vector>
#include "path.h"


class Throw
{
public:
    Throw() {}

    Throw(unsigned int kastID, QString objekt, double vinkel, double hastighed, std::vector<Path> paths, bool success) {
        mKastID = kastID;
        mObjekt = objekt;
        mVinkel = vinkel;
        mHastighed = hastighed;
        mPaths = paths;
        mSuccess = success;
    }

    unsigned int getKastID() const {
        return mKastID;
    }

    void setKastID(unsigned int kastID) {
        mKastID = kastID;
    }

    QString getObjekt() const {
        return mObjekt;
    }

    void setObjekt(const QString &objekt) {
        mObjekt = objekt;
    }

    double getVinkel() const {
        return mVinkel;
    }

    void setVinkel(double vinkel) {
        mVinkel = vinkel;
    }

    double getHastighed() const {
        return mHastighed;
    }

    void setHastighed(double hastighed) {
        mHastighed = hastighed;
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
    unsigned int mKastID;
    QString mObjekt;
    double mVinkel;
    double mHastighed;
    std::vector<Path> mPaths;
    bool mSuccess;
};

#endif // THROW_H
