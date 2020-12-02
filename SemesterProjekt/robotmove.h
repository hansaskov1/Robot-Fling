#ifndef ROBOTMOVE_H
#define ROBOTMOVE_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "rw/math.hpp"
#include "Gripper.h"
#include "path.h"
#include <thread>
#include <future>


class RobotMove
{
public:
    RobotMove();
    RobotMove(unsigned int  getMsInterval, Gripper * gripper ,ur_rtde::RTDEControlInterface * rtdeControl, ur_rtde::RTDEReceiveInterface * rtdeRecieve, double getSpeed, double acceleration);

    std::vector<double> vecRPY2stdVec(rw::math::Vector3D<> position, rw::math::RPY<> RPY);

    void fetchPath(std::promise<Path> && returnPath ,std::atomic<bool>& stop);
    void fetchPathLRelease(std::promise<Path> && returnPath ,std::atomic<bool>& stop,  rw::math::Vector3D<> releasePosi, double maxOffset);
    void fetchPathJRelease(std::promise<Path> && returnPath ,std::atomic<bool>& stop,  rw::math::Q releasePose, double maxOffset);

    Path moveRobotL(rw::math::Vector3D<> position ,rw::math::RPY<> orientation);
    Path moveRobotL(rw::math::Q);
    Path moveRobotJ(rw::math::Q);
    Path moveRobotJ(rw::math::Vector3D<> position ,rw::math::RPY<> orientation);
    Path moveRobotLRelease(rw::math::Vector3D<> position,rw::math::RPY<> orientation, rw::math::Vector3D<> releasePos, double maxOffset);
    Path moveRobotJRelease(rw::math::Q joint, rw::math::Vector3D<> releasePos, double maxOffset);


    double getSpeed() const;
    void setSpeed(double getSpeed);

    double getAcc() const;
    void setAcc(double getAcc);

    unsigned int getMsInterval() const;
    void setMsInterval(unsigned int getMsInterval);

private:
    unsigned int mMsInterval;
    ur_rtde::RTDEControlInterface *mControl = nullptr;
    ur_rtde::RTDEReceiveInterface *mReceive = nullptr;
    Gripper *mGripper = nullptr;
    double mSpeed, mAcc;

};
#endif // ROBOTMOVE_H
