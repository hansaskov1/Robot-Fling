#ifndef ROBOTMOVE_H
#define ROBOTMOVE_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "rw/math.hpp"
#include "path.h"
#include <thread>
#include <future>


class RobotMove
{
public:
    RobotMove();
    RobotMove(unsigned int & msInterval, ur_rtde::RTDEControlInterface * rtdeControl, ur_rtde::RTDEReceiveInterface * rtdeRecieve, double &speed, double &acceleration);

    std::vector<double> vecRPY2stdVec(rw::math::Vector3D<> position, rw::math::RPY<> RPY);

    void fetchPath(std::promise<Path> && returnPath ,std::atomic<bool>& stop);
    Path moveRobotL(rw::math::Vector3D<> position ,rw::math::RPY<> orientation);
    Path moveRobotL(rw::math::Q);
    Path moveRobotJ(rw::math::Q);
    Path moveRobotJ(rw::math::Vector3D<> position ,rw::math::RPY<> orientation);

private:
    unsigned int mMsInterval;
    ur_rtde::RTDEControlInterface *mControl = nullptr;
    ur_rtde::RTDEReceiveInterface *mReceive = nullptr;
    double mSpeed, mAcc;

};
#endif // ROBOTMOVE_H
