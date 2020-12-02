#include "robotmove.h"


RobotMove::RobotMove(unsigned int &msInterval, ur_rtde::RTDEControlInterface* rtdeControl, ur_rtde::RTDEReceiveInterface *rtdeRecieve, double &speed, double &acceleration)
{
    mMsInterval = msInterval;
    mControl = rtdeControl;
    mReceive = rtdeRecieve;
    mSpeed = speed;
    mAcc = acceleration;
}


std::vector<double> RobotMove::vecRPY2stdVec(rw::math::Vector3D<> position, rw::math::RPY<> RPY)
{
    double x = position[0];
    double y = position[1];
    double z = position[2];
    double R = RPY[0];
    double P = RPY[1];
    double Y = RPY[2];

    return std::vector<double>{x,y,z,R,P,Y};
}

void RobotMove::fetchPath(std::promise<Path> && returnPath ,std::atomic<bool>& stop) //Used in thread
{
    Path path;
    auto start = std::chrono::system_clock::now();
    while(!stop)
    {
        path.addJointPose(mReceive->getActualQ());
        path.addJointVel(mReceive->getActualQd());
        path.addToolPose(mReceive->getActualTCPPose());
        path.addToolVel(mReceive->getActualTCPSpeed());
        std::this_thread::sleep_for(std::chrono::milliseconds(mMsInterval));

        auto stop = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedTime = stop-start;
        path.addElapsedTime(elapsedTime.count());
    }
    returnPath.set_value(std::move(path));
}

Path RobotMove::moveRobotL(rw::math::Vector3D<> position ,rw::math::RPY<> orientation)
{
    std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPath, this , std::move(promisePath), std::ref(stop));
    mControl->moveL(toolPositionStdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}
