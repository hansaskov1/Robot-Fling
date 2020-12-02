#include "robotmove.h"


RobotMove::RobotMove(unsigned int msInterval, Gripper* gripper ,ur_rtde::RTDEControlInterface* rtdeControl, ur_rtde::RTDEReceiveInterface *rtdeRecieve, double speed, double acceleration)
{
    mMsInterval = msInterval;
    mGripper = gripper;
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

void RobotMove::fetchPathLRelease(std::promise<Path> &&returnPath, std::atomic<bool> &stop, rw::math::Vector3D<> releasePos, double maxOffset)
{
    bool hasThrown = false;
    Path path;
    auto start = std::chrono::system_clock::now();
    while(!stop)
    {
        const std::vector<double> toolP = mReceive->getActualQ();
        const std::vector<double> toolV = mReceive->getActualQd();
        const std::vector<double> tcpP = mReceive->getActualTCPPose();
        const std::vector<double> tcpV = mReceive->getActualTCPSpeed();

        path.addJointPose(toolP);
        path.addJointVel(toolV);
        path.addToolPose(tcpP);
        path.addToolVel(tcpV);
        std::this_thread::sleep_for(std::chrono::milliseconds(mMsInterval));

        auto stop = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedTime = stop-start;
        path.addElapsedTime(elapsedTime.count());

       rw::math::Vector3D<> estimatedPos;
       estimatedPos[0] = (mMsInterval/1000) * tcpV[0] + tcpP[0];
       estimatedPos[1] = (mMsInterval/1000) * tcpV[1] + tcpP[1];
       estimatedPos[2] = (mMsInterval/1000) * tcpV[2] + tcpP[2];
       rw::math::Vector3D<> vecDiff;
       vecDiff[0] = abs(estimatedPos[0] - releasePos[0]);
       vecDiff[1] = abs(estimatedPos[1] - releasePos[1]);
       vecDiff[2] = abs(estimatedPos[2] - releasePos[2]);

       std::cout << "Diffrence: " << vecDiff << "Estimated: " << estimatedPos << "Diffrence from " << releasePos << std::endl;

       if (vecDiff[0] < maxOffset && vecDiff[1] < maxOffset && vecDiff[2] < maxOffset && !hasThrown)
       {
           mGripper->open();
           hasThrown = true;
       }
    }
    returnPath.set_value(std::move(path));
}

void RobotMove::fetchPathJRelease(std::promise<Path> &&returnPath, std::atomic<bool> &stop, rw::math::Q releasePose, double maxOffset)
{
    bool hasThrown = false;
    Path path;
    auto start = std::chrono::system_clock::now();
    while(!stop)
    {
        const std::vector<double> toolP = mReceive->getActualQ();
        const std::vector<double> toolV = mReceive->getActualQd();
        const std::vector<double> tcpP = mReceive->getActualTCPPose();
        const std::vector<double> tcpV = mReceive->getActualTCPSpeed();

        path.addJointPose(toolP);
        path.addJointVel(toolV);
        path.addToolPose(tcpP);
        path.addToolVel(tcpV);
        std::this_thread::sleep_for(std::chrono::milliseconds(mMsInterval));

        auto stop = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedTime = stop-start;
        path.addElapsedTime(elapsedTime.count());


       rw::math::Q estimatedJoint = releasePose;
       estimatedJoint[1] = (mMsInterval/1000) * toolV[1] + toolP[1];

       rw::math::Q jointDiff(0,0,0,0,0,0);
       jointDiff[1] = abs(estimatedJoint[1] - releasePose[1]);

       std::cout << "Diffrence: " << jointDiff << "Estimated: " << estimatedJoint << "Diffrence from " << releasePose << std::endl;

       if (jointDiff[1] < maxOffset && !hasThrown)
       {
           mGripper->open();
           hasThrown = true;
       }

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

Path RobotMove::moveRobotL(rw::math::Q jointPos)
{
    std::vector<double> JointstdVec = jointPos.toStdVector();
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPath, this , std::move(promisePath), std::ref(stop));
    mControl->moveL_FK(JointstdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}

Path RobotMove::moveRobotJ(rw::math::Q jointPos)
{
    std::vector<double> JointstdVec = jointPos.toStdVector();
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPath, this , std::move(promisePath), std::ref(stop));
    mControl->moveJ(JointstdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}

Path RobotMove::moveRobotJ(rw::math::Vector3D<> position ,rw::math::RPY<> orientation)
{
    std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPath, this , std::move(promisePath), std::ref(stop));
    mControl->moveJ_IK(toolPositionStdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}

Path RobotMove::moveRobotLRelease(rw::math::Vector3D<> position, rw::math::RPY<> orientation, rw::math::Vector3D<> releasePos, double maxOffset)
{
    std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPathLRelease, this , std::move(promisePath), std::ref(stop), releasePos, maxOffset);
    mControl->moveL(toolPositionStdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}

Path RobotMove::moveRobotJRelease(rw::math::Q jointPos, rw::math::Q releasePos, double maxOffset)
{
    std::vector<double> JointstdVec = jointPos.toStdVector();
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotMove::fetchPathJRelease, this , std::move(promisePath), std::ref(stop), releasePos, maxOffset);
    mControl->moveJ(JointstdVec, mSpeed, mAcc);
    stop = true;
    recive.join();
    return futurePath.get();
}

double RobotMove::getSpeed() const{return mSpeed;}
void RobotMove::setSpeed(double speed){mSpeed = speed;}

double RobotMove::getAcc() const{ return mAcc;}
void RobotMove::setAcc(double acc){mAcc = acc;}

unsigned int RobotMove::getMsInterval() const{ return mMsInterval;}
void RobotMove::setMsInterval(unsigned int msInterval){mMsInterval = msInterval;}

