#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/AmbiguityResolver.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/InvKinSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

#include <stdio.h>
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <chrono>
#include <thread>
#include <future>

#include "path.h"
//#include "DetectCollision.h"


class RobotControl {

public:

    RobotControl(std::string ipAdress)
    {
        mIpAdress = ipAdress;
    }


    RobotControl(std::string ipAdress, rw::math::Vector3D<> calPos, rw::math::Rotation3D<> calRot)
    {
        mIpAdress = ipAdress;
        mCalPos = calPos;
        mCalRot = calRot;
        mInvCalRot = calRot.inverse();
    }

    std::vector<double> TObj2TVec(rw::math::Transform3D<> TObject)
    {
        std::vector<double> TVector(6);
        rw::math::Vector3D<> P = TObject.P();
        TVector[0] = P[0];
        TVector[1] = P[1];
        TVector[2] = P[2];

        rw::math::Rotation3D<> R = TObject.R();
        rw::math::RPY<> rpy(R);

        TVector[3] = rpy[0];
        TVector[4] = rpy[1];
        TVector[5] = rpy[2];

        return TVector;
    }

    std::vector<double> vecRPY2stdVec(rw::math::Vector3D<> position, rw::math::RPY<> RPY)
    {
        double x = position[0];
        double y = position[1];
        double z = position[2];
        double R = RPY[0];
        double P = RPY[1];
        double Y = RPY[2];

        return std::vector<double>{x,y,z,R,P,Y};
    }

    rw::math::Vector3D<> world2Robot(rw::math::Vector3D<> worldPosition)
    {
        return mInvCalRot * worldPosition - mInvCalRot * mCalPos;
    }


   static void moveJ(std::vector<double> toolPosition, ur_rtde::RTDEControlInterface& rtde_control, double speed, double acceleration) // Used in own thread
   {
        rtde_control.moveJ(toolPosition,speed,acceleration);
   }

   static void moveL(std::vector<double> toolPosition, ur_rtde::RTDEControlInterface& rtde_control, double speed, double acceleration) // Used in own thread
   {
       rtde_control.moveL(toolPosition,speed,acceleration);
   }

   void fetchQValuesRob(std::promise<Path> && returnPath ,std::atomic<bool>& stop , ur_rtde::RTDEReceiveInterface &rtde_recieve, unsigned int msInterval) //Used in thread
   {
       Path path;
       while(!stop)
       {
           path.addJointPose(rtde_recieve.getActualQ());
           path.addJointVel(rtde_recieve.getActualQd());
           path.addToolPose(rtde_recieve.getActualTCPPose());
           path.addToolVel(rtde_recieve.getActualTCPSpeed());
           std::this_thread::sleep_for(std::chrono::milliseconds(msInterval));
       }
       returnPath.set_value(path);
   }



    Path moveRobotL( rw::math::Vector3D<> position,rw::math::RPY<> orientation, unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface &rtdeRecieve, double speed, double acceleration)
    {
        std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
        std::atomic<bool> stop {false};
        std::promise<Path> promiseQVec;
        auto futureQVec = promiseQVec.get_future();

        std::thread recive(&RobotControl::fetchQValuesRob, this , std::move(promiseQVec), std::ref(stop), std::ref(rtdeRecieve), msInterval);
        std::thread control(RobotControl::moveL, toolPositionStdVec, std::ref(rtdeControl), speed, acceleration);

        control.join();
        stop = true;
        recive.join();
        return futureQVec.get();
    }

    Path moveRobotJ( rw::math::Q jointPose, unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface &rtdeRecieve, double speed, double acceleration){ // For Linear movement in Tool Space

        std::vector<double> toolPositionStdVec = jointPose.toStdVector();
        std::atomic<bool> stop {false};
        std::promise<Path> promiseQVec;
        auto futureQVec = promiseQVec.get_future();

        std::thread recive(&RobotControl::fetchQValuesRob, this , std::move(promiseQVec), std::ref(stop), std::ref(rtdeRecieve), msInterval);
        std::thread control(RobotControl::moveJ, toolPositionStdVec, std::ref(rtdeControl), speed, acceleration);

        control.join();
        stop = true;
        recive.join();
        return futureQVec.get();
    }



    // All movements to get ball
    void getBall(rw::math::Vector3D<> posBallW, double safeGribHeight)
    {

        rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0);                    // Hardcoded home for our robot
        rw::math::Q qSafeGrib(-1.151, -2.202, -0.935, -1.574, 1.571, -0.003);   // Hardcoded safe gripping position

        rw::math::RPY<> rpyBall(0.6, -3.09, 0);                                 // Hardcoded safe orientation
        rw::math::RPY<> rpyGribReady = rpyBall;

        rw::math::Vector3D<> posGribReadyW = posBallW;
        posGribReadyW[2] += safeGribHeight;

        rw::math::Vector3D<> posBallR = world2Robot(posBallW);
        rw::math::Vector3D<> posGribReadyR = world2Robot(posGribReadyW);

       std::vector<std::vector<std::vector<double>>> QFullPath;
       std::cout << "Starting sim" << std::endl;

       {
           double simSpeed = 3;
           double simAcc = 3;
           double msInterval = 10;

           ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
           ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");
           QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
       }


       std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";
       DetectCollision dc(path);
       std::vector<bool> collisionList;

        for (std::vector<std::vector<double>> &qPath : QFullPath )
        {
            for (std::vector<double> &qValues : qPath){
               qValues[0] += 1.151;
              /*  std::cout << "{";
                for (double joint : qValues){
                    std::cout << joint << " ";
                }
                std::cout << "}"<< std::endl;*/
            }
            collisionList.push_back(dc.isCollision(qPath));
        }

        bool collision;
        for (bool isColl : collisionList)
        {
            (isColl)? std::cout << "true" : std::cout << "false";
            std::cout << std::endl;

            if (isColl) collision = true;
        }

    if (!collision)
    {
        double simSpeed = 1;
        double simAcc = 0.2;
        double msInterval = 10;

        ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
        ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);
        QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
        QFullPath.push_back(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
        QFullPath.push_back(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
        QFullPath.push_back(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, 0.2,      0.05  ).getJointPoses());
        QFullPath.push_back(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
        QFullPath.push_back(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
        QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());

    } else
    {
        std::cout << "a collision has occured" << std::endl;
    }
}

    void getBall(rw::math::Vector3D<> ballPositionW)
    {
      double distance = 0.2;    //Hardcoded a distance of 20cm above ball
      getBall(ballPositionW,distance);
    }

private:
std::string mIpAdress;
 rw::math::Vector3D<> mCalPos;
 rw::math::Rotation3D<> mCalRot;
 rw::math::Rotation3D<> mInvCalRot;
};

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



