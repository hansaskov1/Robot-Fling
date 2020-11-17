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
#include "Gripper.h"
#include "path.h"
#include "DetectCollision.h"
#include "Throw.h"



class RobotControl {

public:

    RobotControl() {
        gripper.Init();
    }

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

    void setParam(std::string ipAdress, QString gripIpAdress, rw::math::Vector3D<> calPos, rw::math::Rotation3D<> calRot) {
        mIpAdress = ipAdress;
        mCalPos = calPos;
        mCalRot = calRot;
        mInvCalRot = calRot.inverse();
        gripper.ToConnectToHost(gripIpAdress, 1000);
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

    rw::math::Vector3D<> robot2World(rw::math::Vector3D<> robotPosition)
    {
        return mCalRot * robotPosition + mCalPos;
    }

   void fetchPath(std::promise<Path> && returnPath ,std::atomic<bool>& stop , ur_rtde::RTDEReceiveInterface &rtde_recieve, unsigned int msInterval) //Used in thread
   {

       Path path;
       auto start = std::chrono::system_clock::now();

       while(!stop)
       {
           path.addJointPose(rtde_recieve.getActualQ());
           path.addJointVel(rtde_recieve.getActualQd());
           path.addToolPose(rtde_recieve.getActualTCPPose());
           path.addToolVel(rtde_recieve.getActualTCPSpeed());
           std::this_thread::sleep_for(std::chrono::milliseconds(msInterval));

           auto stop = std::chrono::system_clock::now();
           std::chrono::duration<double> elapsedTime = stop-start;

           path.addElapsedTime(elapsedTime.count());

       }
       returnPath.set_value(std::move(path));
   }


Path moveRobotL( rw::math::Vector3D<> position,rw::math::RPY<> orientation, unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface& rtdeRecieve, double speed, double acceleration)
{
    std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
    std::atomic<bool> stop {false};
    std::promise<Path> promisePath;
    std::future<Path> futurePath = promisePath.get_future();

    std::thread recive(&RobotControl::fetchPath, this , std::move(promisePath), std::ref(stop), std::ref(rtdeRecieve), msInterval);
    rtdeControl.moveL(toolPositionStdVec, speed, acceleration);
    stop = true;
    recive.join();
    return futurePath.get();
}

    Path moveRobotL( rw::math::Q jointPose , unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface &rtdeRecieve, double speed, double acceleration)
    {
        std::vector<double> toolPositionStdVec = jointPose.toStdVector();
        std::atomic<bool> stop {false};
        std::promise<Path> promisPath;
        std::future<Path> futurePath = promisPath.get_future();

        std::thread recive(&RobotControl::fetchPath, this , std::move(promisPath), std::ref(stop), std::ref(rtdeRecieve), msInterval);
        rtdeControl.moveL_FK(toolPositionStdVec, speed, acceleration);
        stop = true;
        recive.join();
        return futurePath.get();
    }

    Path moveRobotJ( rw::math::Q jointPose, unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface &rtdeRecieve, double speed, double acceleration){ // For Linear movement in Tool Space

        std::vector<double> toolPositionStdVec = jointPose.toStdVector();
        std::atomic<bool> stop {false};
        std::promise<Path> promisePath;
        std::future<Path> futurePath = promisePath.get_future();

        std::thread recive(&RobotControl::fetchPath, this , std::move(promisePath), std::ref(stop), std::ref(rtdeRecieve), msInterval);
        rtdeControl.moveJ(toolPositionStdVec, speed, acceleration);
        stop = true;
        recive.join();
        return futurePath.get();
    }


   /* void throwBallToTarget(rw::math::Vector3D<> targetPosition, ){

    }*/

    // All movements to get ball
    void getBall(rw::math::Vector3D<> posBallW, double safeGribHeight)
    {

        rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0);                    // Hardcoded home for our robot
        rw::math::Q qSafeGrib(-1.151, -2.202, -0.935, -1.574, 1.571, -0.003);   // Hardcoded safe gripping position
        rw::math::Q qBallReady(-1.256,-0.175,-2.591,-1.518,1.57,0.0175);        // Hardcode start acc ball throw
        rw::math::Q qBallRelease(-1.256,-1.588,-0.7286,-1.518,1.57,0.0175);     // Hardcode end ball throw
        std::cout << qBallRelease << std::endl;

        rw::math::RPY<> rpyBall(0.6, -3.09, 0);                                 // Hardcoded safe orientation
        rw::math::RPY<> rpyGribReady = rpyBall;

        rw::math::RPY<> rpyRelease(0.455,-2.941,1.359);
        rw::math::Vector3D<> positionRelease (60/1000 , -554/1000 , 697/1000);

        rw::math::Vector3D<> posGribReadyW = posBallW;
        posGribReadyW[2] += safeGribHeight;

        rw::math::Vector3D<> posBallR = world2Robot(posBallW);
        rw::math::Vector3D<> posGribReadyR = world2Robot(posGribReadyW);

        std::vector<std::vector<std::vector<double>>> QFullPath;

        std::string path = "../Scenes/XMLScenes/RobotOnTable/Scene.xml";
        DetectCollision dc(path);

        std::cout << "RobWork Collision check. Not yet tested" << std::endl;
        std::cout << dc.isCollision(50, qHome) << std::endl;                                                      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qSafeGrib) << std::endl;                                                  //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posGribReadyR, rpyGribReady)) << std::endl;      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posBallR, rpyBall)) << std::endl;                //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posGribReadyR, rpyGribReady)) << std::endl;      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qSafeGrib) << std::endl;                                                  //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qHome) << std::endl;                                                      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << std::endl;

       {
           double simSpeed = 3;
           double simAcc = 3;
           double msInterval = 10;

           std::cout << "running sim" <<std::endl;
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

        std::vector<bool> collisionList;

        for (std::vector<std::vector<double>> &qPath : QFullPath )
        {
            std::cout << "New Path" << std::endl;
            for (std::vector<double> &qValues : qPath)
            {
                std::cout << "{";
                for (double joint : qValues)
                {
                    std::cout << joint << " ";
                }
                std::cout << "}"<< std::endl;
            }
            collisionList.push_back(dc.isCollision(qPath));
        }

        bool collision = false;
        for (bool isColl : collisionList)
        {
            (isColl)? std::cout << "true" : std::cout << "false";
            std::cout << std::endl;

            if (isColl) collision = true;
        }


        if (!collision)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::cout << "running robot" << std::endl;
            double speed = 0.5;
            double acceleration = 1;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
            ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, 0.2,      0.05  ));
            gripper.close();
            //while (!gripper.hasGripped());
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            gripper.open();

        } else
        {
            std::cout << "a collision will occur" << std::endl;
            std::cout << "exiting without moving robot" << std::endl;
        }
    }

    void getBall(rw::math::Vector3D<> ballPositionW)
    {
      double distance = 0.2;    //Hardcoded a distance of 20cm above ball
      getBall(ballPositionW,distance);
    }

    Throw getThrow() {
        mThrow.setThrowID(1);
        mThrow.setObject("Bold");
        mThrow.setAngle(45);
        mThrow.setSpeed(200);
        mThrow.setSuccess(1);
        return mThrow;
    }

    double speed(double vinkel, rw::math::Vector3D<> throwPose, rw::math::Vector3D<> cupPose)
    {
        double xDistanceToCup = throwPose[0] - cupPose[0];
        double yDistanceToCup = throwPose[1] - cupPose[1];
        double zDistanceToCup = throwPose[2] - cupPose[2];

        double den = 9.82*(pow(xDistanceToCup,2)+pow(yDistanceToCup,2));
        double num = 2 * cos(pow(vinkel,2)) * tan(vinkel) * (sqrt(pow(xDistanceToCup,2)+pow(yDistanceToCup,2)))+zDistanceToCup;

        double speed = sqrt(den/num);
        return speed;
    }

    //8*18=40*90
    //82.5+190
    //94.75
    //
    //392
    //680.5mm

    //BL = R*Vinkel
    void rotateThrow(double angleThrow, double speed, double acceleration, rw::math::Vector3D<double> cupPos){
        rw::math::Vector3D<double> robBase(40,90,0);
        rw::math::Vector3D<double> cupPosR = cupPos - robBase;
        double angleBase = atan(cupPosR.y()/cupPosR.x());

        double timeFromThrowPose = speed/acceleration;
        double archLenth = 0.5 * acceleration * (timeFromThrowPose * timeFromThrowPose);



    }

    rw::math::Vector3D<double> rampPose(double hastighed, double acceleration, double vinkel, rw::math::Vector3D<double> throwPose, rw::math::Vector3D<double> &stopPose)
    {
        double timeFromThrowPose = hastighed/acceleration;
        double lenghtToRampPose = 0.5 * acceleration * pow(timeFromThrowPose,2);

        rw::math::Vector3D<double> startRampPose;

        double xLenght = lenghtToRampPose * cos(vinkel);
        double yLenght = 0;
        double zLenght = lenghtToRampPose * sin(vinkel);

        startRampPose = rw::math::Vector3D<double>(xLenght, yLenght, zLenght);

        rw::math::Vector3D<double> startPose = throwPose - startRampPose;
        double safety = 1;
        stopPose = (throwPose + startRampPose) * safety;

        return startPose;
    }



    void throwBall(double safeGribHeight)
        {

        std::cout << "Trying to throw ball" << std::endl;
            rw::math::Q qReleaseBall((-67.88*3.1415/180) , (-41.79*3.1415/180) , (-108*3.1415/180) , (-56.88*3.1415/180) , (90.15*3.1415/180) , (0.09*3.1415/180)); // Joint pos
            rw::math::Vector3D<double> releaseBallPosition = rw::math::Vector3D<double>(12.99,-327.40,532.99);
            rw::math::RPY<> rpyRelease(0.547,-2.802,-1.729);
            rw::math::Vector3D<double> cupPosition = rw::math::Vector3D<double>(10, 10, 0);
            rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0);                    // Hardcoded home for our robot


            double throwSpeed = speed(3.1415/2, releaseBallPosition, cupPosition);

            rw::math::Vector3D<double> endPos;
            rw::math::Vector3D<double> startPos = rampPose(throwSpeed, 3, 3.1415/2, releaseBallPosition, endPos);

            rw::math::RPY<> rpyBall(0.6, -3.09, 0);                                 // Hardcoded safe orientation
            rw::math::RPY<> rpyGribReady = rpyBall;

            //rw::math::Vector3D<> posGribReadyW = posBallW;
            //posGribReadyW[2] += safeGribHeight;

            //rw::math::Vector3D<> posBallR = world2Robot(posBallW);
            //rw::math::Vector3D<> posGribReadyR = world2Robot(posGribReadyW);

            rw::math::Vector3D<> robStartPos = world2Robot(startPos);


           std::vector<std::vector<std::vector<double>>> QFullPath;
           std::cout << "Starting sim" << std::endl;

           {
               double simSpeed = 3;
               double simAcc = 3;
               double msInterval = 10;

               ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
               ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");
                std::cout << "Release ball pos : " << releaseBallPosition << std::endl;
                std::cout << "Robstart : " << robStartPos << std::endl;
                std::cout << "RpyRelease : " << rpyRelease << std::endl;

               QFullPath.push_back(moveRobotJ(qReleaseBall,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
               QFullPath.push_back(moveRobotL(robStartPos,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());

               QFullPath.push_back(moveRobotL(endPos,rpyBall,                        msInterval, rtdeControl, rtdeRecive, throwSpeed, simAcc).getJointPoses());




             //  QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           }


           std::string path = "../Scenes/XMLScenes/RobotOnTable/Scene.xml";
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

            bool collision = false;
            for (bool isColl : collisionList)
            {
                (isColl)? std::cout << "true" : std::cout << "false";
                std::cout << std::endl;

                if (isColl) collision = true;
            }

            if (!collision){
                double simSpeed = 3;
                double simAcc = 3;
                double msInterval = 10;

                ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
                ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);

                mThrow.addPath(moveRobotJ(qReleaseBall,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
                mThrow.addPath(moveRobotL(startPos,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
                mThrow.addPath(moveRobotL(endPos,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, throwSpeed, simAcc));

//                mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, 0.2,      0.05  ));
//                gripper.close();
//                //while (!gripper.hasGripped());
//                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//                mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                gripper.open();

            } else
            {
                std::cout << "a collision has occured" << std::endl;
            }
        }

private:
 std::string mIpAdress;
 Gripper gripper;
 rw::math::Vector3D<> mCalPos;
 rw::math::Rotation3D<> mCalRot;
 rw::math::Rotation3D<> mInvCalRot;
 Throw mThrow;
};





