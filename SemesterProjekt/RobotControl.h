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
#include "math.h"
#include "cmath"



class RobotControl {

public:

    RobotControl()
    {
        init();
    }


    RobotControl(std::string ipAdress, rw::math::Vector3D<> calPos, rw::math::Rotation3D<> calRot)
    {
        mIpAdress = ipAdress;
        mCalPos = calPos;
        mCalRot = calRot;
        mInvCalRot = calRot.inverse();
        init();
    }

    void setParam(std::string ipAdress, QString gripIpAdress, rw::math::Vector3D<> calPos, rw::math::Rotation3D<> calRot) {
        mIpAdress = ipAdress;
        mCalPos = calPos;
        mCalRot = calRot;
        mInvCalRot = calRot.inverse();
        gripper.ToConnectToHost(gripIpAdress, 1000);
    }

    void init(){
        gripper.Init();
        scenePath[0] = "../Scenes/XMLScenes/RobotOnTable/Scene.xml";
        scenePath[1] = "../Scenes/XMLScenesCelle1/RobotOnTable/Scene.xml";
        scenePath[2] = "../Scenes/XMLScenesCelle2/RobotOnTable/Scene.xml";
        scenePath[3] = "../Scenes/XMLScenesCelle3/RobotOnTable/Scene.xml";
        scenePath[4] = "../Scenes/XMLScenesCelle4/RobotOnTable/Scene.xml";
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

    Path moveRobotJ( rw::math::Vector3D<> position,rw::math::RPY<> orientation, unsigned int msInterval, ur_rtde::RTDEControlInterface& rtdeControl, ur_rtde::RTDEReceiveInterface& rtdeRecieve, double speed, double acceleration)
    {
        std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
        std::atomic<bool> stop {false};
        std::promise<Path> promisePath;
        std::future<Path> futurePath = promisePath.get_future();

        std::thread recive(&RobotControl::fetchPath, this , std::move(promisePath), std::ref(stop), std::ref(rtdeRecieve), msInterval);
        rtdeControl.moveJ_IK(toolPositionStdVec, speed, acceleration);
        stop = true;
        recive.join();
        return futurePath.get();
    }


   /* void throwBallToTarget(rw::math::Vector3D<> targetPosition, ){

    }*/

    // All movements to get ball
    void getBall(rw::math::Vector3D<> posBallW, rw::math::RPY<> gripOrientation, double safeGribHeight)
    {

        rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0);                    // Hardcoded home for our robot
        rw::math::Q qSafeGrib(-1.151, -2.202, -0.935, -1.574, 1.571, -0.003);   // Hardcoded safe gripping position

        rw::math::RPY<> rpyBall = gripOrientation;                  //rw::math::RPY<> rpyBall(0.6, -3.09, 0)  // Hardcoded safe orientation
        rw::math::RPY<> rpyGribReady = rpyBall;

        rw::math::Vector3D<> posGribReadyW = posBallW;
        posGribReadyW[2] += safeGribHeight;

        rw::math::Vector3D<> posBallR = world2Robot(posBallW);
        rw::math::Vector3D<> posGribReadyR = world2Robot(posGribReadyW);


        DetectCollision dc(scenePath[cellNr]);
        /*
        std::cout << "RobWork Collision check" << std::endl;
        std::cout << dc.isCollision(50, qHome) << std::endl;                                                      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qSafeGrib) << std::endl;                                                  //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posGribReadyR, rpyGribReady)) << std::endl;      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posBallR, rpyBall)) << std::endl;                //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollisionUR(50,rw::math::Transform3D<>(posGribReadyR, rpyGribReady)) << std::endl;      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qSafeGrib) << std::endl;                                                  //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << dc.isCollision(50, qHome) << std::endl;                                                      //for (rw::math::Q &qValues : dc.getQVec()){ std::cout << qValues << std::endl;}
        std::cout << std::endl;
        */
        bool isNormalMode;
       {
           double simSpeed = 3;
           double simAcc = 3;
           double msInterval = 10;

           std::cout << "running sim" <<std::endl;
           dc.setHasCollided(false);
           ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
           ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");

          rw::math::Q startPos = rw::math::Q(rtdeRecive.getActualQ());

           dc.isCollision(moveRobotJ(startPos,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotL(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           isNormalMode = rtdeRecive.getSafetyMode() == 1;
           std::cout << isNormalMode << std::endl;

       }


        if (!dc.getHasCollided() && isNormalMode)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::cout << "running robot" << std::endl;
            double speed = 0.2;
            double acceleration = 0.05;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
            ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotJ(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotL(posBallR, rpyBall,            msInterval, rtdeControl, rtdeRecive, 0.02,      0.01));
            gripper.close();
            //while (!gripper.hasGripped());
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            mThrow.addPath(moveRobotL(posGribReadyR, rpyGribReady,  msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotL(qSafeGrib,                    msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, speed, acceleration));
            gripper.open();

    } else
        {
        std::cout << "a collision or an invalid pose will occur" << std::endl;
        std::cout << "returning without moving robot" << std::endl;
        }
    }

    void getBall(rw::math::Vector3D<> ballPositionW, rw::math::RPY<> gripOrientation)
    {
      double distance = 0.2;    //Hardcoded a distance of 20cm above ball
      return getBall(ballPositionW, gripOrientation,distance);
    }

    void getBall(rw::math::Vector3D<> ballPositionW)
    {
      double distance = 0.2;    //Hardcoded a distance of 20cm above ball
      return getBall(ballPositionW,distance);
    }

    void getBall(rw::math::Vector3D<> ballPositionW, double distance)
    {
      rw::math::RPY<> rpyBall;

      switch (cellNr)
      {
      case 2:
        rpyBall = rw::math::RPY<> (0.6, -3.09, 0);          //Celle 2
      break;
      case 4:
        rpyBall = rw::math::RPY<> (2.6, -1.69, 0);          //Celle 4 //x += 2, y-= 2
      break;
      default:
        std::cout << "Non valid cellNR in RobotControl -> getBall" << std::endl;
      return;
      }

     return getBall(ballPositionW, rpyBall ,distance);
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
        double num = 2 * cos(pow(vinkel,2)) * tan(vinkel) * (sqrt(pow(xDistanceToCup,2)+pow(yDistanceToCup,2))) - zDistanceToCup;

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
    /*
    void rotateThrow(double angleThrow, double speed, double acceleration, rw::math::Vector3D<double> cupPos){
        rw::math::Vector3D<double> robBase(40,90,0);
        rw::math::Vector3D<double> cupPw::math::RPY<> gripOrientationosR = cupPos - robBase;
        double angleBase = atan(cupPosR.y()/cupPosR.x());

        double timeFromThrowPose = speed/acceleration;
        double archLenth = 0.5 * acceleration * (timeFromThrowPose * timeFromThrowPose);



    }

    */
    rw::math::Vector3D<double> rampPose(double hastighed, double acceleration, double vinkel, rw::math::Vector3D<double> throwPose, rw::math::Vector3D<double> &stopPose)
    {
        double timeFromThrowPose = hastighed/acceleration;
        double lenghtToRampPose = 0.5 * acceleration * pow(timeFromThrowPose,2);

        rw::math::Vector3D<double> startRampPose;

        double yLenght = lenghtToRampPose * cos(vinkel);
        double xLenght = 0;
        double zLenght = lenghtToRampPose * sin(vinkel);

        startRampPose = rw::math::Vector3D<double>(xLenght, yLenght, zLenght);

        std::cout << startRampPose << std::endl;

        rw::math::Vector3D<double> startPose = throwPose - startRampPose;
        double safety = 1;
        stopPose = (throwPose + startRampPose) * safety;

        std::cout << startPose << std::endl;
        std::cout << stopPose << std::endl;

        return startPose;
    }

    double rotateBaseToCupAngle(rw::math::Vector3D<> cupPosition, rw::math::Vector3D<> robotBasePosition){
        // robot base = x = 40 , y = 90
        rw::math::Vector3D<> lengthFromCupToBase = robotBasePosition-cupPosition;
        double cLength = std::sqrt((lengthFromCupToBase[0] * lengthFromCupToBase[0]) + (lengthFromCupToBase[1] * lengthFromCupToBase[1]));
        double aLength = std::abs(cupPosition[0]-robotBasePosition[0]);
        double angleA = asin(aLength/cLength);
        return angleA;
    }


    rw::math::Rotation3D<> Rx(double angle)
    {
     return rw::math::Rotation3D<>(
                                    1, 0, 0,
                                    0, std::cos(angle), -std::sin(angle),
                                    0, std::sin(angle),  std::cos(angle)
                                   );
    }

    rw::math::Rotation3D<> Ry(double angle)
    {
     return rw::math::Rotation3D<>(
                                    std::cos(angle), 0 ,  std::sin(angle),
                                     0, 1, 0,
                                    -std::sin(angle),0 , std::cos(angle)
                                   );
    }

    rw::math::Rotation3D<> Rz(double angle)
    {
     return rw::math::Rotation3D<>(
                                    std::cos(angle), -std::sin(angle), 0,
                                    std::sin(angle),  std::cos(angle), 0,
                                    0, 0, 1
                                   );
    }





    void throwBallLinear(rw::math::Vector3D<> cupPos, rw::math::Vector3D<> releasePos, double angle, double lenghtOffset = 0.3)
    {
        rw::math::Vector3D<> posDiff = releasePos - cupPos;
        posDiff[2] = 0;
        rw::math::Vector3D<> eigenVec = posDiff.normalize();  //  std::cout << "1. " << eigenVec << std::endl;

        if (posDiff[1] < 0){

            if (posDiff[0] < 0){
              double offsetAngle = std::acos(std::abs(eigenVec[0]));
              eigenVec = Rz(offsetAngle).inverse()  * eigenVec;   // std::cout << "2. " << eigenVec << std::endl;
              eigenVec = Ry(angle).inverse()        * eigenVec;   // std::cout << "3. " << eigenVec << std::endl;
              eigenVec = Rz(offsetAngle)            * eigenVec;   // std::cout << "4. "<< eigenVec << std::endl;
            } else if (posDiff[0] >= 0){
              double offsetAngle = std::asin(std::abs(eigenVec[0]));
              eigenVec = Rz(offsetAngle).inverse()  * eigenVec;   // std::cout << "2. " << eigenVec << std::endl;
              eigenVec = Rx(-angle).inverse()       * eigenVec;   // std::cout << "3. " << eigenVec << std::endl;
              eigenVec = Rz(offsetAngle)            * eigenVec;   // std::cout << "4. "<< eigenVec << std::endl;
            }

        } else if (posDiff[1] >= 0){

            if (posDiff[0] < 0){
                double offsetAngle = std::asin(std::abs(eigenVec[0]));
                eigenVec = Rz(offsetAngle).inverse()  * eigenVec;   // std::cout << "2. " << eigenVec << std::endl;
                eigenVec = Rx(angle).inverse()        * eigenVec;   // std::cout << "3. " << eigenVec << std::endl;
                eigenVec = Rz(offsetAngle)            * eigenVec;   // std::cout << "4. "<< eigenVec << std::endl;
            } else if (posDiff[0] >= 0){
                double offsetAngle = std::acos(std::abs(eigenVec[0]));
                eigenVec = Rz(offsetAngle).inverse()  * eigenVec;   // std::cout << "2. " << eigenVec << std::endl;
                eigenVec = Ry(-angle).inverse()       * eigenVec;   // std::cout << "3. " << eigenVec << std::endl;
                eigenVec = Rz(offsetAngle)            * eigenVec;   // std::cout << "4. "<< eigenVec << std::endl;
            }

        }

        double lenght = -releasePos[2] / eigenVec[2];

        rw::math::Vector3D<> rampPosW = releasePos + (lenght - lenghtOffset) * eigenVec;  std::cout << rampPosW << std::endl;
        rw::math::Vector3D<> endPosW  = releasePos - (lenght - lenghtOffset) * eigenVec;  std::cout << endPosW << std::endl;
        const double pi = 3.1415;
        double testval = 1.151- pi;
        rw::math::Q qSafeGrib(testval, -2.202, -0.935, -1.574, 1.571, -0.003);

        rw::math::Vector3D<> rampPosR = world2Robot(rampPosW);                            std::cout << rampPosR << std::endl;
        rw::math::Vector3D<> endPosR = world2Robot(endPosW);                              std::cout << endPosR << std::endl;

        rw::math::RPY<> throwOrientation(2.6, -1.09, 0);

        DetectCollision dc(scenePath[cellNr]);

        Path throwPath;
        bool isNormalMode;

        {
            double simSpeed = 3;
            double simAcc = 3;
            double msInterval = 10;

           std::cout << "running sim" << std::endl;
           ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
           ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");
           dc.isCollision(moveRobotJ(qHome,                           msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
          // dc.isCollision(moveRobotJ(qSafeGrib,                           msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           dc.isCollision(moveRobotJ(rampPosR,   throwOrientation,    msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           throwPath = moveRobotL(endPosR,  throwOrientation,         msInterval, rtdeControl, rtdeRecive, 1, simAcc);
           dc.isCollision(throwPath.getJointPoses());
           dc.isCollision(moveRobotJ(qHome,                           msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
          isNormalMode = rtdeRecive.getSafetyMode() == 1;
        }

      //  std::cout << throwPath << std::endl;


        std::vector<double> pathSpeed;

        for (std::vector<double> toolSpeed : throwPath.getToolVel()){
            pathSpeed.push_back(abs(toolSpeed[0], toolSpeed[1], toolSpeed[2]));
        }

        for (double speed : pathSpeed){
           // std::cout <<  speed << std::endl;
        }

        if (!dc.getHasCollided() && isNormalMode)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::cout << "running robot" << std::endl;
            double moveSpeed = 0.5;
            double throwSpeed = speed(angle, cupPos, releasePos);
            double acceleration = 1;
            double throwAcc = 3;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
            ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, moveSpeed, acceleration));

            mThrow.addPath(moveRobotJ(rampPosR, throwOrientation,   msInterval, rtdeControl, rtdeRecive, moveSpeed, acceleration));
            mThrow.addPath(moveRobotL(endPosR, throwOrientation,    msInterval, rtdeControl, rtdeRecive, throwAcc, throwAcc));
            mThrow.addPath(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, moveSpeed, acceleration));
            gripper.open();

            std::cout << "calculated throw speed: " <<throwSpeed << std::endl;

    } else
        {
        std::cout << "a collision will occur" << std::endl;
        std::cout << "returning without moving robot" << std::endl;
        }
    }


    double abs(double val1, double val2, double val3)
    {
        return std::sqrt(val1* val1 + val2*val2 + val3*val3);
    }

    void throwBall(rw::math::Vector3D<> cupPosition,double safeGribHeight)
       // std::cout << rampPos << std::endl;
        {
        std::string path = "../Scenes/XMLScenes/RobotOnTable/Scene.xml";
        rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0);                    // Hardcoded home for our robot


        //Vi arbejder i Y Z
        DetectCollision coli(path);
        rw::math::Vector3D<>robotPosition (40,90,0);
        rw::math::Q qReleaseBall((-67.88*3.1415/180) , (-41.79*3.1415/180) , (-108*3.1415/180) , (-56.88*3.1415/180) , (90.15*3.1415/180) , (0.09*3.1415/180)); // Joint pos
        qReleaseBall[0]+=rotateBaseToCupAngle(cupPosition,robotPosition); // Q value, offset to ball
        coli.setState(qReleaseBall);
        rw::math::Transform3D<> tcpTrans = coli.getTransform();
        rw::math::Vector3D<> tcpPosRobot = tcpTrans.P();
        rw::math::Vector3D<> tcpPosWorld = robot2World(tcpPosRobot);


        double angle = 3.1415/4;
        double throwSpeed = speed(angle,tcpPosWorld,cupPosition);

        rw::math::Vector3D<> stopPosition;
        double acc = 3;

        rw::math::Vector3D<> startPosition = rampPose(throwSpeed,acc,angle,tcpPosWorld,stopPosition);
        rw::math::RPY<> rpyRelease(0.547,-2.802,-1.729);



 /*       std::cout << "Trying to throw ball" << std::endl;
            rw::math::Vector3D<double> releaseBallPositionRobot = rw::math::Vector3D<double>(0.1299,-0.32740,0.53299);

            rw::math::Vector3D<>releaseBallPosition = robot2World(releaseBallPositionRobot);

            rw::math::RPY<> rpyRelease(0.547,-2.802,-1.729);
            rw::math::Vector3D<double> cupPosition = rw::math::Vector3D<double>(0.10, 0.10, 0);


            double throwSpeed = speed(3.1415/4, releaseBallPosition, cupPosition);

            rw::math::Vector3D<double> endPos;
            rw::math::Vector3D<double> startPos = rampPose(throwSpeed, 3, 3.1415/4, releaseBallPosition, endPos);

            rw::math::RPY<> rpyBall(0.6, -3.09, 0);                                 // Hardcoded safe orientation
            rw::math::RPY<> rpyGribReady = rpyBall;

            rw::math::Vector3D<> posGribReadyW = posBallW;
            posGribReadyW[2] += safeGribHeight;

            rw::math::Vector3D<> posBallR = world2Robot(posBallW);
            rw::math::Vector3D<> posGribReadyR = world2Robot(posGribReadyW);

            rw::math::Vector3D<> robStartPos = world2Robot(startPos);
            rw::math::Vector3D<> robEndPos = world2Robot(endPos);
            */


           std::vector<std::vector<std::vector<double>>> QFullPath;
           std::cout << "Starting sim" << std::endl;

           {
               double simSpeed = 3;
               double simAcc = 3;
               double msInterval = 10;

               ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
               ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");
//                std::cout << "Release ball pos : " << releaseBallPosition << std::endl;
//                std::cout << "Robstart : " << robStartPos << std::endl;
//                std::cout << "RpyRelease : " << rpyRelease << std::endl;
//                std::cout << "RobEndPos : " << robEndPos << std::endl;

               QFullPath.push_back(moveRobotJ(qReleaseBall,              msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
               QFullPath.push_back(moveRobotL(startPosition,rpyRelease, msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());

               QFullPath.push_back(moveRobotL(stopPosition,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, throwSpeed, simAcc).getJointPoses());




               QFullPath.push_back(moveRobotJ(qHome,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc).getJointPoses());
           }


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

//                mThrow.addPath(moveRobotJ(qReleaseBall,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotL(startPos,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, simSpeed, simAcc));
//                mThrow.addPath(moveRobotL(endPos,rpyRelease,                        msInterval, rtdeControl, rtdeRecive, throwSpeed, simAcc));

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

    int getCellNr() const{return cellNr;}
    void setCellNr(int value){cellNr = value;}

private:
 std::string mIpAdress;
 Gripper gripper;
 rw::math::Vector3D<> mCalPos;
 rw::math::Rotation3D<> mCalRot;
 rw::math::Rotation3D<> mInvCalRot;
 Throw mThrow;
 const rw::math::Q qHome = rw::math::Q(-1.151,-3.1415/2,0,-3.1415/2,0,0);
 //const std::string  scenePath = "../Scenes/XMLScenes/RobotOnTable/Scene.xml";
 std::array<std::string, 5> scenePath;
 int cellNr = 4;
};


