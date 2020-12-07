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

#include "robotmove.h"


class RobotControl {

public:

    RobotControl()
    {
        gripper.Init();
    }

    void setParam(std::string ipAdress, QString gripIpAdress, int celleNr) {
        mIpAdress = ipAdress;
        gripper.ToConnectToHost(gripIpAdress, 1000);
        cellNr = celleNr;
        switch (celleNr)
        {
        case 2:
            // Robot cal for table 2
            mCalPos = rw::math::Vector3D<>(0.400624689065891, 0.901530744085863, 0.042187492976487);
            mCalRot = rw::math::Rotation3D<double>(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);
            scenePath = "../Scenes/XMLScenesCelle2/RobotOnTable/Scene.xml";
            break;
        case 4:
            // Robot cal for table 4
            mCalPos = rw::math::Vector3D<>(0.404933521031581,0.911568253889385,0.040065747515709);
            mCalRot = rw::math::Rotation3D<double>(0.927485860124202,0.373761533519894,-0.008502666083409,-0.373842123595955,0.927417138009057,-0.011811805634918,0.003470719656776,0.014133937453771,0.999894087349814);
            scenePath = "../Scenes/XMLScenesCelle4/RobotOnTable/Scene.xml";
            break;
        default:
            std::cerr << "Den er ikke lavet endnu" << std::endl;
        }
        mInvCalRot = rw::math::inverse(mCalRot);
    }

    void disconnect() {
        gripper.disconnect();
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


        DetectCollision dc(scenePath);

        bool isNormalMode;

        {
           double simSpeed = 3;
           double simAcc = 3;
           double msInterval = 10;

           std::cout << "running sim" <<std::endl;
           dc.setHasCollided(false);
           ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
           ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");
           ur_rtde::RTDEReceiveInterface RealRobotRecieve(mIpAdress);

           RobotMove Robot(msInterval, &gripper, &rtdeControl, &rtdeRecive, simSpeed, simAcc);

           rw::math::Q startPos = rw::math::Q(RealRobotRecieve.getActualQ());
           Robot.moveRobotJ(startPos).getJointPoses();
           dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
           dc.isCollision(Robot.moveRobotJ(qSafeGrib).getJointPoses());
           dc.isCollision(Robot.moveRobotL(posGribReadyR, rpyGribReady).getJointPoses());
           dc.isCollision(Robot.moveRobotL(posBallR, rpyBall).getJointPoses());
           dc.isCollision(Robot.moveRobotL(posGribReadyR, rpyGribReady ).getJointPoses());
           dc.isCollision(Robot.moveRobotL(qSafeGrib).getJointPoses());
           dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
           isNormalMode = rtdeRecive.getSafetyMode() == 1;
        }

        if (!dc.getHasCollided() && isNormalMode)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::cout << "running robot" << std::endl;
            double speed = 1;
            double acceleration = 0.6;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
            ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);

            RobotMove Robot(msInterval, &gripper, &rtdeControl, &rtdeRecive, speed, acceleration);

            mThrow.addPath(Robot.moveRobotJ(qHome));
            mThrow.addPath(Robot.moveRobotJ(qSafeGrib));
            mThrow.addPath(Robot.moveRobotL(posGribReadyR, rpyGribReady));
            Robot.setSpeedAcc(0.1, 0.05);
            mThrow.addPath(Robot.moveRobotL(posBallR, rpyBall));
            Robot.setSpeedAcc(speed, acceleration);
            gripper.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            //while(!gripper.hasGripped());
            mThrow.addPath(Robot.moveRobotL(posGribReadyR, rpyGribReady));
            mThrow.addPath(Robot.moveRobotL(qSafeGrib));
            mThrow.addPath(Robot.moveRobotJ(qHome));

    } else
        {
        std::cout << "a collision or an invalid pose will occur" << std::endl;
        std::cout << "returning without moving robot" << std::endl;
        }
    }

    void getBall(rw::math::Vector3D<> ballPositionW, rw::math::RPY<> gripOrientation)
    {
      double distance = 0.1;    //Hardcoded a distance of 20cm above ball
      return getBall(ballPositionW, gripOrientation,distance);
    }

    void getBall(rw::math::Vector3D<> ballPositionW)
    {
      double distance = 0.1;    //Hardcoded a distance of 20cm above ball
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

     return getBall(ballPositionW, rpyBall , distance);
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

        double xDistanceToCup = cupPose[0] - throwPose[0];
        double yDistanceToCup = cupPose[1] - throwPose[1];
        double zDistanceToCup = cupPose[2] - throwPose[2];

        double den = 9.82*(pow(xDistanceToCup,2)+pow(yDistanceToCup,2));
        double num = 2 * pow(cos(vinkel),2) * tan(vinkel) * (sqrt(pow(xDistanceToCup,2)+pow(yDistanceToCup,2))) - zDistanceToCup;

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

        std::cout << "Lenght to ramppose " << lenghtToRampPose << std::endl;

        double yLenght = lenghtToRampPose * cos(vinkel);
        double xLenght = 0;
        double zLenght = lenghtToRampPose * sin(vinkel);
        std::cout << "Length of translation for the throw : " << sqrt(yLenght*yLenght+zLenght*zLenght) << std::endl;

        startRampPose = rw::math::Vector3D<double>(xLenght, yLenght, zLenght);

        std::cout << "Start rampPose " << startRampPose << std::endl;

        rw::math::Vector3D<double> startPose = throwPose - startRampPose;
        double safety = 1;
        stopPose = (throwPose + startRampPose) * safety;

        std::cout << "Start Pose " << startPose << std::endl;
        std::cout << "Stop Pose " << stopPose << std::endl;

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

    void rotateThrow(rw::math::Vector3D<> cupPosition){

    }

    void circleThrow(rw::math::Vector3D<> cupPosition, double angle){
        double degrees = 0.0174532925;
        DetectCollision coli(scenePath);
        //Base Q[0] er roteret til center!

        rw::math::Q qHome(-1.151,-3.1415/2,0,-3.1415/2,0,0); // Home pos
        rw::math::Q qSafeGrib(-1.151, -2.202, -0.935, -1.574, 1.571, -0.003);   // Hardcoded safe gripping position
        std::cout << "Angle is : " << angle << std::endl;
        //                                  Base         Shoulder                        Elbow                         Wrist1                           Wrist 2    Wrist 3
        rw::math::Q qReleaseBall            ((-1.1847) , (-angle*degrees)                  , (0)                          , (0*degrees)                , (1.5708) , (0-3.1415/2));
        rw::math::Q qReleaseBallBeforeRotate((-1.1847) , (-angle*degrees)                  , (0)                          , (0*degrees)                , (1.5708) , (0-3.1415/2));
        rw::math::Q qStartPoseTiltAngle     ((-1.1847) , ((45-angle)*degrees)              , (45*degrees)                 , (45*degrees)               , (1.5708) , (0-3.1415/2));
        rw::math::Q qStopPoseTiltAngle      ((-1.1847) , ((-135+90-angle)*degrees)         , (-45*degrees)                , (-45*degrees)              , (1.5708) , (0-3.1415/2));

        rw::math::Vector3D<>robotPosition (0.40,0.90,0);

        //Rotate base to target
        qReleaseBall[0] -= rotateBaseToCupAngle(cupPosition,robotPosition);
        qStartPoseTiltAngle[0] -= rotateBaseToCupAngle(cupPosition,robotPosition);
        qStopPoseTiltAngle[0] -= rotateBaseToCupAngle(cupPosition,robotPosition);

        // Get tcp coords from q value
        coli.setState(qReleaseBall);
        rw::math::Transform3D<> tcpTrans = coli.getTransform();
        rw::math::Vector3D<> tcpPosRobot = tcpTrans.P();
        rw::math::Vector3D<> tcpPosWorld = robot2World(tcpPosRobot);
        double throwSpeed = speed(angle,tcpPosWorld,cupPosition);
        std::cout << "Joint speed at throw is : " << throwSpeed << std::endl;

        DetectCollision dc(scenePath);

        // Start the simulation
        bool isNormalMode;

        {
            double simSpeed = 3;
            double simAcc = 3;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
            ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");

            RobotMove Robot(msInterval,&gripper,&rtdeControl,&rtdeRecive,simSpeed,simAcc);
            dc.setHasCollided(false);
            dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
            dc.isCollision(Robot.moveRobotJ(qReleaseBallBeforeRotate).getJointPoses());
            dc.isCollision(Robot.moveRobotJ(qReleaseBall).getJointPoses());
            dc.isCollision(Robot.moveRobotJ(qStartPoseTiltAngle).getJointPoses());
            Robot.setAcc(3);
            Robot.setSpeed(3.14);
            dc.isCollision(Robot.moveRobotJ(qStopPoseTiltAngle).getJointPoses());
            dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
            isNormalMode = rtdeRecive.getSafetyMode() == 1;
        }
         // DO IT ROBOT MAN!

         if(!dc.getHasCollided() && isNormalMode){

             double speed = 1;
             double acc = 0.5;
             double msInterval = 5;
             ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
             ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);

             RobotMove Robot(msInterval,&gripper,&rtdeControl,&rtdeRecive,speed,acc);

             mThrow.addPath(Robot.moveRobotJ(qHome));
             mThrow.addPath(Robot.moveRobotJ(qReleaseBallBeforeRotate));
             mThrow.addPath(Robot.moveRobotJ(qReleaseBall));
             mThrow.addPath(Robot.moveRobotJ(qStartPoseTiltAngle));
             Robot.setSpeedAcc(1, 3.14);
             mThrow.addPath(Robot.moveRobotJRelease(qStopPoseTiltAngle,qReleaseBall,0.015));
             std::cout << mThrow.getPaths().back();
             Robot.setSpeedAcc(speed, acc);
             mThrow.addPath(Robot.moveRobotJ(qSafeGrib));
             mThrow.addPath(Robot.moveRobotJ(qHome));
        }
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

        double lenght = (-releasePos[2] / eigenVec[2]) - lenghtOffset;
        if (lenght > 0.5) lenght = 0.5;

        rw::math::Vector3D<> rampPosW = releasePos + lenght * eigenVec;  //std::cout << rampPosW << std::endl;
        rw::math::Vector3D<> endPosW  = releasePos - lenght * eigenVec; // std::cout << endPosW << std::endl;

        //rw::math::Q qSafeGrib(testval, -2.202, -0.935, -1.574, 1.571, -0.003);

        rw::math::Vector3D<> rampPosR = world2Robot(rampPosW);                            //std::cout << "Start ramp pose coordinates R" << rampPosR << std::endl;
        rw::math::Vector3D<> endPosR = world2Robot(endPosW);                              //std::cout << "End ramp pose coordinates R"endPosR << std::endl;

        //BEGIN THROW HERE
        //rw::math::RPY<> throwOrientation(0, 0, -1.157);
        rw::math::RPY<> throwOrientation(2.6, -1.69, 0);
         //rw::math::RPY<> throwOrientation(0.733, -1.477, 0);
        DetectCollision dc(scenePath);

        Path throwPath;
        bool isNormalMode;
        double throwSpeed = speed(angle, cupPos, releasePos);

        {
            double simSpeed = 3;
            double simAcc = 3;
            double msInterval = 10;

           std::cout << "running sim" << std::endl;
           ur_rtde::RTDEControlInterface rtdeControl("127.0.0.1");
           ur_rtde::RTDEReceiveInterface rtdeRecive("127.0.0.1");

           RobotMove Robot(msInterval, &gripper, &rtdeControl, &rtdeRecive, simSpeed, simAcc);

           dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
           dc.isCollision(Robot.moveRobotJ(rampPosR, throwOrientation).getJointPoses());
           throwPath = Robot.moveRobotL(endPosR, throwOrientation);
           dc.isCollision(throwPath.getJointPoses());
           dc.isCollision(Robot.moveRobotJ(qHome).getJointPoses());
          isNormalMode = rtdeRecive.getSafetyMode() == 1;
        }

        std::cout << throwPath << std::endl;
        rw::math::Vector3D<> releasePosR =  world2Robot(releasePos);

        double lowestDiff = 10000;
        double timeOfRelease;
        const std::vector<std::vector<double>> toolPose = throwPath.getToolPose();
        for (unsigned int i = 0; i < toolPose.size(); i++){

            double diff = 0;
            diff += std::abs(toolPose.at(i).at(0) - releasePosR[0]);
            diff += std::abs(toolPose.at(i).at(1) - releasePosR[1]);
            diff += std::abs(toolPose.at(i).at(2) - releasePosR[2]);
            if (diff < lowestDiff){
                lowestDiff = diff;
               timeOfRelease = throwPath.getElapsedTime().at(i);
            }
        }

        std::cout << "Here is the time of throw: " << timeOfRelease << std::endl;
        std::vector<double> pathSpeed;
        for (std::vector<double> toolSpeed : throwPath.getToolVel()){
            pathSpeed.push_back(mod(toolSpeed));
        }

        for (double speed : pathSpeed){
           std::cout <<  speed << std::endl;
        }

        if (!dc.getHasCollided() && isNormalMode)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "running robot" << std::endl;
            double moveSpeed = 1;
            double acceleration = 0.3;
            double throwAcc = 3;
            double msInterval = 10;
            ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);
            ur_rtde::RTDEReceiveInterface rtdeRecive(mIpAdress);

            RobotMove Robot(msInterval,&gripper, &rtdeControl, &rtdeRecive, moveSpeed, acceleration);

            mThrow.addPath(Robot.moveRobotJ(qHome));
            mThrow.addPath(Robot.moveRobotJ(rampPosR, throwOrientation));
            //std::thread releaseThread(&RobotControl::releaseBall, this, timeOfRelease);
            mThrow.addPath(Robot.moveRobotLRelease(endPosR, throwOrientation, releasePosR, 0.005));
            mThrow.addPath(Robot.moveRobotJ(qHome));
           // releaseThread.join();

            std::cout << "calculated throw speed: " <<throwSpeed << std::endl;
            std::cout << mThrow.getPaths()[2] << std::endl;

    } else
        {
        std::cout << "a collision will occur" << std::endl;
        std::cout << "returning without moving robot" << std::endl;
        }
    }


    void releaseBall(double releaseTime){
     std::this_thread::sleep_for(std::chrono::microseconds( (long int)((releaseTime+0.1) * 1000000)));
     gripper.open();
    }

    double mod(std::vector<double> vec){
        double res = 0;
        for (double v : vec){
            res += v*v;
        }
        return std::sqrt(res);
    }

    void toggleGripper() {
        if (gripper.hasGripped())
            gripper.open();
        else
            gripper.close();
    }

    void closeOpenGripper() {
        gripper.close();
        while(!gripper.hasGripped());
        gripper.open();
    }


    int getCellNr() const{return cellNr;}
    void setCellNr(int value){cellNr = value;}

    double getThrowAcc() const {return mThrowAcc;}
    void setThrowAcc(double throwAcc) {mThrowAcc = throwAcc;}

    double getThrowSpeed() const{return mThrowSpeed;}
    void setThrowSpeed(double throwSpeed) {mThrowSpeed = throwSpeed;}

private:
    std::string mIpAdress;
    Gripper gripper;
    rw::math::Vector3D<> mCalPos;
 rw::math::Rotation3D<> mCalRot;
 rw::math::Rotation3D<> mInvCalRot;
 Throw mThrow;
 const rw::math::Q qHome = rw::math::Q(-1.151,-3.1415/2,0,-3.1415/2,0,0);
 std::string  scenePath;
 int cellNr;
 double mThrowSpeed = 3.14, mThrowAcc = 3.14;
};

