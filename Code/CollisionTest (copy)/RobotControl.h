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

    // Sim

    void fetchQValues(std::promise<std::vector<std::vector<double>>> && returnQV ,std::atomic<bool>& stop, unsigned int msInterval) //Used in thread
    {
        std::vector<std::vector<double>> qVector;
        ur_rtde::RTDEReceiveInterface rtde_recieve("127.0.0.1");
        while(!stop)
        {
            std::vector<double> q = rtde_recieve.getActualQ();
            qVector.push_back(q);
            std::this_thread::sleep_for(std::chrono::milliseconds(msInterval));
        }
        returnQV.set_value(std::move(qVector));
    }



    static void moveL(std::vector<double> toolPosition) // Used in own thread
    {
        ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
        rtde_control.moveL(toolPosition,2,2);
    }

    static void moveJ(std::vector<double> jointValues) // Used in own thread
    {
        ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
        rtde_control.moveJ(jointValues,2,2);
    }


    std::vector<std::vector<double>> getQFromSim(rw::math::Q jointValues, unsigned int msInterval)  // For Linear movement in Joint Space
    {
        std::vector<double> jointValuesStdVec = jointValues.toStdVector();
        std::atomic<bool> stop {false};
        std::promise<std::vector<std::vector<double>>> promiseQVec;
        auto futureQVec = promiseQVec.get_future();

        std::thread recive(&RobotControl::fetchQValues, this ,std::move(promiseQVec),  std::ref(stop), msInterval);
        std::thread control(RobotControl::moveJ, jointValuesStdVec);

        control.join();
        stop = true;
        recive.join();
        return futureQVec.get();
    }


    std::vector<std::vector<double>> getQFromSim( rw::math::Vector3D<> position,rw::math::RPY<> orientation, unsigned int msInterval){ // For Linear movement in Tool Space

        std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
        std::atomic<bool> stop {false};
        std::promise<std::vector<std::vector<double>>> promiseQVec;
        auto futureQVec = promiseQVec.get_future();

        std::thread recive(&RobotControl::fetchQValues, this ,std::move(promiseQVec),  std::ref(stop), msInterval);
        std::thread control(RobotControl::moveL, toolPositionStdVec);

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


        std::vector<double> qHomeStdVec = qHome.toStdVector();
        std::vector<double> qSafeGribStdVec = qSafeGrib.toStdVector();
        std::vector<double> gribReadyR = vecRPY2stdVec(posGribReadyR,rpyGribReady);
        std::vector<double> ballR = vecRPY2stdVec(posBallR,rpyBall);


        //run simulation...
        std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";

        DetectCollision dc(path);
        std::vector<bool> collisionList;

        double msInterval = 10;
        std::cout << "Starting sim" << std::endl;

        std::vector<std::vector<std::vector<double>>> QFullPath;

        QFullPath.push_back(getQFromSim(qHome, msInterval));
        QFullPath.push_back(getQFromSim(qSafeGrib, msInterval));
        QFullPath.push_back(getQFromSim(posGribReadyR,rpyGribReady, msInterval));
        QFullPath.push_back(getQFromSim(posBallR, rpyBall, msInterval));
        QFullPath.push_back(getQFromSim(posGribReadyR,rpyGribReady, msInterval));
        QFullPath.push_back(getQFromSim(qSafeGrib, msInterval));
        QFullPath.push_back(getQFromSim(qHome, msInterval));

        for (std::vector<std::vector<double>> &qPath : QFullPath )
        {
            /*for (std::vector<double> &qValue : qPath){
                qValue[0] += 1.151;
            }*/
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
        ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);

        double speed = 2;
        double acceleration = 2;
        rtdeControl.moveJ(qHomeStdVec, speed, acceleration);
        rtdeControl.moveJ(qSafeGribStdVec, speed, acceleration);
        rtdeControl.moveL(gribReadyR,speed,acceleration);
        rtdeControl.moveL(ballR,0.1,0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        rtdeControl.moveL(gribReadyR,speed,acceleration);
        rtdeControl.moveJ(qSafeGribStdVec, speed, acceleration);
        rtdeControl.moveJ(qHomeStdVec, speed, acceleration);
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


