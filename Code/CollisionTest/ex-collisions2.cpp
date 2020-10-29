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

using namespace std::chrono;



class DetectCollision{

public:
    DetectCollision(const std::string & path)
    {
        const rw::models::WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load(path);
        if (!workcell.isNull())
        {
             mDevice = workcell->getDevices().front();
             mState = workcell->getDefaultState();
             mDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
        } else
        {
            RW_THROW("WorkCell could not be loaded.");
        }
    }

    DetectCollision(const std::string &path, rw::math::Q startQ) : DetectCollision(path) {
        mDevice->setQ(startQ, mState);
    }


    bool isCollision(size_t checks, rw::math::Transform3D<> endTransform){

        rw::trajectory::LinearInterpolator<rw::math::Transform3D<>> interpolator(getTransform(), endTransform, 1);
        std::vector<rw::math::Q> QVec;

        for (unsigned int i = 1; i < checks; i++)
        {
            rw::math::Transform3D<> T = interpolator.x(static_cast<double>(i) / static_cast<double>(checks));
            rw::math::Q q = calcInverse(T);

            if (q.empty())
            {
                std::cout << "Could not find inverse kinematics from " << T << std::endl;
                return true;
            }

            if (checkCollision())
            {
                std::cout << "Collision occured at" << T << std::endl;
                return true;
            }
            QVec.push_back(q);
        }
        mQVec = QVec;
        return false;
    }

    bool isCollision(size_t checks,rw::math::Transform3D<> startTransform , rw::math::Transform3D<> endTransform){

        rw::trajectory::LinearInterpolator<rw::math::Transform3D<>> interpolator(startTransform, endTransform, 1);
        std::vector<rw::math::Q> QVec;

        for (unsigned int i = 1; i < checks; i++)
        {
            rw::math::Transform3D<> T = interpolator.x(static_cast<double>(i) / static_cast<double>(checks));
            rw::math::Q q = calcInverse(T);

            if (q.empty())
            {
                std::cout << "Could not find inverse kinematics from " << T << std::endl;
                return true;
            }

            if (checkCollision())
            {
                std::cout << "Collision occured at" << T << std::endl;
                return true;
            }
            QVec.push_back(q);
        }
        mQVec = QVec;
        return false;
    }

    bool isCollision(std::vector<std::vector<double>> qVec){

        for (std::vector<double> stdQ : qVec){
            rw::math::Q q(stdQ);
            mDevice->setQ(q,mState);
            if (checkCollision()){
                return true;
            }
        }
        return false;
    }


    void setState(const rw::math::Q jointPosition){mDevice->setQ(jointPosition,mState);}
    rw::math::Q getState() const { return mDevice->getQ(mState);}
    std::vector<rw::math::Q> getQVec() const { return mQVec; }
    rw::math::Transform3D<> getTransform() const {return mDevice->baseTend(mState);}

private:

    rw::math::Q calcInverse(rw::math::Transform3D<> transform)
    {
        //rw::invkin::JacobianIKSolver::Ptr solver1(new rw::invkin::JacobianIKSolver(mDevice, mState));
        rw::invkin::JacobianIKSolver solver(mDevice, mState);
        std::vector<rw::math::Q> qVec = solver.solve(transform, mState);
        rw::math::Q q;
        if (!qVec.empty())
        {
            q = qVec.back();
            mDevice->setQ(q, mState);
            //std::cout << q << std::endl;
        }

        return q;       // NOTE: If there is no kinematic solution q will be uninitialized.
    }

    bool checkCollision()
    {
        rw::proximity::CollisionDetector::QueryResult resEnd;
            mDetector->inCollision(mState, &resEnd);
            return !resEnd.collidingFrames.empty();
    }

    rw::models::Device::Ptr mDevice;
    rw::kinematics::State mState;
    rw::proximity::CollisionDetector::Ptr mDetector;
    std::vector<rw::math::Q> mQVec;

};


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

    void fetchQValues(std::promise<std::vector<std::vector<double>>> && returnQV ,std::atomic<bool>& stop, unsigned int msInterval)
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

/*
    std::vector<std::vector<double>> getQFromSim(rw::math::Q jointValues, unsigned int msInterval)  // For Linear movement in Joint Space
    {
        std::vector<double> jointValuesStdVec = jointValues.toStdVector();
        std::atomic<bool> stop {false};
        std::promise<std::vector<std::vector<double>>> promiseQVec;
        auto futureQVec = promiseQVec.get_future();

        std::thread recive(&RobotControl::fetchQValues,std::move(promiseQVec),  std::ref(stop), msInterval);
        auto moveJ = [](std::vector<double> jointValues)
        {
            ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
            rtde_control.moveJ(jointValues,1,1);
        };
        std::thread control(moveJ, jointValuesStdVec);

        control.join();
        stop = true;
        recive.join();
        return futureQVec.get();
    }


    std::vector<std::vector<double>> getQFromSim(rw::math::RPY<> orientation, rw::math::Vector3D<> position , unsigned int msInterval){ // For Linear movement in Tool Space

        std::vector<double> toolPositionStdVec = vecRPY2stdVec(position,orientation);
        std::atomic<bool> stop {false};
        std::promise<std::vector<std::vector<double>>> promiseQVec;

        auto futureQVec = promiseQVec.get_future();
        std::thread recive(&RobotControl::fetchQValues,std::move(promiseQVec),  std::ref(stop), msInterval);
        auto moveL = [](std::vector<double> toolPosition)
        {
            ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
            rtde_control.moveL(toolPosition,1,1);
        };
        std::thread control(moveL, toolPositionStdVec);

        control.join();
        stop = true;
        recive.join();
        return futureQVec.get();
    }
*/

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

/*
        //run simulation...
        std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";

        DetectCollision dc(path);
        std::vector<bool> collisionList;

        double msInterval = 10;
        std::cout << "Starting sim" << std::endl;
        collisionList.push_back(dc.isCollision(getQFromSim(qHome, msInterval)));
        collisionList.push_back(dc.isCollision(getQFromSim(qSafeGrib, msInterval)));
        collisionList.push_back(dc.isCollision(getQFromSim(posGribReadyW,rpyGribReady, msInterval)));
        collisionList.push_back(dc.isCollision(getQFromSim(posBallW, rpyBall, msInterval)));
        collisionList.push_back(dc.isCollision(getQFromSim(qSafeGrib, msInterval)));
        collisionList.push_back(dc.isCollision(getQFromSim(qHome, msInterval)));

        bool collision;
        for (bool isColl : collisionList)
        {
            (isColl)? std::cout << "true" : std::cout << "false";
            std::cout << endl;

            if (isColl) collision = true;
        }

    if (!collision)
    {*/
        ur_rtde::RTDEControlInterface rtdeControl(mIpAdress);

        double speed = 2;
        double acceleration = 2;
        rtdeControl.moveJ(qHomeStdVec, speed, acceleration);
        rtdeControl.moveJ(qSafeGribStdVec, speed, acceleration);
        rtdeControl.moveL(gribReadyR,speed,acceleration);
        rtdeControl.moveL(ballR,speed,acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        rtdeControl.moveL(gribReadyR,speed,acceleration);
        rtdeControl.moveJ(qSafeGribStdVec, speed, acceleration);
        rtdeControl.moveJ(qHomeStdVec, speed, acceleration);
    /*} else
    {
        std::cout << "a collision has occured" << std::endl;
    }*/
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




int main(int argc, char** argv)
{
     rw::math::Vector3D<> PCal(0.400624689065891, 0.901530744085863, 0.042187492976487);
     rw::math::Rotation3D<double> RCal(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);


    RobotControl RC("127.0.0.1",PCal,RCal);

    rw::math::Vector3D<> ballPosition(0.2,0.2,0.1);

    RC.getBall(ballPosition);


      return 0;

}
