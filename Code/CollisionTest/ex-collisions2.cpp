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



void fetchQValues(std::promise<std::vector<std::vector<double>>> && returnQV ,std::atomic<bool>& stop, unsigned int msInterval){
    std::vector<std::vector<double>> qVector;

    ur_rtde::RTDEReceiveInterface rtde_recieve("127.0.0.1");
    while(!stop){
        std::vector<double> q = rtde_recieve.getActualQ();
        qVector.push_back(q);
        std::this_thread::sleep_for(std::chrono::milliseconds(msInterval));
    }
    returnQV.set_value(std::move(qVector));
}


void moveRobotJ(std::vector<double> jointPosition){
   ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
   std::cout << "Moving to position" << std::endl;
   rtde_control.moveJ(jointPosition);

}

void moveRobotL(std::vector<double> toolPosition, double speed, double acceleration){
    ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
    std::cout << "Moving to position" << std::endl;
    rtde_control.moveL(toolPosition,speed,acceleration);

}

std::vector<std::vector<double>> getQFromSim(std::vector<double> position, unsigned int msInterval){

    std::atomic<bool> stop {false};
    std::promise<std::vector<std::vector<double>>> promiseQVec;

    auto futureQVec = promiseQVec.get_future();
    std::thread recive(&fetchQValues,std::move(promiseQVec),  std::ref(stop), msInterval);

    if (position.size() == 6){
        std::thread control(moveRobotJ, position);

        control.join();

    }

    stop = true;
    recive.join();
    return futureQVec.get();


}


std::vector<std::vector<double>> getQFromSim(std::vector<double> position, double speed, double acceleration, unsigned int msInterval){

    std::atomic<bool> stop {false};
    std::promise<std::vector<std::vector<double>>> promiseQVec;

    auto futureQVec = promiseQVec.get_future();
    std::thread recive(&fetchQValues,std::move(promiseQVec),  std::ref(stop), msInterval);
    std::thread control(moveRobotL, position, speed, acceleration);
    control.join();


    stop = true;
    recive.join();
    return futureQVec.get();

}





int main(int argc, char** argv)
{

    std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";

    rw::math::Vector3D<> vec1(0.191, 0.351, 0.642);
    rw::math::RPY<> R1 ( -1.571, -0.006, -1.571);
    rw::math::Transform3D<> T1(vec1, R1);

    rw::math::Q startQ{2.387, -1.847, 1.759, -3.055, -2.387, -0.006};
    std::vector<double> homeQ{0,-3.145/2,0,-3.145/2,0,0};
    DetectCollision dc(path,startQ);

     std::cout << dc.getState() << std::endl;
     std::cout << dc.isCollision(10,T1) << std::endl;
     std::cout << dc.getState() << std::endl;
     std::cout << vec1[0] << std::endl;

/*
      double velocity = 1;
      double acceleration = 1;
      double blend_1 = 0;

      std::vector<std::vector<double>> jointPath1;
      for (rw::math::Q value : dc.getQVec()){
          std::vector<double> path = value.toStdVector();
          path.push_back(velocity);
          path.push_back(acceleration);
          path.push_back(blend_1);
          jointPath1.push_back(path);
      }*/


     std::vector<double> tcpPosition = {0.191, 0.351, 0.642,-1.571, -0.006, -1.571};

    std::cout << "test1" << std::endl;
    std::vector<std::vector<double>> qVec1 = getQFromSim(startQ.toStdVector(),100);
    std::cout << "test2" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "test3" << std::endl;
    std::vector<std::vector<double>> qVec2 = getQFromSim(tcpPosition,1,1,10);
    std::vector<std::vector<double>> qVec3 = getQFromSim(homeQ,100);
       std::cout << "test4" << std::endl;


    std::cout << dc.isCollision(qVec1)<<std::endl;
    std::cout << dc.isCollision(qVec3) << std::endl;

      return 0;
}
