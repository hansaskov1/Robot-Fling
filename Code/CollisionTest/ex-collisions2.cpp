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

    DetectCollision(const std::string &path, rw::math::Q startQ) : DetectCollision(path)
    {
        mDevice->setQ(startQ, mState);
    }


    bool isCollision(size_t checks, rw::math::Transform3D<> endTransform){

        rw::math::Transform3D<> beginTransform= mDevice->baseTend(mState);
        rw::trajectory::LinearInterpolator<rw::math::Transform3D<>> interpolator(beginTransform, endTransform, 1);
        mQVec.clear();

        for (unsigned int i = 1; i < checks; i++){

            rw::math::Transform3D<> T = interpolator.x(static_cast<double>(i) / static_cast<double>(checks));
          //  std::cout << T << std::endl;
            rw::math::Q q = calcInverse(T);

            if (q.empty()){
                std::cout << "Could not find inverse kinematics from " << T << std::endl;
                return true;
            }

            if (checkCollision())
                return true;

            mQVec.push_back(q);


        }

        return false;
    }

    void setState(const rw::math::Q jointPosition){
        mDevice->setQ(jointPosition,mState);

    }

    rw::math::Q getState() const{
        return mDevice->getQ(mState);
    }

    std::vector<rw::math::Q> getQVec() const
    {
     return mQVec;
    }





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
            std::cout << q << std::endl;
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



using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using rw::proximity::CollisionDetector;
using namespace rwlibs::proximitystrategies;
using rw::loaders::PathLoader;
using namespace rwlibs::pathplanners;



int main(int argc, char** argv)
{

    WorkCell::Ptr workcell = WorkCellLoader::Factory::load("/home/hans/Desktop/Ny mappe/XMLScenes/RobotOnTable/Scene.xml");
    Device::Ptr device = workcell->getDevices().front();
    State state = workcell->getDefaultState();
    CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


    rw::math::Vector3D<> vec1(0.191, 0.351, 0.642);
    rw::math::Vector3D<> vec2(-0.102, 0.806, 0.213);
    rw::math::Vector3D<> vec3(0.096, -0.351, -0.116);

    RPY<> R1 ( -1.571, -0.006, -1.571);
    RPY<> R2 (0.106, -0.191, 2.995);
    RPY<> R3 (2.435, 1.277, -1.977);

    Transform3D<> T1(vec1, R1);
    Transform3D<> T2(vec2, R2);
    Transform3D<> T3(vec3, R3);

   // Q[6]{0.191, 0.351, 0.642, -1.571, -0.006, -1.571}

    Q que{2.387, -1.847, 1.759, -3.055, -2.387, -0.006};

    //device->setQ(que, state);



     std::string path = "/home/hans/Desktop/Ny mappe/XMLScenes/RobotOnTable/Scene.xml";
     DetectCollision dc(path,device->getQ(state));
     //dc.setState(que);

     dc.setState(que);

     std::cout << dc.getState() << std::endl;

     std::cout << dc.isCollision(10,T1) << std::endl;


     ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
     //dc.getQVec()[0].toStdVector();



     double velocity = 1;
     double acceleration = 1;
     double blend_1 = 0.0;
     double blend_2 = 0.3;
     std::vector<std::vector<double>> jointPath;


     for (rw::math::Q value : dc.getQVec()){

         std::vector<double> path = value.toStdVector();
         path.push_back(velocity);
         path.push_back(acceleration);
         path.push_back(blend_1);
         jointPath.push_back(path);

     }

     rtde_control.moveJ({2.387, -1.847, 1.759, -3.055, -2.387, -0.006});

     std::cout << "test" << std::endl;

     rtde_control.moveJ(jointPath);


     rtde_control.moveJ({2.387, -1.847, 1.759, -3.055, -2.387, -0.006});

    double x = T1.P()[0];
    double y = T1.P()[1];
    double z = T1.P()[2];


     rtde_control.moveL({x , y, z, 0, 0 , 0});
}
