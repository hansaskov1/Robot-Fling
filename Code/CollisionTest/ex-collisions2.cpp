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

std::vector<double> TObjectToTVector(rw::math::Transform3D<> TObject){
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







int main(int argc, char** argv)
{

    std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";

    rw::math::Vector3D<> Pcal(0.6521, 0.0503, 0.0411);
    rw::math::Rotation3D<> Rcal(    0.2577,    0.9662,   -0.0013,
                                    0.9662,   -0.2577,   -0.0111,
                                    0.0111,   -0.0016,    0.9999);

    rw::math::Transform3D<> TCal(Pcal,Rcal);


    rw::math::Q startQ{2.387, -1.847, 1.759, -3.055, -2.387, -0.006};
    DetectCollision dc(path,startQ);

    std::vector<double> qSafeGrib = {-0.003, -2.202, -0.935, -1.574, 1.571, -0.003};
    std::vector<double> TBall = {0.586,-0.261, 0.172, -2.22065, 2.22065, 0};
    std::vector<double> TGribReady = TBall;
    TGribReady[2] += 0.15;

    rw::math::Vector3D<> P1(0.586,-0.261, 0.172);
    rw::math::RPY<> R1 ( -2.22065, 2.22065, 0);
    rw::math::Transform3D<> T1(P1, R1);

    std::cout << T1 << std::endl;

     std::vector<double> qHome{/*-1.151*/0,-3.1415/2,0,-3.1415/2,0,0};
/*
    std::vector<std::vector<double>> qVec1 = getQFromSim(qSafeGrib,10);
    std::vector<std::vector<double>> qVec2 = getQFromSim(TGribReady,1,1,10);
    std::vector<std::vector<double>> qVec3 = getQFromSim(TBall,1,1,10);
    std::vector<std::vector<double>> qVec4 = getQFromSim(TGribReady,1,1,10);
    std::vector<std::vector<double>> qVec5 = getQFromSim(qSafeGrib,10);
    std::vector<std::vector<double>> qVec6 = getQFromSim(qHome,10);

    std::cout << "Done" << std::endl;

    std::cout << dc.isCollision(qVec1) << std::endl;
    std::cout << dc.isCollision(qVec2) << std::endl;
    std::cout << dc.isCollision(qVec3) << std::endl;
    std::cout << dc.isCollision(qVec4) << std::endl;
    std::cout << dc.isCollision(qVec5) << std::endl;
    std::cout << dc.isCollision(qVec6) << std::endl;

*/


    rw::math::Vector3D<> VGribReady(0.586,-0.261, 0.172 + 0.15);
    rw::math::RPY<> RGribReady( -2.22065, 2.22065, 0);
    rw::math::Transform3D<> TGribReadyObject(VGribReady, RGribReady);
    rw::math::Transform3D<> TGribReadyCalObject = TCal * TGribReadyObject;
    std::vector<double> TGribReadyCalVector = TObjectToTVector(TGribReadyCalObject);



    rw::math::Vector3D<> TGribReadyCalTranslation = Rcal * VGribReady;
    double x = TGribReadyCalTranslation[0];
    double y = TGribReadyCalTranslation[1];
    double z = TGribReadyCalTranslation[2];
    std::vector<double> TGribReadyCalVector2 = {x, y, z, -2.22065, 2.22065, 0};


   // ur_rtde::RTDEControlInterface rtde_control("192.168.100.53");
    ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");

    double speed = 1;
    double acceleration = 1;


    std::cout << "TGribReadyVectorSim " << std::endl;
    for (double value: TGribReady){
        std::cout << value << " ";
    }
    std::cout << std::endl << std::endl;


    std::cout << "TGribReadyVectorCal " << std::endl;
    for (double value: TGribReadyCalVector2){
        std::cout << value << " ";
    }
    std::cout << std::endl;



    rtde_control.moveJ(qHome, speed, acceleration);
    rtde_control.moveJ(qSafeGrib, speed, acceleration);
    rtde_control.moveL({1.05, 0.54, 0.32, -2.22, 2.22, 0},speed,acceleration);
    rtde_control.moveJ(qSafeGrib, speed, acceleration);
    rtde_control.moveJ(qHome, speed, acceleration);



      return 0;




}
