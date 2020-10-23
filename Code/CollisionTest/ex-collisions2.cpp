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


void moveRobotJ(std::vector<double> position){
   ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
   rtde_control.moveJ(position);
   std::cout << "Moving to position" << std::endl;
}

std::vector<std::vector<double>> getQFromSim(std::vector<double> position, unsigned int msInterval){


    std::atomic<bool> stop {false};
    std::promise<std::vector<std::vector<double>>> promiseQVec;

    std::thread control(moveRobotJ, position);
    auto futureQVec = promiseQVec.get_future();
    std::thread recive(&fetchQValues,std::move(promiseQVec),  std::ref(stop), 100);

    control.join();
    stop = true;
    recive.join();

    return futureQVec.get();

}













/*
std::vector<double> interpolateQDiff(unsigned int times, rw::math::Q startQ, rw::math::Q endQ){

   std::vector<double> vStartQ = startQ.toStdVector();
   std::vector<double> vEndtQ = startQ.toStdVector();
   std::vector<double> vDiff;

   for(unsigned int i = 0; i < 6; i++){
        vDiff.push_back(vStartQ[i]-vEndtQ[i]);
   }

   std::vector<double> partVec;

   for(unsigned int j = 1; j < 6; j++){
       partVec.push_back(vDiff[j] * (i / times));
    }
    return partvec;


}
*/

int main(int argc, char** argv)
{

/*
    std::cout << "*** press enter to exit the program gracefully\n\n" ;

     std::atomic<bool> running { true } ;
     const unsigned int update_interval = 50 ; // update after every 50 milliseconds
     std::thread update_thread( update, std::ref(running), update_interval ) ;

     // do other stuff in parallel: simulated below
     std::cin.get() ;

     // exit gracefully
     running = false ;
     update_thread.join() ;
*/



    std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";

    rw::math::Vector3D<> vec1(0.191, 0.351, 0.642);
    rw::math::RPY<> R1 ( -1.571, -0.006, -1.571);
    rw::math::Transform3D<> T1(vec1, R1);

    rw::math::Q startQ{2.387, -1.847, 1.759, -3.055, -2.387, -0.006};
    DetectCollision dc(path,startQ);

     std::cout << dc.getState() << std::endl;
     std::cout << dc.isCollision(10,T1) << std::endl;
     std::cout << dc.getState() << std::endl;
     std::cout << vec1[0] << std::endl;


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
      }




    std::vector<std::vector<double>> qVec = getQFromSim(startQ.toStdVector(),100);

    for(std::vector<double> q : qVec){
        for (double val : q){
            std::cout << val << " ";
        }
        std::cout << std::endl;
     }


      ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");


      std::cout << "Moving to start position" << std::endl;
      rtde_control.moveJ({2.387, -1.847, 1.759, -3.055, -2.387, -0.006});


      std::cout << "Moving predicted path" << std::endl;
      rtde_control.moveJ(jointPath1);


/*
     ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");

      // Parameters
      velocity = 0.5;
       acceleration = 0.5;
      double dt = 1.0/500; // 2ms
      double lookahead_time = 0.1;
      double gain = 300;
      std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

      // Move to initial joint position with a regular moveJ
      rtde_control.moveJ(joint_q);







      // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
      for (unsigned int i=0; i<1000; i++)
      {
        auto t_start = high_resolution_clock::now();
        rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
        joint_q[1] += 0.001;
        joint_q[2] += 0.001;
        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < dt)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
      }




      rtde_control.servoStop();

      std::cout << "Moving to Home" << std::endl;
      rtde_control.moveJ({0,-3.145/2,0,-3.145/2,0,0});
      std::cout << "Done" << std::endl;


      rtde_control.stopScript();

      */

      return 0;
}
