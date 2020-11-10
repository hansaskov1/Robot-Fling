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

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/EAA.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/invkin/AmbiguityResolver.hpp>
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
#include <math.h>

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
             device = workcell->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        } else
        {
            RW_THROW("WorkCell could not be loaded.");
        }

/*
          std::string serialDevPath = "/home/hans/GithubCode/Robot-Fling/Code/Scenes/UR5_2017/UR5.xml";
          const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(serialDevPath);

         if (!wc.isNull())
         {
          device = wc->findDevice<rw::models::SerialDevice>("UR5_2017");

         } else
         {
             RW_THROW("SerialDeviceWorkcell could not be loaded.");
         }
*/
    }

    DetectCollision(const std::string &path, rw::math::Q startQ) : DetectCollision(path)
    {
        mDevice->setQ(startQ, mState);
    }


    bool isCollision(size_t checks, rw::math::Transform3D<> endTransform)
    {
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

    bool isCollision(size_t checks,rw::math::Transform3D<> startTransform , rw::math::Transform3D<> endTransform)
    {
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

    bool isCollisionUR(size_t checks, rw::math::Transform3D<> endTransform)
    {
        rw::trajectory::LinearInterpolator<rw::math::Transform3D<>> interpolator(getTransform(), endTransform, 1);
        std::vector<rw::math::Q> QVec;
        rw::math::Q lastPose = device->getQ(mState);

        for (unsigned int i = 1; i < checks; i++)
        {
            rw::math::Transform3D<> T = interpolator.x(static_cast<double>(i) / static_cast<double>(checks));
            std::vector<rw::math::Q> qPoses = calcInverseUR(T);

            if (qPoses.empty())
            {
                std::cout << "Could not find inverse kinematics from " << T << std::endl;
                return true;
            }

            rw::math::Q closestPose;
            double smallestPoseDiffrence = 10000000;  //hardcoded high value

            for (rw::math::Q nextPose: qPoses)
            {
               std::array<double, 6> poseDiffrence;
               double totalPoseDiffrence = 0;

               for (unsigned int i = 0; i < 6; i++)
               {
                    poseDiffrence[i] = lastPose[i] - nextPose[i];
                    totalPoseDiffrence += std::abs(poseDiffrence[i]);
               }

               if (smallestPoseDiffrence > totalPoseDiffrence)
               {
                    smallestPoseDiffrence = totalPoseDiffrence;
                    closestPose = nextPose;
               }

            }

//            closestPose[0] += 1.151;

            setState(closestPose);

            if (checkCollision())
            {
                std::cout << "Collision occured at" << T << std::endl;
                return true;
            }

            QVec.push_back(closestPose);
        }
        mQVec = QVec;
        return false;
    }


    bool isCollision(unsigned int checks,rw::math::Q endJointPose)
    {
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(mDevice->getQ(mState), endJointPose, checks);
        std::vector<rw::math::Q> QVec;

        for (unsigned int i = 1; i < checks; i++)
        {
            rw::math::Q q = interpolator.x(i);
            std::cout << q << std::endl;
            setState(q);

            if (checkCollision())
            {
                std::cout << "Collision occured at" << q << std::endl;
                return true;
            }
            QVec.push_back(q);
        }
        setState(endJointPose);
        mQVec = QVec;
        return false;
    }



    bool isCollision(std::vector<std::vector<double>> qVec)
    {
        for (std::vector<double> stdQ : qVec)
        {
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

    std::vector<rw::math::Q> calcInverseUR(rw::math::Transform3D<> transform)
    {
        //rw::invkin::JacobianIKSolver::Ptr solver1(new rw::invkin::JacobianIKSolver(mDevice, mState));
        rw::invkin::JacobianIKSolver solver(mDevice, mState);
        std::vector<rw::math::Q> qVec = solver.solve(transform, mState);

        return qVec;       // NOTE: If there is no kinematic solution qVec will be uninitialized.
    }


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

    bool checkCollision(rw::math::Q q)
    {
        setState(q);
        checkCollision();
    }

    rw::models::SerialDevice::Ptr device;
    rw::models::Device::Ptr mDevice;
    rw::kinematics::State mState;
    rw::proximity::CollisionDetector::Ptr mDetector;
    std::vector<rw::math::Q> mQVec;
};
