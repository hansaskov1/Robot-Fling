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
#include <DetectCollision.h>

using namespace std::chrono;




int main(int argc, char** argv)
{
    DetectCollision dc;


      return 0;

}
