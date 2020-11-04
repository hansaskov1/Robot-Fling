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
#include "DetectCollision.h"
#include "RobotControl.h"
#include <vector>
#include <chrono>
#include <thread>
#include <future>


int main(int argc, char** argv)
{
    //NOTE
    //Husk at initialisere robotten til at have tcp punktet 150mm i y aksen!!!      //Det ændres under Initialize robot -> Configure TCP

    rw::math::Vector3D<> PCal(0.400624689065891, 0.901530744085863, 0.042187492976487);
    rw::math::Rotation3D<double> RCal(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);

    RobotControl RC("127.0.0.1",PCal,RCal);
    rw::math::Vector3D<> ballPosition(0.20,0.20,0.01);

    RC.getBall(ballPosition,0.2);

    std::string path = "../../Code/Scenes/XMLScenes/RobotOnTable/Scene.xml";
    DetectCollision dc(path);


    return 0;
}
