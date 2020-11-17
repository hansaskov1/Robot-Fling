#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
 /*   QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
    */


    rw::math::Vector3D<> PCal(0.400624689065891, 0.901530744085863, 0.042187492976487);
    rw::math::Rotation3D<double> RCal(0.923890908941640 ,0.382647484711815,-0.002547708521920,-0.382655561588167,0.923879135480505,-0.004697255522142,0.000556381736091,0.005314646509101,0.999985722383999);

    RobotControl RC("127.0.0.1",PCal,RCal);
    rw::math::Vector3D<> ballPosition(0.20,0.20,0.05);


    RC.getBall(ballPosition,0.2);
    std::cout << "call throw ball" << std::endl;
    rw::math::Vector3D<> cupPosition (0.10,0.10,0);
    RC.throwBall(cupPosition,0.05);



//    rw::math::Vector3D<> cupPosition (20,20,0);
//    rw::math::Vector3D<> ballReleasePosition (40,120,120);
//    std::cout << cupPosition << std::endl;
//    double speed = RC.speed(3.1614/4 , ballReleasePosition,cupPosition);
//    std::cout << "speed :  " << speed << std::endl;


//    double acc = 3;
//    double timeFromThrowPose = speed / acc;
//    double lengthToRampPose = 0.5 * acc * pow(timeFromThrowPose,2);

//    double x = lengthToRampPose * cos(3.1614/4);
//    double y = 0;
//    double z = lengthToRampPose * sin(3.1614/4);


//    rw::math::Vector3D<> startRamp(x,y,z);

//    rw::math::Vector3D<> startPose = ballReleasePosition - startRamp;

//    rw::math::Vector3D<> stopPose = ballReleasePosition + startRamp;

//    std::cout << "Start ramp : " << startRamp << "\n" <<
//                 "Ball release : " << ballReleasePosition << "\n" <<
//                 "StopPose : " << stopPose << "\n" <<
//                 "StartPose : " << startPose << "\n" <<
//                 std::endl;





//    RC.rampPose()




}
