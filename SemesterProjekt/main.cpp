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
    RC.throwBall(0.05);






}
