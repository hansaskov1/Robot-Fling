#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;
int main(int argc, char** argv) {

    RPY<> rpy(0, 0, 90*Deg2Rad); // 90 degree rotation around x-axis
    Rotation3D<> rot = rpy.toRotation3D(); // create Rotation3D matrix
    EAA<> eaa( rot ); // construct eaa from rotation3d
    Quaternion<> quat( rot ); // construct quaternion from rotation3d
    // there are streaming operators for all math types
    Log::infoLog() << rpy << std::endl;
    Log::infoLog() << rot << std::endl;
    Log::infoLog() << eaa << std::endl;
    Log::infoLog() << quat << std::endl;

    Log::infoLog() << std::endl;


    Log::infoLog() << rot*Vector3D<>(0,1,0) << std::endl;           // rotate a vector (0,1,0) 90 degrees around x-axis
    Transform3D<> t1( Vector3D<>(0,0,1), rot);                      // transform a vector
    Log::infoLog() << t1*Vector3D<>(0,1,0) << std::endl;
    Log::infoLog() << inverse( rot ) << std::endl;                  // calculate the inverse rotation
    Log::infoLog() << inverse( t1 ) << std::endl;                   // calculate the inverse transform
    Log::infoLog() << t1.R() << t1.P() << std::endl;                // get the rotation and translation part of a transform

    RPY<> R1(90*Deg2Rad, 0, 0);
    RPY<> R2(0, 0, 0);
    Vector3D<> V1(1,1,1);
    Vector3D<> V2(0,0,1);

    Transform3D<> T1(V1, R1);
    Transform3D<> T2(V2, R2);

    Log::infoLog() << std::endl;
    Log::infoLog() << T1 * T2 << std::endl;








}
