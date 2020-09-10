#include <stdio.h>
#include <iostream>


int main(int argc, char** argv)
{


   // sdt::cout << "123" << std::endl;

    // The constructor simply takes the IP address of the Robot
    RTDEControlInterface rtde_control("127.0.0.1");
    // First argument is the pose 6d vector followed by speed and acceleration
    rtde_control.moveL({-0.143, -0.435, 0.20, -0.001, 3.12, 0.04}, 0.5, 0.2);



    return 0;
}
