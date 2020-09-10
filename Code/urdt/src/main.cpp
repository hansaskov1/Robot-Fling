#include <stdio.h>
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>



int main()
{

    //std::cout << "hej" << std::endl;


    // The constructor simply takes the IP address of the Robot
    ur_rtde::RTDEControlInterface rtde_control("127.0.0.1");
    // First argument is the pose 6d vector followed by speed and acceleration
    rtde_control.moveL({-1.31305, -1.68469, -0.8460, -0.15287, 1.592, 4.8692}, 0.2, 0.5);


    ur_rtde::RTDEReceiveInterface rtde_receive("127.0.0.1");
    std::vector<double> joint_positions = rtde_receive.getActualQ();


    for (double position: joint_positions){
        std::cout << position << std::endl;
    }





    return 0;
}
