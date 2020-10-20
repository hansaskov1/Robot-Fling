#include <stdio.h>
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <chrono>
#include <thread>


// let go of ball at throwingPose, with appropriate velocity and angle for the ball to hit target pose
void throwBall(const std::array<double, 3> targetCoordinates, const std::array<double, 3> throwingCoordinates, const double angle){


        //calculate velocity from angle and diffrence in position to hit target
        //calculate ramp pose:
            // 1. which is parralel with the target and throwing coordinates in the xy plane                            (calculate xy plane from target/throwing coordinates)
            // 2. which angle between it and throwing coordinates is equal to the variable "angle" in the z coordinate  (calculate line in xy plane with parameter angle that passes through throwing coordinate)
            // 3. which orientation is purpendicular to the line in the xy plane so the ball can
        //calculate acceleration
        //define ramp up pose
        //define throwing pose
        //calculate time at which the tcp point is at throwingCoordinate from a linear motion in carteesian coordinates and a constant acceleration.
        //begin movement to throwing pose
        //call gripper to open after defined time.





}




using namespace ur_rtde;
using namespace std::chrono;

int main()
{
    // example from ur_rtde
    RTDEControlInterface rtde_control("127.0.0.1");

      double velocity = 0.5;
      double acceleration = 0.5;
      double blend_1 = 0.0;
      double blend_2 = 0.02;
      double blend_3 = 0.0;
      std::vector<double> path_pose1 = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, blend_1};
      std::vector<double> path_pose2 = {-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};
      std::vector<double> path_pose3 = {-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, blend_3};

      std::vector<double> path_pose4 = {0, -1.5708, 0, -1.5708, 0, 0, velocity, acceleration, blend_3};
      std::vector<double> path_pose5 = {-2.57064, -1.31272, -0.826605, -3.27744, -3.09263, 1.63532, velocity, acceleration, blend_3};
      std::vector<double> path_pose6 = {-2.5891, -1.48134, -1.62249, -1.90522, -1.03886, -0.9977, velocity, acceleration, blend_3};

      std::vector<std::vector<double>> path;
      path.push_back(path_pose4);
      path.push_back(path_pose5);
      path.push_back(path_pose6);

      // Send a linear path with blending in between - (currently uses separate script)
      rtde_control.moveJ(path);
      rtde_control.stopScript();

      return 0;
}
