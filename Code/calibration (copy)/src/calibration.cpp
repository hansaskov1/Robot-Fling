#include <stdio.h>
#include <iostream>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <thread>
#include <string>
#include <eigen3/Eigen/Core>

int main() {
  std::vector < std::array < double, 3 >> worldPositions;
  std::vector < std::array < double, 3 >> robotPositions;

  std::string urIp = "192.168.100.49";
  ur_rtde::RTDEReceiveInterface rtde_recieve(urIp);

  bool repeat = true;
  std::string continueHolder;

  while (repeat) {

    std::array < double, 3 > worldPosition {0};
    std::array < double, 3 > robotPosition {0};

    std::cout << "Move tcp to position" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::cout << "write \"x y z\" coordinate" << std::endl;

    std::cin >> worldPosition[0];
    std::cin >> worldPosition[1];
    std::cin >> worldPosition[2];

    worldPositions.push_back(worldPosition);

    std::vector < double > tcpPose = rtde_recieve.getActualTCPPose();
    robotPosition[0] = tcpPose[0] * 1000;
    robotPosition[1] = tcpPose[1] * 1000;
    robotPosition[2] = tcpPose[2] * 1000;

    robotPositions.push_back(robotPosition);


    std::cout << "Add another coordinate" << std::endl;
    std::cout << "y/n" << std::endl;

    std::cin >> continueHolder;
    if (continueHolder == "y" || continueHolder == "Y") {
      repeat = true;
    } else if (continueHolder == "n" || continueHolder == "N") {
      repeat = false;
    }
  }

  for (std::array < double, 3 > & array: worldPositions) {
    std::cout << "[";
    for (size_t i = 0; i < 3; i++) {
      std::cout << array[i];
      if (i != 2) {
        std::cout << "; ";
      }
    }
    std::cout << "]" << std::endl;

  }

  for (std::array < double, 3 > & array: robotPositions) {
    std::cout << "[";
    for (size_t i = 0; i < 3; i++) {
      std::cout << array[i];
      if (i != 2) {
        std::cout << "; ";
      }
    }
    std::cout << "]" << std::endl;
  }

  return 0;
}
