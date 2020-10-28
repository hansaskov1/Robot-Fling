#include <stdio.h>
#include <iostream>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <thread>
#include <string>
#include <ostream>
#include <fstream>

int main() {
  std::vector < std::array < double, 3 >> worldPositions;
  std::vector < std::array < double, 3 >> robotPositions;

  std::string urIp = "192.168.100.53";
 //  std::string urIp = "127.0.0.1";
  ur_rtde::RTDEReceiveInterface rtde_recieve(urIp);

  bool repeat = true;
  std::string continueHolder;

  while (repeat) {

    std::array < double, 3 > worldPosition {0};
    std::array < double, 3 > robotPosition {0};

    std::cout << "Move tcp to position" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::cout << "write \"x y z\" coordinate" << std::endl;

    std::cin >> worldPosition[0] * 0.01;
    std::cin >> worldPosition[1] * 0.01;
    std::cin >> worldPosition[2] * 0.01;

    worldPositions.push_back(worldPosition);

    std::vector < double > tcpPose = rtde_recieve.getActualTCPPose();
    robotPosition[0] = tcpPose[0];
    robotPosition[1] = tcpPose[1];
    robotPosition[2] = tcpPose[2];

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

  std::ofstream file;
  file.open("calibration_points.txt");

  for (std::array < double, 3 > & array: worldPositions) {
    file << "[";
    for (size_t i = 0; i < 3; i++) {
      file << array[i];
      if (i != 2) {
        file << "; ";
      }
    }
    file << "]" << "\n";

  }

  for (std::array < double, 3 > & array: robotPositions) {
    file << "[";
    for (size_t i = 0; i < 3; i++) {
      file << array[i];
      if (i != 2) {
        file << "; ";
      }
    }
    file << "]" << "\n";
  }

  file.close();


  return 0;
}
