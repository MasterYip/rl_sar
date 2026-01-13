#include <iostream>
#include <chrono>
#include <thread>
#include "../include/ahrs_interface.h"

int main(int argc, char** argv)
{
  std::string port = "/dev/ttyTHS1";
  int baud = 115200;
  if (argc >= 2) port = argv[1];
  if (argc >= 3) baud = std::stoi(argv[2]);

  FDILink::AHRSInterface ahrs(port, baud);
  ahrs.setImuCallback([](const FDILink::ImuData &d){
    std::cout << "IMU: Q=" << d.qw << "," << d.qx << "," << d.qy << "," << d.qz
              << " | G=" << d.gx << "," << d.gy << "," << d.gz
              << " | A=" << d.ax << "," << d.ay << "," << d.az << std::endl;
  });

  if (!ahrs.start()) {
    std::cerr << "Failed to start AHRS interface" << std::endl;
    return 1;
  }

  std::cout << "Started AHRSInterface on " << port << " @ " << baud << " bps. Press Ctrl+C to quit." << std::endl;
  // keep running
  while (true) std::this_thread::sleep_for(std::chrono::seconds(1));

  return 0;
}
