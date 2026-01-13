#ifndef AHRS_INTERFACE_H
#define AHRS_INTERFACE_H

#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <serial/serial.h>
#include "fdilink_data_struct.h"
#include "crc_table.h"

namespace FDILink {

struct ImuData {
  double qw=0, qx=0, qy=0, qz=0; // orientation
  float gx=0, gy=0, gz=0; // angular velocity (rad/s)
  float ax=0, ay=0, az=0; // linear acceleration (m/s^2)
};

class AHRSInterface {
public:
  using ImuCallback = std::function<void(const ImuData&)>;

  AHRSInterface(const std::string &port = "/dev/ttyTHS1", int baud = 115200, int timeout_ms = 20);
  ~AHRSInterface();

  bool start();
  void stop();
  void setImuCallback(ImuCallback cb);

private:
  void readLoop();

  std::thread thread_;
  std::atomic<bool> running_;
  serial::Serial serial_;
  ImuCallback imu_cb_;
  std::string port_;
  int baud_;
  int timeout_ms_;

  // latest frames
  FDILink::imu_frame_read last_imu_;
  FDILink::ahrs_frame_read last_ahrs_;
  bool last_imu_valid_ = false;
  bool last_ahrs_valid_ = false;
  std::mutex frame_mutex_;
};

} // namespace FDILink

#endif // AHRS_INTERFACE_H
