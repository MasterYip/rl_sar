#include "ahrs_interface.h"
#include <iostream>
#include <cstring>
#include <chrono>

// copy minimal frame/len/type constants (kept local to avoid ROS dependency)
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48

using namespace FDILink;

AHRSInterface::AHRSInterface(const std::string &port, int baud, int timeout_ms)
  : running_(false), serial_(port, (uint32_t)baud, serial::Timeout::simpleTimeout(timeout_ms)),
    port_(port), baud_(baud), timeout_ms_(timeout_ms)
{
}

AHRSInterface::~AHRSInterface()
{
  stop();
}

bool AHRSInterface::start()
{
  try {
    if (!serial_.isOpen()) serial_.open();
  } catch (std::exception &e) {
    std::cerr << "AHRSInterface: failed to open serial: " << e.what() << std::endl;
    return false;
  }
  if (!serial_.isOpen()) {
    std::cerr << "AHRSInterface: serial not open" << std::endl;
    return false;
  }

  running_ = true;
  thread_ = std::thread(&AHRSInterface::readLoop, this);
  return true;
}

void AHRSInterface::stop()
{
  if (running_) {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }
  if (serial_.isOpen()) serial_.close();
}

void AHRSInterface::setImuCallback(ImuCallback cb)
{
  imu_cb_ = cb;
}

void AHRSInterface::readLoop()
{
  while (running_)
  {
    if (!serial_.isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    uint8_t check_head[1] = {0xff};
    size_t head_s = serial_.read(check_head, 1);
    if (head_s != 1) continue;
    if (check_head[0] != FRAME_HEAD) continue;

    uint8_t head_type[1] = {0xff};
    if (serial_.read(head_type, 1) != 1) continue;

    uint8_t check_len[1] = {0xff};
    if (serial_.read(check_len, 1) != 1) continue;

    // read sn, crc8, crc16
    uint8_t check_sn[1] = {0xff};
    if (serial_.read(check_sn, 1) != 1) continue;
    uint8_t head_crc8[1] = {0xff};
    if (serial_.read(head_crc8, 1) != 1) continue;
    uint8_t head_crc16_H[1] = {0xff};
    uint8_t head_crc16_L[1] = {0xff};
    if (serial_.read(head_crc16_H, 1) != 1) continue;
    if (serial_.read(head_crc16_L, 1) != 1) continue;

    // build header bytes for CRC8 check (start,type,size,sn)
    uint8_t hdr4[4];
    hdr4[0] = check_head[0];
    hdr4[1] = head_type[0];
    hdr4[2] = check_len[0];
    hdr4[3] = check_sn[0];

    // handle IMU frame
    if (head_type[0] == TYPE_IMU)
    {
      // check header crc8
      uint8_t crc8 = CRC8_Table(hdr4, 4);
      if (crc8 != head_crc8[0]) continue;

      // read imu payload (IMU_LEN + frame_end)
      if (serial_.read(last_imu_.read_buf.read_msg, IMU_LEN + 1) != (IMU_LEN + 1)) continue;

      uint16_t head_crc16 = (uint16_t)head_crc16_L[0] + ((uint16_t)head_crc16_H[0] << 8);
      uint16_t calc_crc16 = CRC16_Table(last_imu_.frame.data.data_buff, IMU_LEN);
      if (head_crc16 != calc_crc16) continue;
      if (last_imu_.frame.frame_end != FRAME_END) continue;

      {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        last_imu_ = last_imu_;// already in read_buf
        last_imu_valid_ = true;
        // build combined ImuData and publish via callback
        ImuData out;
        if (last_ahrs_valid_) {
          out.qw = last_ahrs_.frame.data.data_pack.Qw;
          out.qx = last_ahrs_.frame.data.data_pack.Qx;
          out.qy = last_ahrs_.frame.data.data_pack.Qy;
          out.qz = last_ahrs_.frame.data.data_pack.Qz;
          out.gx = last_ahrs_.frame.data.data_pack.RollSpeed;
          out.gy = last_ahrs_.frame.data.data_pack.PitchSpeed;
          out.gz = last_ahrs_.frame.data.data_pack.HeadingSpeed;
        }
        out.ax = last_imu_.frame.data.data_pack.accelerometer_x;
        out.ay = last_imu_.frame.data.data_pack.accelerometer_y;
        out.az = last_imu_.frame.data.data_pack.accelerometer_z;
        if (imu_cb_) imu_cb_(out);
      }
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      uint8_t hdr4b[4];
      hdr4b[0] = check_head[0]; hdr4b[1] = head_type[0]; hdr4b[2] = check_len[0]; hdr4b[3] = check_sn[0];
      uint8_t crc8 = CRC8_Table(hdr4b, 4);
      if (crc8 != head_crc8[0]) continue;

      if (serial_.read(last_ahrs_.read_buf.read_msg, AHRS_LEN + 1) != (AHRS_LEN + 1)) continue;
      uint16_t head_crc16 = (uint16_t)head_crc16_L[0] + ((uint16_t)head_crc16_H[0] << 8);
      uint16_t calc_crc16 = CRC16_Table(last_ahrs_.frame.data.data_buff, AHRS_LEN);
      if (head_crc16 != calc_crc16) continue;
      if (last_ahrs_.frame.frame_end != FRAME_END) continue;

      {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        last_ahrs_valid_ = true;
        ImuData out;
        out.qw = last_ahrs_.frame.data.data_pack.Qw;
        out.qx = last_ahrs_.frame.data.data_pack.Qx;
        out.qy = last_ahrs_.frame.data.data_pack.Qy;
        out.qz = last_ahrs_.frame.data.data_pack.Qz;
        out.gx = last_ahrs_.frame.data.data_pack.RollSpeed;
        out.gy = last_ahrs_.frame.data.data_pack.PitchSpeed;
        out.gz = last_ahrs_.frame.data.data_pack.HeadingSpeed;
        if (last_imu_valid_) {
          out.ax = last_imu_.frame.data.data_pack.accelerometer_x;
          out.ay = last_imu_.frame.data.data_pack.accelerometer_y;
          out.az = last_imu_.frame.data.data_pack.accelerometer_z;
        }
        if (imu_cb_) imu_cb_(out);
      }
    }

    // loop continues
  }
}
