/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 4 //可该最大从机数

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include "config.h"
#include "motor_control.h"
#include "ethercat.h"
#include "sys/time.h"
#include "motor_data.h"

// === 添加：线程管理类 ===
class EtherCATThreadManager {
private:
    std::thread sendThread;
    std::thread receiveThread;
    std::atomic<bool> running{false};

    std::mutex ethercat_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> send_complete_{false};

public:
    void startThreads();
    void stopThreads();
    // void updateMotorCommand(int slave, int motor, const OD_Motor_Msg& cmd);
    // OD_Motor_Msg getMotorStatus(int slave, int motor) const;
    // EtherCAT_Msg getRawRxData(int slave) const;
    void sendThreadFunc();
    void receiveThreadFunc();
};

extern EtherCATThreadManager ethercatManager;

extern MotorData motorData;
// === 添加结束 ===

#ifdef __cplusplus
extern "C" {
#endif



void EtherCAT_Transmit(EtherCAT_Msg* MasterCommand);
void EtherCAT_Init(char *ifname);
void EtherCAT_Run();
void EtherCAT_Command_Set();
void startRun();

#ifdef __cplusplus
};
#endif

#endif // PROJECT_RT_ETHERCAT_H