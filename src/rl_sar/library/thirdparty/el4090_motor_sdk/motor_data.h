#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

#include "ethercat.h"
#include "motor_control.h"
#include <shared_mutex>
#include <mutex>

#define SLAVE_NUMBER 4

class MotorData {
private:
    EtherCAT_Msg tx_msgs[SLAVE_NUMBER];
    EtherCAT_Msg rx_msgs[SLAVE_NUMBER];
    OD_Motor_Msg rx_motor_msg[SLAVE_NUMBER][6];
    mutable std::shared_mutex tx_mutexes[SLAVE_NUMBER];
    mutable std::shared_mutex rx_mutexes[SLAVE_NUMBER];
    mutable std::shared_mutex motor_msg_mutexes[SLAVE_NUMBER];
    
public:
    // 读取方法
    const EtherCAT_Msg& getTxMsg(int slave) const {
        std::shared_lock<std::shared_mutex> lock(tx_mutexes[slave]);
        return tx_msgs[slave];
    }
    
    const EtherCAT_Msg& getRxMsg(int slave) const {
        std::shared_lock<std::shared_mutex> lock(rx_mutexes[slave]);
        return rx_msgs[slave];
    }
    
    OD_Motor_Msg getRxMotorMsg(int slave, int passage) const {
        std::shared_lock<std::shared_mutex> lock(motor_msg_mutexes[slave]);
        return rx_motor_msg[slave][passage-1];
    }
    
    // 写入方法
    void setTxMsg(int slave, const EtherCAT_Msg& msg) {
        std::unique_lock<std::shared_mutex> lock(tx_mutexes[slave]);
        tx_msgs[slave] = msg;
    }
    
    void updateRxMsg(int slave, const EtherCAT_Msg& rx_msg) {
        std::unique_lock<std::shared_mutex> lock(rx_mutexes[slave]);
        rx_msgs[slave] = rx_msg;
    }
    
    void updateRxMotorMsg(int slave, const OD_Motor_Msg motor_msg[6]) {
        std::unique_lock<std::shared_mutex> lock(motor_msg_mutexes[slave]);
        for(int i = 0; i < 6; ++i) {
            // rx_motor_msg[slave][i] = motor_msg[i];
            if (motor_msg[i].motor_id != 0) {
                rx_motor_msg[slave][i] = motor_msg[i];
            }
        }
    }
};

#endif