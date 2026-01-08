//
// Created by bismarck on 11/18/22.
//

#ifndef SOEM_QUEUE_H
#define SOEM_QUEUE_H

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>

// extern "C" {
#include "transmit.h"
// };

using boost::lockfree::spsc_queue, boost::lockfree::capacity;
typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;

typedef struct
{
    int passage;
    Motor_Msg motor;
} Queue_Msg;

typedef std::shared_ptr<Queue_Msg> Queue_Msg_ptr;

// === 添加：反馈消息结构体 ===

typedef struct
{
    int slave_id;
    int motor_id;
    OD_Motor_Msg motor_status;
} Feedback_Msg;

typedef std::shared_ptr<Feedback_Msg> Feedback_Msg_ptr;
// === 添加结束 ===

extern spsc_queue<Queue_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];

// === 添加：反馈消息队列声明 ===
extern spsc_queue<Feedback_Msg_ptr, capacity<20>> feedback_messages[SLAVE_NUMBER][6];
// === 添加结束 ===

extern std::atomic<bool> running;
extern std::thread runThread;

#endif //SOEM_QUEUE_H
