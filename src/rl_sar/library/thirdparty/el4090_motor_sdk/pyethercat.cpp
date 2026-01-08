#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "transmit.h"
#include "motor_data.h"
#include "queue.h"

namespace py = pybind11;

extern MotorData motorData;

class PyEtherCATControl {
public:
    bool init(const std::string& interface) {
        EtherCAT_Init(const_cast<char*>(interface.c_str()));
        return (ec_slavecount > 0);
    }

    void start() {
        ethercatManager.startThreads();
    }

    void stop() {
        ethercatManager.stopThreads();
    }

    void set_motor_speed(int slave, uint8_t passage, uint16_t motor_id, float spd, uint16_t cur,
                        uint8_t ack_status) {
        EtherCAT_Msg msg = motorData.getTxMsg(slave);
        ::set_motor_speed(&msg, passage, motor_id, spd, cur, ack_status);
        motorData.setTxMsg(slave, msg);
    }

    void set_motor_position(int slave, uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur,
                            uint8_t ack_status) {
        EtherCAT_Msg msg = motorData.getTxMsg(slave);
        ::set_motor_position(&msg, passage, motor_id, pos, spd, cur, ack_status);
        motorData.setTxMsg(slave, msg);
    }

    void stop_motor(int slave, uint8_t passage, uint16_t motor_id, uint16_t cur, uint8_t ack_status) {
        EtherCAT_Msg msg = motorData.getTxMsg(slave);
        ::set_motor_speed(&msg, passage, motor_id, 0.0f, cur, ack_status);
        motorData.setTxMsg(slave, msg);
    }

    void motor_mixed_control(int slave, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos,
                            float spd, float tor) {
        EtherCAT_Msg msg = motorData.getTxMsg(slave);
        ::send_motor_ctrl_cmd(&msg, passage, motor_id, kp, kd, pos, spd, tor);
        motorData.setTxMsg(slave, msg);
    }

    std::vector<float> get_motor_status(int slave, int passage, int ack_status) const {
        OD_Motor_Msg msg = motorData.getRxMotorMsg(slave, passage);
         if (ack_status == 1) {
        return {
            static_cast<float>(msg.motor_id),
            msg.angle_actual_rad,
            msg.speed_actual_rad,
            msg.current_actual_float
        };
        } else if (ack_status == 2) {
        return {
            static_cast<float>(msg.motor_id),
            msg.angle_actual_float,
            msg.speed_actual_float,
            msg.current_actual_float
        };
        } else {
        return {
            static_cast<float>(msg.motor_id),
            msg.angle_actual_float,
            msg.speed_actual_float,
            msg.current_actual_float
        };
        }
    }

    std::vector<std::vector<float>> get_all_motor_status(int ack_status) const {
        std::vector<std::vector<float>> result;
        for (int slave = 0; slave < SLAVE_NUMBER; ++slave) {
            for (int motor = 0; motor < 6; ++motor) {
                OD_Motor_Msg msg = motorData.getRxMotorMsg(slave, motor);
                if(ack_status == 1 ){
                    result.push_back({
                        static_cast<float>(slave),
                        static_cast<float>(motor),
                        static_cast<float>(msg.motor_id),
                        msg.angle_actual_rad,
                        msg.speed_actual_rad,
                        msg.current_actual_float,
                    });
                }
                else if(ack_status ==2){
                    result.push_back({
                        static_cast<float>(slave),
                        static_cast<float>(motor),
                        static_cast<float>(msg.motor_id),
                        msg.angle_actual_float,
                        msg.speed_actual_float,
                        msg.current_actual_float,
                    });
                }
                else{
                    result.push_back({
                        static_cast<float>(slave),
                        static_cast<float>(motor),
                        static_cast<float>(msg.motor_id),
                        msg.angle_actual_float,
                        msg.speed_actual_float,
                        msg.current_actual_float,
                    });
                }
            }
        }
        return result;
    }

    int get_slave_count() const {
        return ec_slavecount;
    }
};

PYBIND11_MODULE(pyethercat, m) {
    m.doc() = "Python interface for EtherCAT motor control";

    py::class_<PyEtherCATControl>(m, "EtherCATControl")
        .def(py::init<>())
        .def("init", &PyEtherCATControl::init,
             py::arg("interface"),
             "Initialize EtherCAT on the given network interface (e.g., 'eth0')")
        .def("start", &PyEtherCATControl::start,
             "Start EtherCAT communication threads")
        .def("stop", &PyEtherCATControl::stop,
             "Stop EtherCAT communication threads")
        .def("set_motor_speed", &PyEtherCATControl::set_motor_speed,
             py::arg("slave"), py::arg("passage"), py::arg("motor_id"),
             py::arg("spd"), py::arg("cur") = 40, py::arg("ack_status"),
             "Set speed mode command for a specific motor.\n"
             "  slave: slave index (0-based)\n"
             "  passage: CAN channel (1~6)\n"
             "  motor_id: unique motor ID\n"
             "  spd: target speed (rad/s or RPM depending on firmware)\n"
             "  cur: current limit (default=40)\n"
             "  ack_status: feedback level (default=2 for full status)")
        .def("set_motor_position", &PyEtherCATControl::set_motor_position,
             py::arg("slave"), py::arg("passage"), py::arg("motor_id"),
             py::arg("pos"), py::arg("spd") = 30, py::arg("cur") = 40, py::arg("ack_status"),
             "Set position mode command for a specific motor.\n"
             "  pos: target position (degrees or radians)\n"
             "  spd: max speed during move (default=30)\n"
             "  cur: current limit (default=40)")
        .def("stop_motor", &PyEtherCATControl::stop_motor,
             py::arg("slave"), py::arg("passage"), py::arg("motor_id"),
             py::arg("cur") = 40, py::arg("ack_status") = 2,
             "Stop motor by setting speed to zero.")
        .def("motor_mixed_control", &PyEtherCATControl::motor_mixed_control,
             py::arg("slave"), py::arg("passage"), py::arg("motor_id"),
             py::arg("kp"), py::arg("kd"), py::arg("pos"), py::arg("spd"), py::arg("tor"),
             "Send advanced mixed (impedance/trajectory) control command.")
        .def("get_motor_status", &PyEtherCATControl::get_motor_status, "Get status of a specific motor",
             py::arg("slave"), py::arg("motor"), py::arg("ack_status"))
        .def("get_all_motor_status", &PyEtherCATControl::get_all_motor_status, "Get status of all motors", py::arg("ack_status"))
        .def("get_slave_count", &PyEtherCATControl::get_slave_count,
             "Get number of detected EtherCAT slaves");

    m.attr("SLAVE_NUMBER") = SLAVE_NUMBER;
    m.attr("MOTOR_PER_SLAVE") = 6;
}
