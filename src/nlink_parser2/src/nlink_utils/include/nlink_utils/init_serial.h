#ifndef INIT_SERIAL_H
#define INIT_SERIAL_H

#include "serial/serial_port.hpp" 
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace asio {
    class io_context;
}
extern asio::io_context g_ioc;  

std::unique_ptr<SerialPort> initSerial(
    const rclcpp::Node::SharedPtr &node,
    SerialPort::data_ready_cb_f data_callback = nullptr
);

void runSerialIO();

void stopSerialIO();

#endif // INIT_SERIAL_H