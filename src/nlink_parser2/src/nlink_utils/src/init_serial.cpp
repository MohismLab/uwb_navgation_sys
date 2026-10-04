#include "nlink_utils/init_serial.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

asio::io_context g_ioc;

std::unique_ptr<SerialPort> initSerial(
    const rclcpp::Node::SharedPtr &node,
    SerialPort::data_ready_cb_f data_callback)
{
    try {
        std::string port_name;
        int baud_rate;

        node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        node->declare_parameter<int>("baud_rate", 921600);
        node->get_parameter("port_name", port_name);
        node->get_parameter("baud_rate", baud_rate);

        RCLCPP_INFO(node->get_logger(), "try to open serial port with %s,%d", 
                   port_name.c_str(), baud_rate);

        auto serial = std::make_unique<SerialPort>(
            g_ioc, 
            port_name, 
            baud_rate,
            data_callback ? data_callback : [node](const std::string& data) {
                RCLCPP_DEBUG(node->get_logger(), "Received %zu bytes of data", data.size());
            }
        );

        RCLCPP_INFO(node->get_logger(), "Serial port opened successfully, waiting for data.");
        return serial;

    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Unhandled Exception: %s", e.what());
        throw;
    }
}

void runSerialIO() {
    g_ioc.run();
}

void stopSerialIO() {
    g_ioc.stop();
}