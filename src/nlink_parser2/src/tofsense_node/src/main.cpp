#include <rclcpp/rclcpp.hpp>
#include "tofsense_node/tofsenseinit.h"
#include "nlink_utils/init_serial.h"
#include "nprotocol_extracter.h"
#include <memory>
#include <thread>

extern asio::io_context g_ioc;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tofsense_parser");
    auto extracter = std::make_shared<NProtocolExtracter>();
    auto serial = initSerial(node, [extracter](const std::string & data)
    {
        extracter->AddNewData(data);
    });
    tofsense::Init init(node, extracter.get(), serial.get());
    std::thread io_thread([]()
    {
        g_ioc.run();
    });
    
    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    g_ioc.stop();
    io_thread.join();

    return 0;
}