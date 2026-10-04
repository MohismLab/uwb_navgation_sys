#include "rclcpp/rclcpp.hpp"
#include "nlink_utils/init_serial.h"
#include "linktrack_aoa_node/linktrackaoainit.h"
#include <memory>
#include <thread>

extern asio::io_context g_ioc;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("linktrack_aoa");

    NProtocolExtracter protocol_extraction;

    auto serial = initSerial(node, [&protocol_extraction](const std::string & data)
    {
        protocol_extraction.AddNewData(data);
    });

    linktrack_aoa::Init aoaInit(node, &protocol_extraction, serial.get());

    std::thread io_thread([]()
    {
        g_ioc.run();
    });

    rclcpp::WallRate loop_rate(1000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    g_ioc.stop();
    io_thread.join();
    return 0;
}