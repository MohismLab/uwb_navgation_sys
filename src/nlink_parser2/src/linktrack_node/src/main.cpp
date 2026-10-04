#include <rclcpp/rclcpp.hpp>
#include "linktrack_node/linktrackinit.h"
#include "nlink_utils/init_serial.h"
#include "nprotocol_extracter.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

extern asio::io_context g_ioc;

void printHexData(const std::string & data)
{
    if (!data.empty())
    {
        std::cout << "data received: ";
        for (size_t i = 0; i < data.size(); ++i)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2)
                      << int(uint8_t(data.at(i))) << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("linktrack_parser");

    NProtocolExtracter protocol_extraction;

    auto serial = initSerial(node, [&protocol_extraction](const std::string & data)
    {
        protocol_extraction.AddNewData(data);
        //printHexData(data);
    });

    linktrack::Init init(node, &protocol_extraction, serial.get());

    std::thread io_thread([]()
    {
        g_ioc.run();
    });

    rclcpp::Rate loop_rate(1000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    g_ioc.stop();
    io_thread.join();

    return EXIT_SUCCESS;
}