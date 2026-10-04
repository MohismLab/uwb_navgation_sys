#include <rclcpp/rclcpp.hpp>
#include "tofsensem_node/tofsenseminit.h"
#include "nlink_utils/init_serial.h"
#include "nlink_tofsensem_frame0.h"
#include "nlink_utils.h"
#include "nprotocol_extracter.h"
#include <chrono>
#include <thread>
#include <memory>

extern asio::io_context g_ioc;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tofsensem_parser");
    auto extracter = std::make_shared<NProtocolExtracter>();
    auto serial = initSerial(node, [extracter](const std::string & data)
    {
        extracter->AddNewData(data);
    });
    tofsensem::Init init(extracter, node);
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