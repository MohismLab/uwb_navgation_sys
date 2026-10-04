#include "linktrack_aoa_node/linktrackaoainit.h"
#include <std_msgs/msg/string.hpp>
#include "nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_linktrack_nodeframe0.h"
#include "nlink_utils/nutils.h"
#include "nlink_utils/linktrack_protocols.h"
#include "serial/serial_port.hpp"

namespace linktrack_aoa
{
std::map<NLinkProtocol*, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
nlink_parser2::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser2::msg::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;
static SerialPort* g_serial = nullptr;

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength
{
public:
    NLTAoa_ProtocolNodeFrame0()
        : NLinkProtocolVLength(
            true,
            g_nltaoa_nodeframe0.fixed_part_size,
            {g_nltaoa_nodeframe0.frame_header, g_nltaoa_nodeframe0.function_mark})
    {
    }

protected:
    void UnpackFrameData(const uint8_t* data) override
    {
        g_nltaoa_nodeframe0.UnpackData(data, length());
    }
};

Init::Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter* protocol_extraction,
    SerialPort* serial)
    : node_(node)
{
    g_serial = serial;
    initDataTransmission();
    initNodeFrame0(protocol_extraction);
    initAoaNodeFrame0(protocol_extraction);
}

static void DTCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (g_serial)
    {
        g_serial->write(msg->data);
    }
}

void Init::initDataTransmission()
{
    dt_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "nlink_linktrack_data_transmission",
        1000,
        DTCallback);
}

void Init::initNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe0";
            publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe0.result;
        auto& msg_data = g_msg_nodeframe0;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];

            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.data.resize(node->data_length);
            memcpy(msg_node.data.data(), node->data, node->data_length);
        }

        auto pub = std::dynamic_pointer_cast<
            rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe0>>(publishers_[protocol]);

        if (pub)
        {
            pub->publish(msg_data);
        }
    });
}

void Init::initAoaNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLTAoa_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!publishers_[protocol])
        {
            auto topic = "nlink_linktrack_aoa_nodeframe0";
            publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackAoaNodeframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nltaoa_nodeframe0.result;
        auto& msg_data = g_msg_aoa_nodeframe0;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;
        msg_data.local_time = data.local_time;
        msg_data.system_time = data.system_time;
        msg_data.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];

            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.angle = node->angle;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto pub = std::dynamic_pointer_cast<
            rclcpp::Publisher<nlink_parser2::msg::LinktrackAoaNodeframe0>>(publishers_[protocol]);

        if (pub)
        {
            pub->publish(msg_data);
        }
    });
}

}  /* namespace linktrack_aoa */