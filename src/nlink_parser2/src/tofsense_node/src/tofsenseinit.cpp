#include "tofsense_node/tofsenseinit.h"
#include "nlink_utils/nlink_protocol.h"
#include "nlink_tofsense_frame0.h"
#include "nlink_utils.h"
#include "nlink_utils/nutils.h"
#include "serial/serial_port.hpp"

class NTS_ProtocolFrame0 : public NLinkProtocol
{
public:
    NTS_ProtocolFrame0();

protected:
    void UnpackFrameData(const uint8_t * data) override;
};

NTS_ProtocolFrame0::NTS_ProtocolFrame0()
    : NLinkProtocol(
        true,
        g_nts_frame0.fixed_part_size,
        {g_nts_frame0.frame_header, g_nts_frame0.function_mark})
{
}

void NTS_ProtocolFrame0::UnpackFrameData(const uint8_t * data)
{
    g_nts_frame0.UnpackData(data, length());
}

namespace tofsense
{
static nlink_parser2::msg::TofsenseFrame0 g_msg_frame0;

#pragma pack(push, 1)
struct
{
    char header[2]{0x57, 0x10};
    uint8_t reserved0[2]{0xff, 0xff};
    uint8_t id{};
    uint8_t reserved1[2]{0xff, 0xff};
    uint8_t checkSum{};
} g_command_read;
#pragma pack(pop)

Init::Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter * protocol_extraction,
    SerialPort * serial)
    : node_(node), serial_(serial)
{
    is_inquire_mode_ = serial_ ? node_->declare_parameter<bool>("inquire_mode", true) : false;

    InitFrame0(protocol_extraction);
}

void Init::InitFrame0(NProtocolExtracter * protocol_extraction)
{
    auto protocol_frame0_ = new NTS_ProtocolFrame0;
    protocol_extraction->AddProtocol(protocol_frame0_);

    protocol_frame0_->SetHandleDataCallback([this, protocol_frame0_]()
    {
        if (!publishers_frame0_[protocol_frame0_])
        {
            if (is_inquire_mode_)
            {
                auto topic = "nlink_tofsense_cascade";
                publisher_cascade_ = node_->create_publisher<nlink_parser2::msg::TofsenseCascade>(
                    topic, 50);
                TopicAdvertisedTip(topic);  
            }
            else
            {
                auto topic = "nlink_tofsense_frame0";
                publishers_frame0_[protocol_frame0_] = node_->create_publisher<nlink_parser2::msg::TofsenseFrame0>(
                    topic, 50);
                TopicAdvertisedTip(topic);  
            }
        }

        const auto & data = g_nts_frame0.result;

        g_msg_frame0.id = data.id;
        g_msg_frame0.system_time = data.system_time;
        g_msg_frame0.dis = data.dis;
        g_msg_frame0.dis_status = data.dis_status;
        g_msg_frame0.signal_strength = data.signal_strength;
        g_msg_frame0.range_precision = data.range_precision;

        if (is_inquire_mode_)
        {
            frame0_map_[data.id] = g_msg_frame0;
        }
        else
        {
            publishers_frame0_[protocol_frame0_]->publish(g_msg_frame0);
        }
    });

    if (is_inquire_mode_ && serial_)
    {
        timer_scan_ = node_->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency_),
            [this]()
            {
                frame0_map_.clear();
                node_index_ = 0;
                timer_read_->reset();
            });

        timer_read_ = node_->create_wall_timer(
            std::chrono::milliseconds(6),
            [this]()
            {
                if (node_index_ >= 8)
                {
                    if (!frame0_map_.empty())
                    {
                        nlink_parser2::msg::TofsenseCascade msg;
                        for (const auto & kv : frame0_map_)
                        {
                            msg.nodes.push_back(kv.second);
                        }
                        publisher_cascade_->publish(msg);
                    }
                    timer_read_->cancel();
                }
                else
                {
                    g_command_read.id = node_index_;
                    auto data = reinterpret_cast<uint8_t *>(&g_command_read);
                    NLink_UpdateCheckSum(data, sizeof(g_command_read));

                    if (serial_)
                    {
                        std::string write_data(reinterpret_cast<const char *>(data), sizeof(g_command_read));
                        serial_->write(write_data);
                    }
                    ++node_index_;
                }
            });
        timer_read_->cancel();  
    }
}

}  /* namespace tofsense */