#include "linktrack_node/linktrackinit.h"

#include "nlink_parser2/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe0.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe3.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe4.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe5.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe6.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe7.hpp"
#include "nlink_parser2/msg/linktrack_tagframe0.hpp"
#include "serial/serial_port.hpp"
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include "nlink_utils/nutils.h"
#include "nlink_utils/linktrack_protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                        \
    for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) \
    {                                                                  \
        DEST[_CNT] = SRC[_CNT];                                        \
    }

namespace linktrack
{
std::map<NLinkProtocol*, std::shared_ptr<rclcpp::PublisherBase>> publishers_;

nlink_parser2::msg::LinktrackAnchorframe0 g_msg_anchorframe0;
nlink_parser2::msg::LinktrackTagframe0 g_msg_tagframe0;
nlink_parser2::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser2::msg::LinktrackNodeframe1 g_msg_nodeframe1;
nlink_parser2::msg::LinktrackNodeframe2 g_msg_nodeframe2;
nlink_parser2::msg::LinktrackNodeframe3 g_msg_nodeframe3;
nlink_parser2::msg::LinktrackNodeframe4 g_msg_nodeframe4;
nlink_parser2::msg::LinktrackNodeframe5 g_msg_nodeframe5;
nlink_parser2::msg::LinktrackNodeframe6 g_msg_nodeframe6;
nlink_parser2::msg::LinktrackNodeframe7 g_msg_nodeframe7;

static SerialPort* serial_ = nullptr;

// NLink_LinkTrack_Setting_Frame0 (NLink V1.4): 0x54 0x00, 128 bytes, checksum = sum of all
// previous bytes. Byte 2 bit0 selects read (1) / write (0), byte 36 the anchor group
// (0: A0-A9, 1: A10-A19, 2: A20-A29), then 10 x {x, y, z} int24 * 1000 from byte 37.
constexpr size_t kSettingFrameSize = 128;
constexpr size_t kSettingAnchorGroupIndex = 36;
constexpr size_t kSettingAnchorCoords = 37;
constexpr int kAnchorsPerGroup = 10;
// unused anchor slots read back as -8388.000 m (x/y/z = -8388000); not in the NLink manual,
// observed on firmware with 4 configured anchors
constexpr int32_t kAnchorUnset = -8388000;
// anything beyond 1 km is not an anchor coordinate
constexpr int32_t kAnchorMaxMm = 1000 * 1000;

class NLT_ProtocolSettingFrame0 : public NLinkProtocol
{
public:
    NLT_ProtocolSettingFrame0() : NLinkProtocol(true, kSettingFrameSize, {0x54, 0x00}) {}
    uint8_t frame[kSettingFrameSize];

protected:
    void UnpackFrameData(const uint8_t* data) override
    {
        std::copy(data, data + kSettingFrameSize, frame);
    }
};

static int32_t parseInt24(const uint8_t* p)
{
    int32_t v = p[0] | (p[1] << 8) | (p[2] << 16);
    return (v & 0x800000) ? v - 0x1000000 : v;
}

Init::Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter* protocol_extraction,
    SerialPort* serial) : node_(node)
{
    serial_ = serial;
    initDataTransmission();
    initTagPose();
    initAnchorSettings(protocol_extraction);
    initAnchorFrame0(protocol_extraction);
    initTagFrame0(protocol_extraction);
    initNodeFrame0(protocol_extraction);
    initNodeFrame1(protocol_extraction);
    initNodeFrame2(protocol_extraction);
    initNodeFrame3(protocol_extraction);
    initNodeFrame4(protocol_extraction);
    initNodeFrame5(protocol_extraction);
    initNodeFrame6(protocol_extraction);
    initNodeFrame7(protocol_extraction);
}

static void DTCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (serial_)
    {
        serial_->write(msg->data);
    }
}

void Init::initDataTransmission()
{
    dt_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "nlink_linktrack_data_transmission", 1000, DTCallback);
}

void Init::initTagPose()
{
    pose_topic_prefix_ = node_->declare_parameter<std::string>("pose_topic_prefix", "/uwb");
    tag_name_prefix_ = node_->declare_parameter<std::string>("tag_name_prefix", "rm_");
    pose_frame_id_ = node_->declare_parameter<std::string>("pose_frame_id", "world");
}

void Init::publishTagPose(uint8_t id, const float pos_3d[3])
{
    // publishers are created lazily, so the topic follows the tag id configured on the module
    auto it = tag_pose_pubs_.find(id);
    if (it == tag_pose_pubs_.end())
    {
        auto topic = pose_topic_prefix_ + "/" + tag_name_prefix_ + std::to_string(id) + "/pose";
        it = tag_pose_pubs_.emplace(id, node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10)).first;
        RCLCPP_INFO(node_->get_logger(), "tag %u pose -> %s", id, topic.c_str());
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = pose_frame_id_;
    msg.pose.position.x = pos_3d[0];
    msg.pose.position.y = pos_3d[1];
    msg.pose.position.z = pos_3d[2];
    // UWB gives position only, orientation is left as identity
    msg.pose.orientation.w = 1.0;
    it->second->publish(msg);
}

void Init::initAnchorSettings(NProtocolExtracter* protocol_extraction)
{
    if (!node_->declare_parameter<bool>("read_anchors", true))
    {
        return;
    }
    anchor_group_ = node_->declare_parameter<int>("anchor_group", 0);
    anchor_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/anchors", 10);

    auto protocol = new NLT_ProtocolSettingFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([this, protocol]
    {
        // 0x54 0x00 + an 8 bit checksum also turns up by chance inside the ~45 kB/s of
        // position frames: only take the frame while our request is pending, for the group we
        // asked, and with plausible coordinates
        const uint8_t* f = protocol->frame;
        int group = f[kSettingAnchorGroupIndex];
        if (!anchor_request_pending_ || group != anchor_group_)
        {
            return;
        }
        std::map<int, std::array<double, 3>> found;
        for (int i = 0; i < kAnchorsPerGroup; ++i)
        {
            const uint8_t* p = f + kSettingAnchorCoords + 9 * i;
            int32_t x = parseInt24(p), y = parseInt24(p + 3), z = parseInt24(p + 6);
            if (x == kAnchorUnset && y == kAnchorUnset && z == kAnchorUnset)
            {
                continue;
            }
            if (std::abs(x) > kAnchorMaxMm || std::abs(y) > kAnchorMaxMm || std::abs(z) > kAnchorMaxMm)
            {
                return;  // not a real settings frame
            }
            found[group * kAnchorsPerGroup + i] = {x / 1000.0, y / 1000.0, z / 1000.0};
        }
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        anchor_request_pending_ = false;
        anchors_ = found;
        for (const auto& [id, a] : anchors_)
        {
            RCLCPP_INFO(node_->get_logger(), "anchor A%d: (%.3f, %.3f, %.3f)", id, a[0], a[1], a[2]);
        }
        if (anchor_request_timer_)
        {
            anchor_request_timer_->cancel();
        }
    });

    // like NAssistant: ask the module for its settings until it answers
    anchor_request_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this] { requestAnchorSettings(); });
    anchor_publish_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this] { publishAnchors(); });
}

void Init::requestAnchorSettings()
{
    if (!serial_ || ++anchor_requests_ > 10)
    {
        RCLCPP_WARN(node_->get_logger(), "no Setting_Frame0 answer, anchors not shown");
        anchor_request_timer_->cancel();
        return;
    }
    // read request: only bit0 (read) of `mix` set; every other field 0xFF, which is
    // not a valid value for any of them, so nothing can be taken as a setting to write
    std::string req(kSettingFrameSize, static_cast<char>(0xFF));
    req[0] = 0x54;
    req[1] = 0x00;
    req[2] = 0x01;
    req[kSettingAnchorGroupIndex] = static_cast<char>(anchor_group_);
    uint8_t sum = 0;
    for (size_t i = 0; i < kSettingFrameSize - 1; ++i)
    {
        sum += static_cast<uint8_t>(req[i]);
    }
    req[kSettingFrameSize - 1] = static_cast<char>(sum);
    anchor_request_pending_ = true;
    serial_->write(req);
    RCLCPP_INFO(node_->get_logger(), "requested anchor coordinates (group %d)", anchor_group_);
}

void Init::publishAnchors()
{
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = node_->now();
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    for (const auto& [id, a] : anchors_)
    {
        visualization_msgs::msg::Marker model;
        model.header.frame_id = pose_frame_id_;
        model.header.stamp = stamp;
        model.ns = "uwb_anchors";
        model.id = 2 * id;
        // origin of the model = antenna = anchor coordinate
        model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        model.mesh_resource = "package://nlink_parser2/meshes/uwb_anchor.dae";
        model.mesh_use_embedded_materials = true;
        model.pose.position.x = a[0];
        model.pose.position.y = a[1];
        model.pose.position.z = a[2];
        model.pose.orientation.w = 1.0;
        model.scale.x = model.scale.y = model.scale.z = 1.0;
        // republished every second; anything not refreshed (e.g. an anchor removed) fades out
        model.lifetime = rclcpp::Duration::from_seconds(3.0);
        markers.markers.push_back(model);

        visualization_msgs::msg::Marker label = model;
        label.id = 2 * id + 1;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.mesh_resource.clear();
        label.pose.position.z = a[2] + 0.35;
        label.scale.z = 0.3;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0;
        label.text = "A" + std::to_string(id);
        markers.markers.push_back(label);
    }
    if (!markers.markers.empty())
    {
        anchor_pub_->publish(markers);
    }
}

void Init::initAnchorFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolAnchorFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_anchorframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackAnchorframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        auto data = nlt_anchorframe0_.result;
        linktrack::g_msg_anchorframe0.role = data.role;
        linktrack::g_msg_anchorframe0.id = data.id;
        linktrack::g_msg_anchorframe0.voltage = data.voltage;
        linktrack::g_msg_anchorframe0.local_time = data.local_time;
        linktrack::g_msg_anchorframe0.system_time = data.system_time;
        auto& msg_nodes = linktrack::g_msg_anchorframe0.nodes;
        msg_nodes.clear();
        decltype(linktrack::g_msg_anchorframe0.nodes)::value_type msg_node;
        for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i)
        {
            auto node = data.nodes[i];
            msg_node.role = node->role;
            msg_node.id = node->id;
            ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
            ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr);
            msg_nodes.push_back(msg_node);
            publishTagPose(node->id, node->pos_3d);
        }
        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackAnchorframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_anchorframe0);
        }
    });
}

void Init::initTagFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolTagFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_tagframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackTagframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_tagframe0.result;
        auto& msg_data = g_msg_tagframe0;

        linktrack::g_msg_tagframe0.role = data.role;
        linktrack::g_msg_tagframe0.id = data.id;
        linktrack::g_msg_tagframe0.local_time = data.local_time;
        linktrack::g_msg_tagframe0.system_time = data.system_time;
        linktrack::g_msg_tagframe0.voltage = data.voltage;

        ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d);
        ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d);
        ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d);
        ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr);
        ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d);
        ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d);
        ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d);
        ARRAY_ASSIGN(msg_data.quaternion, data.quaternion);
        publishTagPose(data.id, data.pos_3d);

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackTagframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_tagframe0);
        }
    });
}

void Init::initNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        const auto& data = g_nlt_nodeframe0.result;
        auto& msg_data = linktrack::g_msg_nodeframe0;
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

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe0);
        }
    });
}

void Init::initNodeFrame1(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame1;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe1";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe1>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe1.result;
        auto& msg_data = linktrack::g_msg_nodeframe1;
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
            ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe1>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe1);
        }
    });
}

void Init::initNodeFrame2(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame2;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe2";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe2>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe2.result;
        auto& msg_data = linktrack::g_msg_nodeframe2;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe2.role = data.role;
        linktrack::g_msg_nodeframe2.id = data.id;
        linktrack::g_msg_nodeframe2.local_time = data.local_time;
        linktrack::g_msg_nodeframe2.system_time = data.system_time;
        linktrack::g_msg_nodeframe2.voltage = data.voltage;
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.pos_3d, data.pos_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.eop_3d, data.eop_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.vel_3d, data.vel_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.imu_gyro_3d, data.imu_gyro_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.imu_acc_3d, data.imu_acc_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.angle_3d, data.angle_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.quaternion, data.quaternion);

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe2>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe2);
        }
    });
}

void Init::initNodeFrame3(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame3;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe3";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe3>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe3.result;
        auto& msg_data = linktrack::g_msg_nodeframe3;
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
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe3>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe3);
        }
    });
}

void Init::initNodeFrame4(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame4;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe4";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe4>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe4.result;

        linktrack::g_msg_nodeframe4.role = data.role;
        linktrack::g_msg_nodeframe4.id = data.id;
        linktrack::g_msg_nodeframe4.local_time = data.local_time;
        linktrack::g_msg_nodeframe4.system_time = data.system_time;
        linktrack::g_msg_nodeframe4.voltage = data.voltage;

        linktrack::g_msg_nodeframe4.tags.resize(data.tag_count);
        for (int i = 0; i < data.tag_count; ++i)
        {
            auto& msg_tag = linktrack::g_msg_nodeframe4.tags[i];
            auto tag = data.tags[i];
            msg_tag.id = tag->id;
            msg_tag.voltage = tag->voltage;
            msg_tag.anchors.resize(tag->anchor_count);
            for (int j = 0; j < tag->anchor_count; ++j)
            {
                auto& msg_anchor = msg_tag.anchors[j];
                auto anchor = tag->anchors[j];
                msg_anchor.id = anchor->id;
                msg_anchor.dis = anchor->dis;
            }
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe4>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe4);
        }
    });
}

void Init::initNodeFrame5(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame5;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe5";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe5>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe5.result;
        auto& msg_data = linktrack::g_msg_nodeframe5;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe5.role = data.role;
        linktrack::g_msg_nodeframe5.id = data.id;
        linktrack::g_msg_nodeframe5.local_time = data.local_time;
        linktrack::g_msg_nodeframe5.system_time = data.system_time;
        linktrack::g_msg_nodeframe5.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe5>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe5);
        }
    });
}

void Init::initNodeFrame6(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame6;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe6";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe6>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe6.result;
        auto& msg_data = linktrack::g_msg_nodeframe6;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe6.role = data.role;
        linktrack::g_msg_nodeframe6.id = data.id;

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

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe6>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe6);
        }
    });
}

void Init::initNodeFrame7(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame7;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe7";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe7>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        const auto& data = g_nlt_nodeframe7.result;
        auto& msg_data = linktrack::g_msg_nodeframe7;
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
            msg_node.angle0 = node->angle0;
            msg_node.angle1 = node->angle1;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe7>>(linktrack::publishers_[protocol]);

        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe7);
        }
    });
}

}  /* namespace linktrack */