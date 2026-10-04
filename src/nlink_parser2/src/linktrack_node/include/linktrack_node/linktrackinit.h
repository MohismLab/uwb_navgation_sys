#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include "rclcpp/rclcpp.hpp"
#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <unordered_map>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nlink_utils.h"
#include "nprotocol_extracter.h"
#include "nlink_utils/nlink_protocol.h"

class SerialPort;
class NProtocolExtracter;

namespace linktrack
{
extern std::map<NLinkProtocol *, std::shared_ptr<rclcpp::PublisherBase>> publishers_;

class Init
{
public:
    explicit Init(
        rclcpp::Node::SharedPtr node,
        NProtocolExtracter *protocol_extraction,
        SerialPort *serial);

private:
    void initDataTransmission();
    void initTagPose();
    void publishTagPose(uint8_t id, const float pos_3d[3]);
    void initAnchorSettings(NProtocolExtracter *protocol_extraction);
    void requestAnchorSettings();
    void publishAnchors();
    void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
    void initTagFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame1(NProtocolExtracter *protocol_extraction);
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void initNodeFrame3(NProtocolExtracter *protocol_extraction);
    void initNodeFrame4(NProtocolExtracter *protocol_extraction);
    void initNodeFrame5(NProtocolExtracter *protocol_extraction);
    void initNodeFrame6(NProtocolExtracter *protocol_extraction);
    void initNodeFrame7(NProtocolExtracter *protocol_extraction);

    std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;

    // tag id -> <pose_topic_prefix>/<tag_name_prefix><id>/pose
    std::unordered_map<uint8_t, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> tag_pose_pubs_;
    std::string pose_topic_prefix_;
    std::string tag_name_prefix_;
    std::string pose_frame_id_;

    // anchor coordinates read from the module (Setting_Frame0), published as RViz models
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr anchor_pub_;
    rclcpp::TimerBase::SharedPtr anchor_request_timer_;
    rclcpp::TimerBase::SharedPtr anchor_publish_timer_;
    std::mutex anchor_mutex_;  // filled from the serial thread, published from the ROS thread
    std::map<int, std::array<double, 3>> anchors_;
    int anchor_group_ = 0;
    int anchor_requests_ = 0;
    // a Setting_Frame0 is only accepted as the answer to our own read request
    std::atomic<bool> anchor_request_pending_{false};
};

}  /* namespace linktrack */

#endif  /* LINKTRACKINIT_H */