#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
class SerialPort;

#include "nprotocol_extracter.h"
#include "nlink_utils/nlink_protocol.h"
#include "std_msgs/msg/string.hpp"
#include "nlink_parser2/msg/linktrack_aoa_nodeframe0.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe0.hpp"

class NProtocolExtracter;

namespace linktrack_aoa
{
extern std::map<NLinkProtocol*, std::shared_ptr<rclcpp::PublisherBase>> publishers_;

class Init
{
public:
  explicit Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter *protocol_extraction,
    SerialPort *serial 
  );

private:
  void initDataTransmission();
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void initAoaNodeFrame0(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
};

}  /* namespace linktrack_aoa */

#endif  /* LINKTRACKAOAINIT_H */