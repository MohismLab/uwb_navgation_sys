#include "nlink_utils/nutils.h"

#include <rclcpp/rclcpp.hpp>

void TopicAdvertisedTip(const char *topic) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s has been advertised,use 'ros2 topic "
           "echo /%s' to view the data", topic, topic);
}
