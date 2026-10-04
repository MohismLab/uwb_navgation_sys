#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include "nprotocol_extracter.h"
#include "nlink_parser2/msg/tofsense_m_frame0.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <memory>

namespace tofsensem
{
class Init
{
public:
    explicit Init(
        std::shared_ptr<NProtocolExtracter> protocol_extraction,
        std::shared_ptr<rclcpp::Node> node);

private:
    void InitFrame0(std::shared_ptr<NProtocolExtracter> protocol_extraction);

    std::unordered_map<NProtocolBase *, rclcpp::Publisher<nlink_parser2::msg::TofsenseMFrame0>::SharedPtr> publishers_;
    std::shared_ptr<rclcpp::Node> node_;
};

}  /* namespace tofsensem */

#endif  /* TOFSENSEMINIT_H */