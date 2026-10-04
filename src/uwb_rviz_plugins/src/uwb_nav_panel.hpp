#ifndef UWB_RVIZ_PLUGINS__UWB_NAV_PANEL_HPP_
#define UWB_RVIZ_PLUGINS__UWB_NAV_PANEL_HPP_

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <map>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

class QFormLayout;

namespace uwb_rviz_plugins
{

// RTS-style control of the UWB robots (uwb_fleet.py + one uwb_goal_nav.py per robot):
// selection checkboxes synced with clicking robots in the 3D view, runtime parameters of
// the selected robots' nav nodes (/uwb_goal_nav_<robot>), stop buttons.
class UwbNavPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit UwbNavPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void apply();
  void readBack();
  void stopSelected();
  void stopAll();
  void checkboxToggled();
  void startCalibration();

private:
  void setRobots(const std::vector<std::string> & robots);
  void showSelection(const std::set<std::string> & selected);
  std::vector<std::string> selectedRobots() const;
  rclcpp::AsyncParametersClient::SharedPtr params(const std::string & robot);
  void cancel(const std::vector<std::string> & robots);
  void setStatus(const QString & text, bool ok);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> robots_;
  std::map<std::string, QCheckBox *> checks_;
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> params_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> cancel_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr select_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selection_sub_;
  // in-place magnetometer calibration (mag_flat_calib.py on each car)
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> calib_pubs_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> calib_subs_;
  std::map<std::string, QLabel *> calib_labels_;
  QFormLayout * calib_form_;
  QDoubleSpinBox * calib_turns_;
  void subscribeCalibration();

  QHBoxLayout * check_row_;
  QDoubleSpinBox * max_speed_;
  QDoubleSpinBox * max_accel_;
  QDoubleSpinBox * k_p_;
  QDoubleSpinBox * tolerance_cm_;
  QLabel * status_;
};

}  // namespace uwb_rviz_plugins

#endif  // UWB_RVIZ_PLUGINS__UWB_NAV_PANEL_HPP_
