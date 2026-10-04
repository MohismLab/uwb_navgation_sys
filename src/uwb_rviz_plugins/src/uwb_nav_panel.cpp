#include "uwb_nav_panel.hpp"

#include <QFormLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <memory>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace uwb_rviz_plugins
{

namespace
{
// same order as COLORS in uwb_fleet.py and the robot groups in uwb_nav.rviz
const char * kColors[] = {"#4fabff", "#ffa13d", "#6edb78", "#d973f2", "#f2e659"};

QDoubleSpinBox * makeSpin(double lo, double hi, double step, int decimals, double value)
{
  auto * spin = new QDoubleSpinBox;
  spin->setRange(lo, hi);
  spin->setSingleStep(step);
  spin->setDecimals(decimals);
  spin->setValue(value);
  return spin;
}

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

QString join(const std::vector<std::string> & v)
{
  QStringList l;
  for (const auto & s : v) {
    l << QString::fromStdString(s);
  }
  return l.join(", ");
}
}  // namespace

UwbNavPanel::UwbNavPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  check_row_ = new QHBoxLayout;

  // ranges = UwbGoalNav.TUNABLE in uwb_goal_nav.py
  max_speed_ = makeSpin(0.02, 1.0, 0.05, 2, 0.3);
  max_accel_ = makeSpin(0.05, 2.0, 0.1, 2, 0.4);
  k_p_ = makeSpin(0.1, 5.0, 0.1, 1, 0.8);
  tolerance_cm_ = makeSpin(2.0, 50.0, 1.0, 0, 5.0);

  auto * form = new QFormLayout;
  form->addRow("选中", check_row_);
  form->addRow("最大速度 (m/s)", max_speed_);
  form->addRow("最大加速度 (m/s²)", max_accel_);
  form->addRow("比例增益 (1/s)", k_p_);
  form->addRow("到达容差 (cm)", tolerance_cm_);

  auto * apply_btn = new QPushButton("应用");
  auto * read_btn = new QPushButton("读取");
  auto * stop_btn = new QPushButton("停车");
  auto * stop_all_btn = new QPushButton("全部停车");
  const char * red = "QPushButton { background-color: #c0392b; color: white; font-weight: bold; }";
  stop_btn->setStyleSheet(red);
  stop_all_btn->setStyleSheet(red);
  auto * row1 = new QHBoxLayout;
  row1->addWidget(apply_btn);
  row1->addWidget(read_btn);
  auto * row2 = new QHBoxLayout;
  row2->addWidget(stop_btn);
  row2->addWidget(stop_all_btn);

  auto * hint = new QLabel("在 3D 视图中点击机器人选中（Interact 工具），再用 2D Goal Pose 指定目标");
  hint->setWordWrap(true);
  hint->setStyleSheet("color: gray;");
  status_ = new QLabel;
  status_->setWordWrap(true);

  // magnetometer calibration: spin the selected cars in place (mag_flat_calib.py on the car)
  auto * calib_title = new QLabel("<b>地磁校准</b>（原地转圈，选中的机器人）");
  calib_turns_ = makeSpin(2.0, 20.0, 1.0, 0, 5.0);
  calib_turns_->setSuffix(" 圈");
  auto * calib_btn = new QPushButton("转圈校准选中的机器人");
  auto * calib_row = new QHBoxLayout;
  calib_row->addWidget(calib_turns_);
  calib_row->addWidget(calib_btn);
  calib_form_ = new QFormLayout;
  auto * calib_hint = new QLabel("转圈前确认机器人周围 0.5 m 内没有障碍物；“停车”可随时中止");
  calib_hint->setWordWrap(true);
  calib_hint->setStyleSheet("color: gray;");

  auto * layout = new QVBoxLayout;
  layout->addLayout(form);
  layout->addLayout(row1);
  layout->addLayout(row2);
  layout->addWidget(status_);
  layout->addWidget(hint);
  layout->addSpacing(10);
  layout->addWidget(calib_title);
  layout->addLayout(calib_row);
  layout->addLayout(calib_form_);
  layout->addWidget(calib_hint);
  layout->addStretch();
  setLayout(layout);
  connect(calib_btn, &QPushButton::clicked, this, &UwbNavPanel::startCalibration);

  connect(apply_btn, &QPushButton::clicked, this, &UwbNavPanel::apply);
  connect(read_btn, &QPushButton::clicked, this, &UwbNavPanel::readBack);
  connect(stop_btn, &QPushButton::clicked, this, &UwbNavPanel::stopSelected);
  connect(stop_all_btn, &QPushButton::clicked, this, &UwbNavPanel::stopAll);

  setRobots({"rm_0", "rm_1", "rm_2"});
}

void UwbNavPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  select_pub_ = node_->create_publisher<std_msgs::msg::String>("/uwb_nav/select", 10);
  // latched by uwb_fleet.py, so the current selection arrives right after start
  auto qos = rclcpp::QoS(1).transient_local();
  selection_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/uwb_nav/selection", qos, [this](const std_msgs::msg::String & msg) {
      auto v = split(msg.data);
      std::set<std::string> selected(v.begin(), v.end());
      QMetaObject::invokeMethod(this, [this, selected] {showSelection(selected);}, Qt::QueuedConnection);
    });
  subscribeCalibration();
}

void UwbNavPanel::setRobots(const std::vector<std::string> & robots)
{
  for (auto & [name, check] : checks_) {
    check_row_->removeWidget(check);
    delete check;
  }
  checks_.clear();
  robots_ = robots;
  for (size_t i = 0; i < robots_.size(); ++i) {
    auto * check = new QCheckBox(QString::fromStdString(robots_[i]));
    check->setStyleSheet(QString("QCheckBox { color: %1; font-weight: bold; }").arg(kColors[i % 5]));
    connect(check, &QCheckBox::toggled, this, &UwbNavPanel::checkboxToggled);
    check_row_->addWidget(check);
    checks_[robots_[i]] = check;
  }

  while (calib_form_->rowCount() > 0) {
    calib_form_->removeRow(0);
  }
  calib_labels_.clear();
  for (size_t i = 0; i < robots_.size(); ++i) {
    auto * name = new QLabel(QString::fromStdString(robots_[i]));
    name->setStyleSheet(QString("color: %1; font-weight: bold;").arg(kColors[i % 5]));
    auto * label = new QLabel("未连接");
    label->setWordWrap(true);
    label->setStyleSheet("color: gray;");
    calib_form_->addRow(name, label);
    calib_labels_[robots_[i]] = label;
  }
  subscribeCalibration();
}

void UwbNavPanel::subscribeCalibration()
{
  if (!node_) {
    return;
  }
  calib_subs_.clear();
  auto latched = rclcpp::QoS(1).transient_local();
  for (const auto & robot : robots_) {
    // created up front: discovery takes a moment, so a publisher made on click would see
    // no subscriber yet
    if (!calib_pubs_[robot]) {
      calib_pubs_[robot] = node_->create_publisher<std_msgs::msg::Float32>(
        "/" + robot + "/imu/flat_calib/start", 10);
    }
    calib_subs_[robot] = node_->create_subscription<std_msgs::msg::String>(
      "/" + robot + "/imu/flat_calib/status", latched, [this, robot](const std_msgs::msg::String & msg) {
        QString text = QString::fromStdString(msg.data);
        QMetaObject::invokeMethod(this, [this, robot, text] {
          auto it = calib_labels_.find(robot);
          if (it == calib_labels_.end()) {
            return;
          }
          // statuses of mag_flat_calib.py: 待命 / 校准中 / 完成 / 失败
          const char * color = text.startsWith("完成") ? "#27ae60" :
          text.startsWith("失败") ? "#c0392b" :
          text.startsWith("校准中") ? "#e67e22" : "gray";
          it->second->setStyleSheet(QString("color: %1;").arg(color));
          it->second->setText(text);
        }, Qt::QueuedConnection);
      });
  }
}

void UwbNavPanel::startCalibration()
{
  auto robots = selectedRobots();
  if (robots.empty()) {
    setStatus("先选中要校准的机器人", false);
    return;
  }
  if (!node_) {
    return;
  }
  std::vector<std::string> offline;
  for (const auto & robot : robots) {
    auto & pub = calib_pubs_[robot];
    if (!pub) {
      pub = node_->create_publisher<std_msgs::msg::Float32>("/" + robot + "/imu/flat_calib/start", 10);
    }
    if (pub->get_subscription_count() == 0) {
      offline.push_back(robot);
      continue;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(calib_turns_->value());
    pub->publish(msg);
  }
  if (!offline.empty()) {
    setStatus("这些车上的 IMU/校准程序没在运行 (car_imu.launch.py): " + join(offline), false);
  } else {
    setStatus("开始校准: " + join(robots), true);
  }
}

void UwbNavPanel::showSelection(const std::set<std::string> & selected)
{
  for (auto & [name, check] : checks_) {
    QSignalBlocker block(check);   // do not echo the fleet's own selection back
    check->setChecked(selected.count(name) > 0);
  }
  auto sel = selectedRobots();
  setStatus(sel.empty() ? "未选中机器人" : "已选中: " + join(sel), true);
}

std::vector<std::string> UwbNavPanel::selectedRobots() const
{
  std::vector<std::string> out;
  for (const auto & r : robots_) {
    if (checks_.at(r)->isChecked()) {
      out.push_back(r);
    }
  }
  return out;
}

void UwbNavPanel::checkboxToggled()
{
  if (!select_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  for (const auto & r : selectedRobots()) {
    msg.data += (msg.data.empty() ? "" : ",") + r;
  }
  select_pub_->publish(msg);
}

rclcpp::AsyncParametersClient::SharedPtr UwbNavPanel::params(const std::string & robot)
{
  auto & client = params_[robot];
  if (!client && node_) {
    client = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/uwb_goal_nav_" + robot);
  }
  return client;
}

void UwbNavPanel::apply()
{
  auto robots = selectedRobots();
  if (robots.empty()) {
    setStatus("先选中机器人", false);
    return;
  }
  std::vector<rclcpp::Parameter> values{
    {"max_speed", max_speed_->value()},
    {"max_accel", max_accel_->value()},
    {"k_p", k_p_->value()},
    {"tolerance", tolerance_cm_->value() / 100.0},
  };
  std::vector<std::string> offline;
  for (const auto & robot : robots) {
    auto client = params(robot);
    if (!client || !client->service_is_ready()) {
      offline.push_back(robot);
      continue;
    }
    client->set_parameters(
      values,
      [this, robot](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        QString error;
        for (const auto & r : future.get()) {
          if (!r.successful) {
            error = QString::fromStdString(r.reason);
          }
        }
        QString who = QString::fromStdString(robot);
        QMetaObject::invokeMethod(this, [this, who, error] {
          if (error.isEmpty()) {
            setStatus(who + " 已应用", true);
          } else {
            setStatus(who + " 被拒绝: " + error, false);
          }
        }, Qt::QueuedConnection);
      });
  }
  if (!offline.empty()) {
    setStatus("导航节点未运行: " + join(offline), false);
  }
}

void UwbNavPanel::readBack()
{
  auto robots = selectedRobots();
  if (robots.empty()) {
    setStatus("先选中机器人", false);
    return;
  }
  auto client = params(robots.front());
  if (!client || !client->service_is_ready()) {
    setStatus("导航节点未运行: " + QString::fromStdString(robots.front()), false);
    return;
  }
  std::string robot = robots.front();
  client->get_parameters(
    {"max_speed", "max_accel", "k_p", "tolerance"},
    [this, robot](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      auto values = future.get();
      QMetaObject::invokeMethod(this, [this, values, robot] {
        for (const auto & p : values) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            continue;
          }
          double v = p.as_double();
          if (p.get_name() == "max_speed") {
            max_speed_->setValue(v);
          } else if (p.get_name() == "max_accel") {
            max_accel_->setValue(v);
          } else if (p.get_name() == "k_p") {
            k_p_->setValue(v);
          } else if (p.get_name() == "tolerance") {
            tolerance_cm_->setValue(v * 100.0);
          }
        }
        setStatus("已读取 " + QString::fromStdString(robot) + " 的参数", true);
      }, Qt::QueuedConnection);
    });
}

void UwbNavPanel::cancel(const std::vector<std::string> & robots)
{
  if (!node_) {
    return;
  }
  for (const auto & robot : robots) {
    auto & pub = cancel_pubs_[robot];
    if (!pub) {
      pub = node_->create_publisher<std_msgs::msg::Empty>("/uwb_nav/" + robot + "/cancel", 10);
    }
    pub->publish(std_msgs::msg::Empty());
  }
}

void UwbNavPanel::stopSelected()
{
  auto robots = selectedRobots();
  if (robots.empty()) {
    setStatus("先选中机器人", false);
    return;
  }
  cancel(robots);
  setStatus("已停车: " + join(robots), true);
}

void UwbNavPanel::stopAll()
{
  cancel(robots_);
  setStatus("全部停车", true);
}

void UwbNavPanel::setStatus(const QString & text, bool ok)
{
  status_->setStyleSheet(ok ? "color: #27ae60;" : "color: #c0392b;");
  status_->setText(text);
}

void UwbNavPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString robots;
  if (config.mapGetString("robots", &robots)) {
    setRobots(split(robots.toStdString()));
  }
}

void UwbNavPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("robots", join(robots_).remove(' '));
}

}  // namespace uwb_rviz_plugins

PLUGINLIB_EXPORT_CLASS(uwb_rviz_plugins::UwbNavPanel, rviz_common::Panel)
