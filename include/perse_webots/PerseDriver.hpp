#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include <yaml-cpp/yaml.h>
#include <vector>
#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include <math.h>

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

namespace perse_driver {

struct PointParam {
  rclcpp::ParameterValue x;
  rclcpp::ParameterValue y;
};

struct Point {
  double x;
  double y;
  bool hasCone;
};

class PerseDriver : public webots_ros2_driver::PluginInterface {
 public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) override;

 private:
  bool preStep();

  void spawnCones();
  void fetchPoints(const std::string yaml_file_path);
  void fetchParameters();
  void setupCallbacks();

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg, size_t path_index);
  void lookAheadIndexCallback(const std_msgs::msg::Int64::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ledCallback(const std_msgs::msg::Bool::SharedPtr msg);

  geometry_msgs::msg::Twist cmd_vel_msg;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;

  WbDeviceTag led;
  std_msgs::msg::Bool led_msg;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_subscription;

  WbNodeRef path_group_node_;
  WbNodeRef look_ahead_group_node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr look_ahead_index_subscription_;

  std_msgs::msg::Float64 encoder_msg;
  double last_encoder_reading = 0;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_pub;

  WbDeviceTag bumper;
  std_msgs::msg::Bool bumper_msg;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_pub;

  std_msgs::msg::Bool inductive_sensor_msg;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inductive_sensor_pub;

  double publish_timestep;
  double last_update;

  double encoder_constant;

  double max_forward;
  double max_backward;
  double max_steer;

  std::vector<std::string> path_colors_;
  std::vector<std::string> path_topics_;
  std::string look_ahead_index_topic_;
  std::vector<std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>>> path_subscriptions_;
  std::vector<nav_msgs::msg::Path::SharedPtr> latest_paths_;

  int64_t look_ahead_index_ = -1;

  WbNodeRef inductive_sensor_node;
  std::vector<Point> points = {{-1, -1}, {1, 1}};
  std::vector<WbNodeRef> coneNodes;

  webots_ros2_driver::WebotsNode* node;
};
}  // namespace perse_driver
#endif
