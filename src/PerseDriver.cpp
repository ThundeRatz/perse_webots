#include "perse_webots/PerseDriver.hpp"

#include <webots/gyro.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <cstdio>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"

#define ENCODER_RESOLUTION 0.001
#define PLATE_SIZE 1.0
#define WHEELBASE 0.257

namespace perse_driver {

static void matrixToQuaternion(const double R[9], double q[4]) {
  double tr = R[0] + R[4] + R[8];
  if (tr > 0) {
    double S = sqrt(tr + 1.0) * 2;  // S=4*qw
    q[3] = 0.25 * S;
    q[0] = (R[7] - R[5]) / S;
    q[1] = (R[2] - R[6]) / S;
    q[2] = (R[3] - R[1]) / S;
  } else if ((R[0] > R[4]) && (R[0] > R[8])) {
    double S = sqrt(1.0 + R[0] - R[4] - R[8]) * 2;  // S=4*qx
    q[3] = (R[7] - R[5]) / S;
    q[0] = 0.25 * S;
    q[1] = (R[1] + R[3]) / S;
    q[2] = (R[2] + R[6]) / S;
  } else if (R[4] > R[8]) {
    double S = sqrt(1.0 + R[4] - R[0] - R[8]) * 2;  // S=4*qy
    q[3] = (R[2] - R[6]) / S;
    q[0] = (R[1] + R[3]) / S;
    q[1] = 0.25 * S;
    q[2] = (R[5] + R[7]) / S;
  } else {
    double S = sqrt(1.0 + R[8] - R[0] - R[4]) * 2;  // S=4*qz
    q[3] = (R[3] - R[1]) / S;
    q[0] = (R[2] + R[6]) / S;
    q[1] = (R[5] + R[7]) / S;
    q[2] = 0.25 * S;
  }
}

void PerseDriver::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) {
  this->node = node;

  path_group_node_ = wb_supervisor_node_get_from_def("PATH_GROUP");
  if (!path_group_node_) {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not find DEF 'PATH_GROUP'. Please add a 'DEF "
                 "PATH_GROUP Group {}' node to your Webots world.");
  }

  look_ahead_group_node_ = wb_supervisor_node_get_from_def("LOOK_AHEAD_GROUP");
  if (!look_ahead_group_node_) {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not find DEF 'LOOK_AHEAD_GROUP'. Please add a 'DEF "
                 "LOOK_AHEAD_GROUP Group {}' node to your Webots world.");
  }

  fetchParameters();
  setupCallbacks();

  wbu_car_init();
  wbu_driver_init();

  // This parameter is read when loading the URDF file
  publish_timestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;

  bumper = wb_robot_get_device("bumper");
  led = wb_robot_get_device("led");

  wb_touch_sensor_enable(bumper, publish_timestep * 1000);

  perse_node_ = wb_supervisor_node_get_from_def("PERSE");

  inductive_sensor_node = wb_supervisor_node_get_from_proto_def(perse_node_, "INDUCTIVE_SENSOR");

  spawnCones();
}

void PerseDriver::fetchParameters() {
  max_forward = node->declare_parameter("properties.limits.speed.forward", rclcpp::PARAMETER_DOUBLE).get<double>();
  max_backward = node->declare_parameter("properties.limits.speed.backward", rclcpp::PARAMETER_DOUBLE).get<double>();
  max_steer = node->declare_parameter("properties.limits.steer", rclcpp::PARAMETER_DOUBLE).get<double>();

  encoder_constant = node->declare_parameter("encoder_constant", rclcpp::PARAMETER_DOUBLE).get<double>();

  auto mission_points = node->declare_parameter("mission_points", rclcpp::PARAMETER_STRING);
  fetchPoints(mission_points.get<std::string>());

  // NEW: Get path topics to subscribe to
  node->declare_parameter("path_topics", rclcpp::PARAMETER_STRING_ARRAY);
  auto path_topics = node->get_parameter("path_topics").as_string_array();

  // NEW: Get colors for each path (RGB format)
  node->declare_parameter("path_colors", rclcpp::PARAMETER_STRING_ARRAY);
  auto path_colors = node->get_parameter("path_colors").as_string_array();

  node->declare_parameter("topics.look_ahead_index", rclcpp::PARAMETER_STRING);
  auto look_ahead_index_topic = node->get_parameter("topics.look_ahead_index").as_string();

  path_topics_ = path_topics;
  path_colors_ = path_colors;
  this->look_ahead_index_topic_ = look_ahead_index_topic;
}

void PerseDriver::setupCallbacks() {
  cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>("/perse/actuator/cmd_vel", rclcpp::SensorDataQoS().reliable(),
                                                                              std::bind(&PerseDriver::cmdVelCallback, this, std::placeholders::_1));
  led_subscription = node->create_subscription<std_msgs::msg::Bool>("/perse/actuator/led_rod", rclcpp::SensorDataQoS().reliable(),
                                                                    std::bind(&PerseDriver::ledCallback, this, std::placeholders::_1));

  // Subscribe to multiple path topics
  for (size_t i = 0; i < path_topics_.size(); ++i) {
    auto topic = path_topics_[i];
    auto subscription = node->create_subscription<nav_msgs::msg::Path>(topic, rclcpp::SensorDataQoS().reliable(),
                                                                       [this, i](const nav_msgs::msg::Path::SharedPtr msg) {
                                                                         this->pathCallback(msg, i);
                                                                         RCLCPP_INFO(node->get_logger(), "received path at index: %zu", i);
                                                                       });
    path_subscriptions_.push_back(subscription);

    latest_paths_.push_back(nullptr);

    RCLCPP_INFO(node->get_logger(), "Subscribed to path topic: %s", topic.c_str());
  }

  if (!this->look_ahead_index_topic_.empty()) {
    look_ahead_index_subscription_ =
        node->create_subscription<std_msgs::msg::Int64>(this->look_ahead_index_topic_, rclcpp::SensorDataQoS().reliable(),
                                                        std::bind(&PerseDriver::lookAheadIndexCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(), "Subscribed to look-ahead index topic: %s", this->look_ahead_index_topic_.c_str());
  }

  encoder_pub = node->create_publisher<std_msgs::msg::Float64>("/perse/sensors/encoder_fusion/distance", rclcpp::SensorDataQoS().reliable());
  bumper_pub = node->create_publisher<std_msgs::msg::Bool>("/perse/sensors/bumper", rclcpp::SensorDataQoS().reliable());
  inductive_sensor_pub = node->create_publisher<std_msgs::msg::Bool>("/perse/sensors/inductive_sensor", rclcpp::SensorDataQoS().reliable());
  odom_gt_pub = node->create_publisher<nav_msgs::msg::Odometry>("/perse/simulation/odom_gt", rclcpp::SensorDataQoS().reliable());

  encoder_msg.data = 0;
  encoder_pub->publish(encoder_msg);

  bumper_msg.data = false;
  bumper_pub->publish(bumper_msg);

  inductive_sensor_msg.data = false;
  inductive_sensor_pub->publish(inductive_sensor_msg);
}

void PerseDriver::ledCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  led_msg.data = msg->data;
}

void PerseDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void PerseDriver::pathCallback(const nav_msgs::msg::Path::SharedPtr msg, size_t path_index) {
  if (!path_group_node_ || msg->poses.empty()) {
    return;
  }

  if (path_index >= latest_paths_.size()) {
    latest_paths_.resize(path_index + 1);
  }
  latest_paths_[path_index] = msg;

  std::string points_str;
  std::string indices_str;

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto& pose = msg->poses[i].pose;
    points_str += std::to_string(pose.position.x) + " " + std::to_string(pose.position.y) + " " + std::to_string(0.1);
    indices_str += std::to_string(i);

    if (i < msg->poses.size() - 1) {
      points_str += ", ";
      indices_str += ", ";
    }
  }

  // Get color for this path (default to red if not specified)
  std::string color = "1 0 0";  // Default red
  if (path_index < path_colors_.size()) {
    color = path_colors_[path_index];
  }

  // Generate the VRML string for the IndexedLineSet geometry
  std::string shape_string =
      "Shape {\n"
      "  appearance Appearance {\n"
      "    material Material {\n"
      "      emissiveColor " +
      color +
      "\n"
      "    }\n"
      "  }\n"
      "  geometry IndexedLineSet {\n"
      "    coord Coordinate {\n"
      "      point [\n"
      "        " +
      points_str +
      "\n"
      "      ]\n"
      "    }\n"
      "    coordIndex [\n"
      "      " +
      indices_str +
      "\n"
      "    ]\n"
      "  }\n"
      "}";

  WbFieldRef children_field = wb_supervisor_node_get_field(path_group_node_, "children");
  if (!children_field) {
    RCLCPP_ERROR(node->get_logger(), "Could not get 'children' field of PATH_GROUP.");
    return;
  }

  // Manage path visualization by index
  int field_count = wb_supervisor_field_get_count(children_field);

  // Fill gaps with empty groups if needed (when path_index >= field_count)
  while (field_count <= path_index) {
    wb_supervisor_field_import_mf_node_from_string(children_field, -1, "Group {}");
    field_count++;
  }

  // Remove existing path at this index (now guaranteed to exist)
  wb_supervisor_field_remove_mf(children_field, path_index);

  // Import the new path node at the specific index
  wb_supervisor_field_import_mf_node_from_string(children_field, path_index, shape_string.c_str());

  if (look_ahead_index_ >= 0) {
    const auto look_ahead_pose_index = std::min<size_t>(static_cast<size_t>(look_ahead_index_), msg->poses.size() - 1);

    const auto& pose = msg->poses[look_ahead_pose_index].pose;
    std::stringstream sphere_string;
    sphere_string << "Transform {\n"
                  << "  translation " << pose.position.x << " " << pose.position.y << " 0.1\n"
                  << "  children [\n"
                  << "    Shape {\n"
                  << "      appearance Appearance {\n"
                  << "        material Material {\n"
                  << "          emissiveColor 1 1 0\n"
                  << "        }\n"
                  << "      }\n"
                  << "      geometry Sphere { radius 0.04 }\n"
                  << "    }\n"
                  << "  ]\n"
                  << "}";

    WbFieldRef look_ahead_children_field = wb_supervisor_node_get_field(look_ahead_group_node_, "children");
    if (look_ahead_children_field) {
      int look_ahead_field_count = wb_supervisor_field_get_count(look_ahead_children_field);
      while (look_ahead_field_count <= static_cast<int>(path_index)) {
        wb_supervisor_field_import_mf_node_from_string(look_ahead_children_field, -1, "Group {}");
        look_ahead_field_count++;
      }

      wb_supervisor_field_remove_mf(look_ahead_children_field, path_index);
      wb_supervisor_field_import_mf_node_from_string(look_ahead_children_field, path_index, sphere_string.str().c_str());
    }
  }
}

void PerseDriver::lookAheadIndexCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  if (!look_ahead_group_node_) {
    return;
  }

  look_ahead_index_ = msg->data;

  if (look_ahead_index_ < 0) {
    return;
  }

  WbFieldRef look_ahead_children_field = wb_supervisor_node_get_field(look_ahead_group_node_, "children");
  if (!look_ahead_children_field) {
    RCLCPP_ERROR(node->get_logger(), "Could not get 'children' field of LOOK_AHEAD_GROUP.");
    return;
  }

  for (size_t path_index = 0; path_index < latest_paths_.size(); ++path_index) {
    const auto& path = latest_paths_[path_index];
    if (!path || path->poses.empty()) {
      continue;
    }

    const auto look_ahead_pose_index = std::min<size_t>(static_cast<size_t>(look_ahead_index_), path->poses.size() - 1);
    const auto& pose = path->poses[look_ahead_pose_index].pose;

    std::stringstream sphere_string;
    sphere_string << "Transform {\n"
                  << "  translation " << pose.position.x << " " << pose.position.y << " 0.1\n"
                  << "  children [\n"
                  << "    Shape {\n"
                  << "      appearance Appearance {\n"
                  << "        material Material {\n"
                  << "          emissiveColor 1 1 0\n"
                  << "        }\n"
                  << "      }\n"
                  << "      geometry Sphere { radius 0.04 }\n"
                  << "    }\n"
                  << "  ]\n"
                  << "}";

    int look_ahead_field_count = wb_supervisor_field_get_count(look_ahead_children_field);
    while (look_ahead_field_count <= static_cast<int>(path_index)) {
      wb_supervisor_field_import_mf_node_from_string(look_ahead_children_field, -1, "Group {}");
      look_ahead_field_count++;
    }

    wb_supervisor_field_remove_mf(look_ahead_children_field, path_index);
    wb_supervisor_field_import_mf_node_from_string(look_ahead_children_field, path_index, sphere_string.str().c_str());
  }
}

void PerseDriver::fetchPoints(const std::string yaml_file_path) {
  // Vector to store the read points
  points.clear();

  try {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    if (config["points"] && config["points"].IsSequence()) {
      int i = 1;
      for (const auto& pt : config["points"]) {
        double x = pt["x"].as<double>();
        double y = pt["y"].as<double>();
        bool hasCone = pt["has_cone"].as<bool>();

        // Create a Point object and add it to the vector
        points.push_back({x, y, hasCone});

        RCLCPP_INFO_STREAM(node->get_logger(), "Read point " << i << ": x=" << x << ", y=" << y);
        i++;
      }
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Error: 'points' sequence not found in YAML file.");
    }

  } catch (const YAML::BadFile& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Error: Could not open or parse the file: " << e.what());
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "YAML parsing error: " << e.what());
  }

  // You can now use the points_vector
  RCLCPP_INFO_STREAM(node->get_logger(), "\nSuccessfully read " << points.size() << " points.");
}

void PerseDriver::spawnCones() {
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  coneNodes.clear();

  for (int i = 0; i < this->points.size(); i++) {
    auto point = this->points[i];
    std::stringstream node_string;
    std::stringstream def_string;

    def_string << "CONE" << i;

    node_string << "DEF " << def_string.str() << " ConePlate { name \"plate" << i << "\"\ntranslation " << point.x << " " << point.y << " 0.0 }";

    if (point.hasCone) {
      node_string << "TrafficCone { physics Physics { mass 1 }\n name \"cone" << i << "\"\ntranslation " << point.x << " " << point.y << " 0.12 }";
    }
    wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, node_string.str().c_str());

    WbNodeRef coneNode = wb_supervisor_node_get_from_def(def_string.str().c_str());
    if (coneNode != NULL) {
      coneNodes.push_back(coneNode);
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Error: Cone " << i << " was not found");
    }
  }
}

void PerseDriver::step() {
  if (wbu_driver_step() == -1) {
    return;  // Sai se a simulação terminou
  }

  // Motor e Servo
  auto velocity_pwm = std::clamp<double>(cmd_vel_msg.linear.x, -1.0, 1.0);
  auto steer_angle = std::clamp<double>(cmd_vel_msg.angular.z, -max_steer, max_steer);

  auto command_steer_angle = steer_angle * sgn(-velocity_pwm);

  wbu_driver_set_cruising_speed(velocity_pwm >= 0 ? velocity_pwm * max_forward * 3.6 : velocity_pwm * max_backward * 3.6);

  // wbu_driver_set_gear(velocity_pwm >= 0 ? 1 : -1);
  // wbu_driver_set_throttle(std::abs(velocity_pwm));

  wbu_driver_set_steering_angle(command_steer_angle);

  // Encoder
  double encoder_reading = wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_REAR_LEFT) * encoder_constant;

  if (std::abs(encoder_reading - last_encoder_reading) >= ENCODER_RESOLUTION) {
    std_msgs::msg::Float64 encoder_msg;
    encoder_msg.data = std::ceil(encoder_reading * (1 / ENCODER_RESOLUTION)) / (1 / ENCODER_RESOLUTION);
    encoder_pub->publish(encoder_msg);
    last_encoder_reading = encoder_msg.data;
  }

  // Bumper
  double bumper_reading = wb_touch_sensor_get_value(bumper);
  bool last_bumper_reading = bumper_msg.data;

  bumper_msg.data = bumper_reading;
  if (bumper_reading != last_bumper_reading) {
    bumper_pub->publish(bumper_msg);
  }

  // Led
  wb_led_set(led, this->led_msg.data ? 1 : 0);

  // Inductive Sensor (inside 1x1m square around any cone)
  const double* position = wb_supervisor_node_get_position(inductive_sensor_node);

  bool inductive_sensor_reading = false;
  for (auto cone_node : coneNodes) {
    const double* cone_position = wb_supervisor_node_get_position(cone_node);

    if (std::abs(position[0] - cone_position[0]) < PLATE_SIZE / 2 && std::abs(position[1] - cone_position[1]) < PLATE_SIZE / 2) {
      inductive_sensor_reading = true;
      break;
    }
  }

  if (inductive_sensor_reading != inductive_sensor_msg.data) {
    inductive_sensor_msg.data = inductive_sensor_reading;
    inductive_sensor_pub->publish(inductive_sensor_msg);
  } else {
    inductive_sensor_msg.data = inductive_sensor_reading;
  }

  // Publish ground truth odometry
  if (perse_node_) {
    const double* pos = wb_supervisor_node_get_position(perse_node_);
    const double* rot_matrix = wb_supervisor_node_get_orientation(perse_node_);
    if (pos && rot_matrix) {
      double q[4];
      matrixToQuaternion(rot_matrix, q);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = node->get_clock()->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      odom_msg.pose.pose.position.x = pos[0];
      odom_msg.pose.pose.position.y = pos[1];
      odom_msg.pose.pose.position.z = pos[2];

      odom_msg.pose.pose.orientation.x = q[0];
      odom_msg.pose.pose.orientation.y = q[1];
      odom_msg.pose.pose.orientation.z = q[2];
      odom_msg.pose.pose.orientation.w = q[3];

      // Get velocity if available
      const double* velocity = wb_supervisor_node_get_velocity(perse_node_);
      if (velocity) {
        odom_msg.twist.twist.linear.x = velocity[0];
        odom_msg.twist.twist.linear.y = velocity[1];
        odom_msg.twist.twist.linear.z = velocity[2];
        odom_msg.twist.twist.angular.x = velocity[3];
        odom_msg.twist.twist.angular.y = velocity[4];
        odom_msg.twist.twist.angular.z = velocity[5];
      }

      odom_gt_pub->publish(odom_msg);
    }
  }
}

bool PerseDriver::preStep() {
  // Update only if needed
  // if (wb_robot_get_time() - last_update < publish_timestep)
  // return false;
  // last_update = wb_robot_get_time();
  return true;
}

}  // namespace perse_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perse_driver::PerseDriver, webots_ros2_driver::PluginInterface)
