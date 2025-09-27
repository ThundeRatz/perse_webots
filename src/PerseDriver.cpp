#include "perse_webots/PerseDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <webots/gyro.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>

#define ENCODER_RESOLUTION 0.01
#define PLATE_SIZE 1.0
#define WHEELBASE 0.257

namespace perse_driver {

void PerseDriver::init(

    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
  this->node = node;

  wbu_car_init();
  wbu_driver_init();

  // This parameter is read when loading the URDF file
  publish_timestep = parameters.count("updateRate")
                         ? 1.0 / atof(parameters["updateRate"].c_str())
                         : 0;

  bumper = wb_robot_get_device("bumper");
  led = wb_robot_get_device("led");

  wb_touch_sensor_enable(bumper, publish_timestep * 1000);

  inductive_sensor_node = wb_supervisor_node_get_from_def("INDUCTIVE_SENSOR");
  auto odometry_variables =
      node->declare_parameter("odometry_variables", rclcpp::PARAMETER_STRING);

  node->declare_parameter("encoder_constant", rclcpp::PARAMETER_DOUBLE);
  encoder_constant = node->get_parameter("encoder_constant").as_double();

  fetch_points(odometry_variables.get<std::string>());
  spawnCones();

  cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      "/perse/actuator/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&PerseDriver::cmdVelCallback, this, std::placeholders::_1));

  led_subscription = node->create_subscription<std_msgs::msg::Bool>(
      "/perse/actuator/led_rod", rclcpp::SensorDataQoS().reliable(),
      std::bind(&PerseDriver::ledCallback, this, std::placeholders::_1));

  this->encoder_pub = node->create_publisher<std_msgs::msg::Float64>(
      "/perse/sensors/encoder_fusion/distance", 10);
  std_msgs::msg::Float64 encoder_msg;
  encoder_msg.data = 0;
  encoder_pub->publish(encoder_msg);

  this->bumper_pub =
      node->create_publisher<std_msgs::msg::Bool>("/perse/sensors/bumper", 10);
  bumper_msg.data = false;
  bumper_pub->publish(bumper_msg);

  this->inductive_sensor_pub = node->create_publisher<std_msgs::msg::Bool>(
      "/perse/sensors/inductive_sensor", 10);
  inductive_sensor_msg.data = false;
  inductive_sensor_pub->publish(inductive_sensor_msg);
}

void PerseDriver::fetch_points(const std::string yaml_file_path) {
  // Vector to store the read points
  points.clear();

  try {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // Access the 'ros__parameters' node
    if (config["/decision_node"] &&
        config["/decision_node"]["ros__parameters"]) {
      YAML::Node parameters = config["/decision_node"]["ros__parameters"];

      // Loop through the points p1, p2, p3, etc.
      // Start from 1 and continue until a point is not found
      int i = 1;
      while (true) {
        std::string point_name = "p" + std::to_string(i);
        if (parameters[point_name]) {
          // Get the 'x' and 'y' values
          double x = parameters[point_name]["x"].as<double>();
          double y = parameters[point_name]["y"].as<double>();

          // Create a Point object and add it to the vector
          points.push_back({x, y});

          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Read point " << i << ": x=" << x << ", y=" << y);
          i++;
        } else {
          // No more points found, exit the loop
          break;
        }
      }
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Error: 'ros__parameters' node not found.");
    }

  } catch (const YAML::BadFile &e) {
    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Error: Could not open or parse the file: " << e.what());
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "YAML parsing error: " << e.what());
  }

  // You can now use the points_vector
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "\nSuccessfully read " << points.size() << " points.");
}

void PerseDriver::spawnCones() {
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children_field =
      wb_supervisor_node_get_field(root_node, "children");
  coneNodes.clear();

  for (int i = 0; i < this->points.size(); i++) {
    auto point = this->points[i];
    std::stringstream node_string;
    std::stringstream def_string;

    def_string << "CONE" << i;

    node_string << "DEF " << def_string.str() << " ConePlate { name \"plate"
                << i << "\"\ntranslation " << point.x << " " << point.y
                << " 0.0 }";

    node_string << "TrafficCone { physics Physics { mass 1 }\n name \"cone" << i
                << "\"\ntranslation " << point.x << " " << point.y << " 0.12 }";
    wb_supervisor_field_import_mf_node_from_string(root_children_field, -1,
                                                   node_string.str().c_str());

    WbNodeRef coneNode =
        wb_supervisor_node_get_from_def(def_string.str().c_str());
    if (coneNode != NULL) {
      coneNodes.push_back(coneNode);
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Error: Cone " << i << " was not found");
    }
  }
}

void PerseDriver::ledCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  led_msg.data = msg->data;
}

void PerseDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void PerseDriver::step() {
  if (wbu_driver_step() == -1) {
    return; // Sai se a simulação terminou
  }

  // Motor e Servo
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_servo_left =
      angular_speed == 0.0 ? 0.0
                           : std::clamp<double>(angular_speed, -0.99, 0.99) *
                                 sgn(-forward_speed);

  wbu_driver_set_cruising_speed(forward_speed * 3.6);
  wbu_driver_set_steering_angle(command_servo_left);

  // Encoder
  double encoder_reading =
      wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_REAR_LEFT) * encoder_constant;

  if (std::abs(encoder_reading - last_encoder_reading) >= ENCODER_RESOLUTION) {
    std_msgs::msg::Float64 encoder_msg;
    encoder_msg.data = std::ceil(encoder_reading * (1 / ENCODER_RESOLUTION)) /
                       (1 / ENCODER_RESOLUTION);
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
  const double *position =
      wb_supervisor_node_get_position(inductive_sensor_node);

  bool inductive_sensor_reading = false;
  for (auto cone_node : coneNodes) {
    const double *cone_position = wb_supervisor_node_get_position(cone_node);

    if (std::abs(position[0] - cone_position[0]) < PLATE_SIZE / 2 &&
        std::abs(position[1] - cone_position[1]) < PLATE_SIZE / 2) {
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
}

bool PerseDriver::preStep() {
  // Update only if needed
  if (wb_robot_get_time() - last_update < publish_timestep)
    return false;
  last_update = wb_robot_get_time();
  return true;
}

} // namespace perse_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perse_driver::PerseDriver,
                       webots_ros2_driver::PluginInterface)
