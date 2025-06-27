#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/TransformStamped.hpp"
#include "bimanual_msgs/msg/tactile.hpp"
#include "bimanual_msgs/msg/grasp_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class WorldMockup : public rclcpp::Node {
public:
  WorldMockup();

private:
  void timer_callback();

  // Camera/tactile publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rgb_pub_, right_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_depth_pub_, right_depth_pub_;
  rclcpp::Publisher<bimanual_msgs::msg::Tactile>::SharedPtr left_tactile_pub_, right_tactile_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_marker_pub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal object state
  std::string grasp_state_;  // "none", "left", "right"
  geometry_msgs::msg::TransformStamped object_tf_;
  double object_radius_;
};
