#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

/**
 * @brief A mock simulation node for bimanual manipulation.
 * 
 * Simulates a dual-arm robot system with basic hand and camera behavior.
 * Receives command positions and hand states, and publishes:
 *  - Feedback positions and hand states (mirroring commands)
 *  - Simulated RGB + depth images
 * 
 * Publishes all outputs synchronously at 5 Hz.
 */
class BimanualMockup : public rclcpp::Node
{
public:
  /**
   * @brief Construct the BimanualMockup node.
   */
  BimanualMockup();

private:
  // Command callbacks
  void right_arm_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void left_arm_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void right_hand_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void left_hand_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // Timer callback to publish feedback and camera data
  void publish_all();

  // Last received commands
  sensor_msgs::msg::JointState right_cmd_pos_;
  sensor_msgs::msg::JointState left_cmd_pos_;
  std_msgs::msg::Float32MultiArray right_hand_state_;
  std_msgs::msg::Float32MultiArray left_hand_state_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_fdbk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_fdbk_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_hand_fdbk_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_hand_fdbk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_depth_pub_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_cmd_pos_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_cmd_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr right_hand_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr left_hand_cmd_sub_;

  // Timer for synchronized publishing
  rclcpp::TimerBase::SharedPtr timer_;
};

