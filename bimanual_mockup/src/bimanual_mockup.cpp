#include "bimanual_mockup/bimanual_mockup.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

BimanualMockup::BimanualMockup() : Node("bimanual_mockup")
{
  using namespace std::chrono_literals;

  // Initialize publishers
  right_fdbk_pub_ = create_publisher<sensor_msgs::msg::JointState>("/right/fdbk/positions", 10);
  left_fdbk_pub_ = create_publisher<sensor_msgs::msg::JointState>("/left/fdbk/positions", 10);
  right_hand_fdbk_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/right/fdbk/handstate", 10);
  left_hand_fdbk_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/left/fdbk/handstate", 10);
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/right/camera/color/image_raw", 10);
  right_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/right/camera/depth/image_raw", 10);
  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/left/camera/color/image_raw", 10);
  left_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/left/camera/depth/image_raw", 10);

  // Initialize subscribers
  right_cmd_pos_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/right/cmd/positions", 10, std::bind(&BimanualMockup::right_arm_cmd_cb, this, _1));
  left_cmd_pos_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/left/cmd/positions", 10, std::bind(&BimanualMockup::left_arm_cmd_cb, this, _1));
  right_hand_cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/right/cmd/handstate", 10, std::bind(&BimanualMockup::right_hand_cmd_cb, this, _1));
  left_hand_cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/left/cmd/handstate", 10, std::bind(&BimanualMockup::left_hand_cmd_cb, this, _1));

  // Start periodic publishing
  timer_ = create_wall_timer(200ms, std::bind(&BimanualMockup::publish_all, this));
}

// === Sub Callback Implementations ===
void BimanualMockup::right_arm_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  right_cmd_pos_ = *msg;
}

void BimanualMockup::left_arm_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  left_cmd_pos_ = *msg;
}

void BimanualMockup::right_hand_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  right_hand_state_ = *msg;
}

void BimanualMockup::left_hand_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  left_hand_state_ = *msg;
}

// === Timer Callback ===
void BimanualMockup::publish_all()
{
  // Feedback mirrors command
  right_fdbk_pub_->publish(right_cmd_pos_);
  left_fdbk_pub_->publish(left_cmd_pos_);
  right_hand_fdbk_pub_->publish(right_hand_state_);
  left_hand_fdbk_pub_->publish(left_hand_state_);

  // Create blank images (white for color, gray for depth)
  cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1000)); // 1m depth in mm

  auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg();
  auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth).toImageMsg();

  color_msg->header.stamp = now();
  depth_msg->header.stamp = now();

  right_image_pub_->publish(*color_msg);
  right_depth_pub_->publish(*depth_msg);
  left_image_pub_->publish(*color_msg);
  left_depth_pub_->publish(*depth_msg);
}

// === Main Entry Point ===
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BimanualMockup>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
