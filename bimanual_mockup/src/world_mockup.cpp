#include "bimanual_mockup/world_mockup.hpp"
#include <chrono>

using namespace std::chrono_literals;

WorldMockup::WorldMockup() : Node("world_mockup") {
  // Publishers
  left_rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/left/camera/color/image_raw", 10);
  right_rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/right/camera/color/image_raw", 10);
  left_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/left/camera/depth/image_raw", 10);
  right_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/right/camera/depth/image_raw", 10);

  left_tactile_pub_ = create_publisher<bimanual_msgs::msg::Tactile>("/left/tactile", 10);
  right_tactile_pub_ = create_publisher<bimanual_msgs::msg::Tactile>("/right/tactile", 10);

  object_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/object_marker", 10);

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Parameters
  declare_parameter<double>("object_radius", 0.025);
  get_parameter("object_radius", object_radius_);

  // Initialize object TF (positioned mid-air)
  object_tf_.header.frame_id = "world";
  object_tf_.child_frame_id = "object";
  object_tf_.transform.translation.x = 0.4;
  object_tf_.transform.translation.y = 0.0;
  object_tf_.transform.translation.z = 0.2;
  object_tf_.transform.rotation.w = 1.0;

  grasp_state_ = "none";

  timer_ = create_wall_timer(200ms, std::bind(&WorldMockup::timer_callback, this));
}

void WorldMockup::timer_callback() {
  // Publish dummy images
  sensor_msgs::msg::Image img;
  img.header.stamp = now();
  img.header.frame_id = "camera_frame";
  img.height = 480;
  img.width = 640;
  img.encoding = "rgb8";
  img.step = img.width * 3;
  img.data.resize(img.step * img.height, 255);  // white image

  left_rgb_pub_->publish(img);
  right_rgb_pub_->publish(img);
  left_depth_pub_->publish(img);
  right_depth_pub_->publish(img);

  // Publish TF for object
  object_tf_.header.stamp = now();
  tf_broadcaster_->sendTransform(object_tf_);

  // Publish dummy marker
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = "world";
  marker.ns = "object";
  marker.id = 0;
  marker.type = marker.SPHERE;
  marker.action = marker.ADD;
  marker.pose.position.x = object_tf_.transform.translation.x;
  marker.pose.position.y = object_tf_.transform.translation.y;
  marker.pose.position.z = object_tf_.transform.translation.z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = object_radius_ * 2.0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  object_marker_pub_->publish(marker);

  // (Later) compute tactile contact and update grasp state
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldMockup>());
  rclcpp::shutdown();
  return 0;
}
