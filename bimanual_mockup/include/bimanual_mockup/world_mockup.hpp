#pragma once

#ifndef BIMANUAL_MOCKUP__WORLD_MOCKUP_HPP_
#define BIMANUAL_MOCKUP__WORLD_MOCKUP_HPP_

#include <array>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include "bimanual_msgs/msg/tactile.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bimanual_mockup
{

/**
 * @brief Struct representing a simple simulated object
 */
struct SimObject
{
  std::array<float, 3> color; ///< RGB color values in [0.0, 1.0]
  float radius;               ///< Radius of the object in meters
  geometry_msgs::msg::TransformStamped tf; ///< Pose of object in world frame
  double velocity_z = 0.0;
};

/**
 * @brief Struct representing a camera
 */
struct Camera
{
  std::array<double, 4> color_intrinsics; ///< [fx, fy, cx, cy] 
  std::array<int, 2> color_resolution;    ///< [width, height]

  std::array<double, 4> depth_intrinsics; ///< [fx, fy, cx, cy] 
  std::array<int, 2> depth_resolution;    ///< [width, height]

  geometry_msgs::msg::TransformStamped obj_tf;  ///< Transform of object in camera frame
};

/**
 * @brief Node that simulates the world including object pose, tactile sensing, and camera output
 */
class WorldMockup : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit WorldMockup(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Load camera parameters from YAML file
   * @param path Path to YAML file
   */
  void load_camera_params(const std::string & path);

  /**
   * @brief Load object parameters from YAML file
   * @param path Path to YAML file
   */
  void load_object_params(const std::string & path);

  /**
   * @brief Load world parameters (e.g. gravity) from YAML file
   * @param path Path to YAML file
   */
  void load_world_params(const std::string & path);

  /**
   * @brief Publishes all simulated outputs (camera, tactile, etc.)
   */
  void publish_state();

  // Internal state variables
  double timestep_ = 0.2;      // seconds
  SimObject object_;
  Camera right_cam_, left_cam_;
  double gravity_;
  double table_height_;
  std::string grasp_state_{"none"};   // "none", "left", "right"
  geometry_msgs::msg::Pose initial_pose_;
  const std::vector<std::string> left_fingertips = {
    "left_index_fingertip", "left_middle_fingertip", "left_pinky_fingertip", "left_thumb_fingertip"
  };
  const std::vector<std::string> right_fingertips = {
    "right_index_fingertip", "right_middle_fingertip", "right_pinky_fingertip", "right_thumb_fingertip"
  };

  // Camera/tactile publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_color_pub_, right_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_depth_pub_, right_depth_pub_;
  rclcpp::Publisher<bimanual_msgs::msg::Tactile>::SharedPtr left_tactile_pub_, right_tactile_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_marker_pub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat render_sphere_color(const SimObject & object, const geometry_msgs::msg::TransformStamped & cam_tf,
                          const std::array<double, 4> & intrinsics, const std::array<int, 2> & resolution);

  cv::Mat render_sphere_depth(const SimObject & object, const geometry_msgs::msg::TransformStamped & cam_tf,
                          const std::array<double, 4> & intrinsics, const std::array<int, 2> & resolution);
};

}  // namespace bimanual_mockup

#endif  // BIMANUAL_MOCKUP__WORLD_MOCKUP_HPP_
