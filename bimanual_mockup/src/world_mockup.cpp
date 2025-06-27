#include "bimanual_mockup/world_mockup.hpp"

using namespace bimanual_mockup;

WorldMockup::WorldMockup(const rclcpp::NodeOptions & options)
: Node("world_mockup", options)
{
  // Declare parameters
  declare_parameter("camera_params_file", "");
  declare_parameter("object_params_file", "");
  declare_parameter("world_params_file", "");
  declare_parameter("use_rviz", true);

  // Retrieve parameter values
  std::string cam_path = get_parameter("camera_params_file").as_string();
  std::string obj_path = get_parameter("object_params_file").as_string();
  std::string world_path = get_parameter("world_params_file").as_string();

  // Load from YAML files
  load_camera_params(cam_path);
  load_object_params(obj_path);
  load_world_params(world_path);

  // Create publishers
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

  // Timer for regular state publishing
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(timestep_ * 1000.0)),
    std::bind(&WorldMockup::publish_state, this)
  );

  RCLCPP_INFO(get_logger(), "WorldMockup node started.");
}

void WorldMockup::load_camera_params(const std::string & path)
{
  RCLCPP_INFO(get_logger(), "Loading camera parameters from: %s", path.c_str());

  YAML::Node config = YAML::LoadFile(path);

  for (const auto& side : {"left_camera", "right_camera"}) {
    if (!config[side]) continue;

    auto pose = config[side]["pose"];
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = std::string(side) + "_frame";
    tf.transform.translation.x = pose["position"][0].as<double>();
    tf.transform.translation.y = pose["position"][1].as<double>();
    tf.transform.translation.z = pose["position"][2].as<double>();
    tf.transform.rotation.x = pose["orientation"][0].as<double>();
    tf.transform.rotation.y = pose["orientation"][1].as<double>();
    tf.transform.rotation.z = pose["orientation"][2].as<double>();
    tf.transform.rotation.w = pose["orientation"][3].as<double>();

    // Store or broadcast this TF as needed
  }
}

void WorldMockup::load_object_params(const std::string & path)
{
  RCLCPP_INFO(get_logger(), "Loading object parameters from: %s", path.c_str());
  
  YAML::Node config = YAML::LoadFile(path);
  const auto& obj = config["object"];
  object_.color[0] = obj["color"][0].as<float>();
  object_.color[1] = obj["color"][1].as<float>();
  object_.color[2] = obj["color"][2].as<float>();
  object_.radius = obj["radius"].as<float>();

  RCLCPP_DEBUG(get_logger(), "Radius: %f", object_.radius);
}

void WorldMockup::load_world_params(const std::string & path)
{
  RCLCPP_INFO(get_logger(), "Loading world parameters from: %s", path.c_str());

  YAML::Node config = YAML::LoadFile(path);
  gravity_ = config["world"]["gravity"].as<double>();
  table_height_ = config["world"]["table_height"].as<double>();
}

void WorldMockup::publish_state()
{
  // 1. Apply gravity if not grasped
  if (grasp_state_ == "none") {
    object_.velocity_z -= gravity_ * timestep_;  // gravity * timestep (0.2 sec)
    object_.tf.transform.translation.z += object_.velocity_z * timestep_;

    // 2. Table collision check
    double min_z = table_height_ + object_.radius;
    if (object_.tf.transform.translation.z < min_z) {
      object_.tf.transform.translation.z = min_z;
      object_.velocity_z = 0.0;
    }
  }

  // 3. Broadcast object TF
  object_.tf.header.stamp = get_clock()->now();
  object_.tf.header.frame_id = "world";
  object_.tf.child_frame_id = "object";
  tf_broadcaster_->sendTransform(object_.tf);

  // 4. Publish sphere marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = get_clock()->now();
  marker.ns = "object";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = object_.tf.transform.translation.x;
  marker.pose.position.y = object_.tf.transform.translation.y;
  marker.pose.position.z = object_.tf.transform.translation.z;
  marker.pose.orientation.w = 1.0;  // no rotation
  marker.scale.x = object_.radius * 2.0;
  marker.scale.y = object_.radius * 2.0;
  marker.scale.z = object_.radius * 2.0;
  marker.color.r = object_.color[0];
  marker.color.g = object_.color[1];
  marker.color.b = object_.color[2];
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  object_marker_pub_->publish(marker);

  // TODO: Check fingertip distances and update tactile signals
  // TODO: Generate and publish synthetic images (color + depth)
}

/// @brief Main entry point for the world_mockup node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bimanual_mockup::WorldMockup>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
