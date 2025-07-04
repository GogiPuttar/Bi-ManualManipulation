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
  left_color_pub_ = create_publisher<sensor_msgs::msg::Image>("/left/camera/color/image_raw", 10);
  right_color_pub_ = create_publisher<sensor_msgs::msg::Image>("/right/camera/color/image_raw", 10);
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

  // Initial pose
  initial_pose_.position.x = 0.4;
  initial_pose_.position.y = 0.8;
  initial_pose_.position.z = 0.5;
  initial_pose_.orientation.w = 1.0;

  object_.tf.transform.translation.x = initial_pose_.position.x;
  object_.tf.transform.translation.y = initial_pose_.position.y;
  object_.tf.transform.translation.z = initial_pose_.position.z;

  object_.tf.transform.rotation.x = initial_pose_.orientation.x;
  object_.tf.transform.rotation.y = initial_pose_.orientation.y;
  object_.tf.transform.rotation.z = initial_pose_.orientation.z;
  object_.tf.transform.rotation.w = initial_pose_.orientation.w;

  // 3. Broadcast object TF
  object_.tf.header.stamp = get_clock()->now();
  object_.tf.header.frame_id = "world";
  object_.tf.child_frame_id = "object";
  tf_broadcaster_->sendTransform(object_.tf);

  // Respawn parameters
  // bin at (0.0, 0.6, table_height)
  bin_center_.x = -0.6;
  bin_center_.y = 0.2;
  bin_center_.z = table_height_ + object_.radius;

  // Respawn near right hand (~shoulder level)
  right_spawn_center_.x = 0.3;
  right_spawn_center_.y = 0.6;

  RCLCPP_INFO(get_logger(), "WorldMockup node started.");
}

void WorldMockup::load_camera_params(const std::string & path)
{
  RCLCPP_INFO(get_logger(), "Loading camera parameters from: %s", path.c_str());
  YAML::Node config = YAML::LoadFile(path);

  for (const auto& cam_key : {"left_camera", "right_camera"}) {
    const auto& node = config[cam_key];
    if (!node) continue;

    Camera* cam = (std::string(cam_key) == "left_camera") ? &left_cam_ : &right_cam_;

    auto c_intr = node["color_intrinsics"];
    cam->color_intrinsics = {c_intr["fx"].as<double>(), c_intr["fy"].as<double>(),
                             c_intr["cx"].as<double>(), c_intr["cy"].as<double>()};

    cam->color_resolution = {
      node["color_resolution"]["width"].as<int>(),
      node["color_resolution"]["height"].as<int>()
    };

    auto d_intr = node["depth_intrinsics"];
    cam->depth_intrinsics = {d_intr["fx"].as<double>(), d_intr["fy"].as<double>(),
                             d_intr["cx"].as<double>(), d_intr["cy"].as<double>()};

    cam->depth_resolution = {
      node["depth_resolution"]["width"].as<int>(),
      node["depth_resolution"]["height"].as<int>()
    };
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
  // Check fingertip distances and update tactile signals
  double contact_thresh = object_.radius + 0.015;  // 1.5cm beyond object surface

  bimanual_msgs::msg::Tactile left_msg, right_msg;
  left_msg.contacts.resize(4, false);
  right_msg.contacts.resize(4, false);

  for (int i = 0; i < 4; ++i) {
    try {
      auto tf = tf_buffer_->lookupTransform(left_fingertips[i], "object", tf2::TimePointZero);
      double dx = tf.transform.translation.x;
      double dy = tf.transform.translation.y;
      double dz = tf.transform.translation.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < contact_thresh)
        left_msg.contacts[i] = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Left finger %d TF lookup failed: %s", i, ex.what());
    }

    try {
      auto tf = tf_buffer_->lookupTransform(right_fingertips[i], "object", tf2::TimePointZero);
      double dx = tf.transform.translation.x;
      double dy = tf.transform.translation.y;
      double dz = tf.transform.translation.z;

      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      if (dist < contact_thresh)
        right_msg.contacts[i] = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Right finger %d TF lookup failed: %s", i, ex.what());
    }
  }

  left_tactile_pub_->publish(left_msg);
  right_tactile_pub_->publish(right_msg);

  // Constants
  const double palm_thresh = 0.1; // meters

  // Step 1: Check grasp logic
  int left_contacts = 0, right_contacts = 0;
  bool left_grasped = check_grasp("left_palm", left_msg.contacts, left_contacts, palm_thresh);
  bool right_grasped = check_grasp("right_palm", right_msg.contacts, right_contacts, palm_thresh);

  if (left_grasped && grasp_state_ != "left") {
    // Just grabbed by left hand
    tf2::Transform tf_obj;
    tf2::fromMsg(object_.tf.transform, tf_obj);
    auto palm_tf = tf_buffer_->lookupTransform("left_palm", object_.tf.header.frame_id, tf2::TimePointZero);
    tf2::Transform tf_palm;
    tf2::fromMsg(palm_tf.transform, tf_palm);

    tf2::Transform new_tf = tf_palm * tf_obj;
    object_.tf.transform = tf2::toMsg(new_tf);

    object_.tf.header.frame_id = "left_palm";
    object_.velocity_z = 0.0;
    grasp_state_ = "left";

  } else if (right_grasped && grasp_state_ != "right" && !left_grasped) {
    // Just grabbed by right hand
    tf2::Transform tf_obj;
    tf2::fromMsg(object_.tf.transform, tf_obj);
    auto palm_tf = tf_buffer_->lookupTransform("right_palm", object_.tf.header.frame_id, tf2::TimePointZero);
    tf2::Transform tf_palm;
    tf2::fromMsg(palm_tf.transform, tf_palm);

    tf2::Transform new_tf = tf_palm * tf_obj;
    object_.tf.transform = tf2::toMsg(new_tf);

    object_.tf.header.frame_id = "right_palm";
    object_.velocity_z = 0.0;
    grasp_state_ = "right";

  } else if (!left_grasped && !right_grasped && grasp_state_ != "none") {
    // Just released
    tf2::Transform tf_obj;
    tf2::fromMsg(object_.tf.transform, tf_obj);
    auto world_tf = tf_buffer_->lookupTransform("world", object_.tf.header.frame_id, tf2::TimePointZero);
    tf2::Transform tf_world;
    tf2::fromMsg(world_tf.transform, tf_world);

    tf2::Transform new_tf = tf_world * tf_obj;
    object_.tf.transform = tf2::toMsg(new_tf);

    object_.tf.header.frame_id = "world";
    object_.velocity_z = 0.0;
    grasp_state_ = "none";
  }

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
  // object_.tf.header.frame_id = "world";
  object_.tf.child_frame_id = "object";
  tf_broadcaster_->sendTransform(object_.tf);

  // 4. Publish sphere marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "object";
  marker.header.stamp = get_clock()->now();
  marker.ns = "object";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
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

  // Get transform from object to camera 
  try {
    left_cam_.obj_tf = tf_buffer_->lookupTransform("left_camera", "object", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Could not transform from 'object' to 'left_camera': %s", ex.what());
    return;  // skip this cycle
  }

  try {
    right_cam_.obj_tf = tf_buffer_->lookupTransform("right_camera", "object", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Could not transform from 'object' to 'right_camera': %s", ex.what());
    return;  // skip this cycle
  }

  auto left_color = render_sphere_color(object_, left_cam_.obj_tf, left_cam_.color_intrinsics, left_cam_.color_resolution);
  auto left_depth = render_sphere_depth(object_, left_cam_.obj_tf, left_cam_.depth_intrinsics, left_cam_.depth_resolution);
  auto right_color = render_sphere_color(object_, right_cam_.obj_tf, right_cam_.color_intrinsics, right_cam_.color_resolution);
  auto right_depth = render_sphere_depth(object_, right_cam_.obj_tf, right_cam_.depth_intrinsics, right_cam_.depth_resolution);

  // Convert to ROS msgs

  // Build header
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();

  header.frame_id = "left_camera";  // or "right_camera" for right images
  auto left_color_msg = cv_bridge::CvImage(header, "bgr8", left_color).toImageMsg();
  auto left_depth_msg = cv_bridge::CvImage(header, "32FC1", left_depth).toImageMsg();

  header.frame_id = "right_camera";
  auto right_color_msg = cv_bridge::CvImage(header, "bgr8", right_color).toImageMsg();
  auto right_depth_msg = cv_bridge::CvImage(header, "32FC1", right_depth).toImageMsg();

  left_color_pub_->publish(*left_color_msg);
  left_depth_pub_->publish(*left_depth_msg);
  right_color_pub_->publish(*right_color_msg);
  right_depth_pub_->publish(*right_depth_msg);

  // Respawn checker
  if (grasp_state_ == "none") {
    // Check if object is in bin
    double dx = object_.tf.transform.translation.x - bin_center_.x;
    double dy = object_.tf.transform.translation.y - bin_center_.y;
    double dz = object_.tf.transform.translation.z - bin_center_.z;

    double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy < bin_radius_ && std::abs(dz) < 0.05) {
      respawn_object_near_right_hand();
    }
  }
}

cv::Mat WorldMockup::render_sphere_color(const SimObject & obj,
                                       const geometry_msgs::msg::TransformStamped & tf_cam_obj,
                                       const std::array<double, 4> & intr,
                                       const std::array<int, 2> & res)
{
  cv::Mat image(res[1], res[0], CV_8UC3, cv::Scalar(0, 0, 0));

  // Transform object center to camera frame
  tf2::Vector3 cam_coords(tf_cam_obj.transform.translation.x,
                          tf_cam_obj.transform.translation.y,
                          tf_cam_obj.transform.translation.z
                        );

  RCLCPP_DEBUG(get_logger(), "Cam coords: [%.2f, %.2f, %.2f]", cam_coords.x(), cam_coords.y(), cam_coords.z());

  if (cam_coords.z() <= 0.1) return image;

  double u = intr[0] * (cam_coords.x() / cam_coords.z()) + intr[2];
  double v = intr[1] * (cam_coords.y() / cam_coords.z()) + intr[3];
  int radius_px = static_cast<int>(intr[0] * obj.radius / cam_coords.z());

  RCLCPP_DEBUG(get_logger(), "Projected pixel: (u, v) = (%.1f, %.1f), radius_px = %d", u, v, radius_px);

  if (u < 0 || u >= res[0] || v < 0 || v >= res[1]) {
    RCLCPP_DEBUG(get_logger(), "Object is out of view: skipping draw.");
    return image;
  }

  cv::circle(image, cv::Point(u, v), radius_px,
             cv::Scalar(255 * obj.color[2], 255 * obj.color[1], 255 * obj.color[0]), -1);

  return image;
}

cv::Mat WorldMockup::render_sphere_depth(const SimObject & obj,
                                         const geometry_msgs::msg::TransformStamped & tf_cam_obj,
                                         const std::array<double, 4> & intr,
                                         const std::array<int, 2> & res)
{
  cv::Mat depth(res[1], res[0], CV_32FC1, cv::Scalar(0.0));

  // Transform object center to camera frame
  tf2::Vector3 cam_coords(tf_cam_obj.transform.translation.x,
                          tf_cam_obj.transform.translation.y,
                          tf_cam_obj.transform.translation.z
                        );

  if (cam_coords.z() <= 0.1) return depth;

  double u = intr[0] * (cam_coords.x() / cam_coords.z()) + intr[2];
  double v = intr[1] * (cam_coords.y() / cam_coords.z()) + intr[3];
  int radius_px = static_cast<int>(intr[0] * obj.radius / cam_coords.z());

  cv::circle(depth, cv::Point(u, v), radius_px, cam_coords.z(), -1);

  return depth;
}

bool WorldMockup::check_grasp(const std::string& palm_frame,
                              const std::vector<bool>& contact_flags,
                              int& num_contacts,
                              double palm_thresh)
{
  num_contacts = 0;

  for (int i = 0; i < 4; ++i)
    if (contact_flags[i]) ++num_contacts;

  if (num_contacts < 3)
    return false;
  
  try {
    auto tf = tf_buffer_->lookupTransform(palm_frame, "object", tf2::TimePointZero);
    double dx = tf.transform.translation.x;
    double dy = tf.transform.translation.y;
    double dz = tf.transform.translation.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist < palm_thresh;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

void WorldMockup::respawn_object_near_right_hand()
{
  double dx = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * spawn_window_xy_;
  double dy = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * spawn_window_xy_;

  object_.tf.header.frame_id = "world";
  object_.tf.transform.translation.x = right_spawn_center_.x + dx;
  object_.tf.transform.translation.y = right_spawn_center_.y + dy;
  object_.tf.transform.translation.z = spawn_height_;
  object_.tf.transform.rotation.x = 0.0;
  object_.tf.transform.rotation.y = 0.0;
  object_.tf.transform.rotation.z = 0.0;
  object_.tf.transform.rotation.w = 1.0;

  object_.velocity_z = 0.0;
  grasp_state_ = "none";

  RCLCPP_INFO(get_logger(), "Object respawned at (%.2f, %.2f, %.2f)",
              object_.tf.transform.translation.x,
              object_.tf.transform.translation.y,
              object_.tf.transform.translation.z);
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
