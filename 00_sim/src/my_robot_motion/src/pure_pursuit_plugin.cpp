#include <algorithm>
#include <chrono>
#include <cmath>

#include "my_robot_motion/pure_pursuit_plugin.hpp"
#include "tf2/time.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h" // purpose?? works without it also...

namespace robot_motion
{

void PurePursuitPlugin::configure(
const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  plugin_name_ = name;
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  carrot_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot", 10);

  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));

  node_->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node_->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node_->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
}

void PurePursuitPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PurePursuitPlugin");
  carrot_pub_.reset();
}

void PurePursuitPlugin::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuitPlugin");
  carrot_pub_->on_activate();
}

void PurePursuitPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuitPlugin");
  carrot_pub_->on_deactivate();
}

void PurePursuitPlugin::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(logger_, "Path received with " << path.poses.size() << " poses");
  RCLCPP_INFO_STREAM(logger_, "Path frame " << path.header.frame_id);
  global_plan_ = path;
}

void PurePursuitPlugin::setSpeedLimit(const double &, const bool &)
{}


geometry_msgs::msg::TwistStamped PurePursuitPlugin::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
{

  geometry_msgs::msg::TwistStamped cmd_vel; // result of the algorithm
  cmd_vel.header.frame_id = "base_footprint"; // robot_pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // Single "base_footprint <- map" lookup, exactly like the reference
  // Python (self.tf_buffer.lookup_transform("base_footprint", "map", ...)).
  geometry_msgs::msg::TransformStamped map_to_base_tf;
  try {
    map_to_base_tf = tf_->lookupTransform(
      "base_footprint", "map", tf2::TimePointZero, tf2::durationFromSec(0.2)); // robot_pose.header.frame_id;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "could not transform: %s", ex.what());
    return cmd_vel;
  }

  tf2::Transform transform_tf;
  tf2::fromMsg(map_to_base_tf.transform, transform_tf);

  // carrot_pose is already expressed in base_footprint (the robot's own
  // frame), so its position IS the robot-relative (dx, dy) directly - used
  // for the goal-distance check, the heading error, and the curvature.
  geometry_msgs::msg::Pose carrot_pose = getCarrotPose(transform_tf);

  double dx = carrot_pose.position.x;
  double dy = carrot_pose.position.y;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose = carrot_pose;
  pose_stamped.header.frame_id = "base_footprint";
  pose_stamped.header.stamp = clock_->now(); // node_->get_clock()->now()??
  carrot_pub_->publish(pose_stamped);

  cmd_vel.header.stamp = clock_->now();
  double heading_error = std::atan2(dy, dx);
  if (std::abs(heading_error) > (60.0 * M_PI / 180.0)) {  // tune this threshold
    // Target is behind / sharply off to the side: rotate in place first
    double heading_sign = (heading_error > 0.0) - (heading_error < 0.0);
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = max_angular_velocity_ * heading_sign;
  } else {
    double l_square = dx * dx + dy * dy;
    double curvature = (l_square > 0.01) ? (2.0 * dy) / l_square : 0.0;
    cmd_vel.twist.linear.x = max_linear_velocity_;
    cmd_vel.twist.angular.z = std::clamp(
      cmd_vel.twist.linear.x * curvature, -max_angular_velocity_, max_angular_velocity_);
  }

  return cmd_vel;
}

geometry_msgs::msg::Pose PurePursuitPlugin::transformPose(
    const tf2::Transform & transform_tf, const geometry_msgs::msg::PoseStamped & pose)
{
  tf2::Transform pose_tf;
  tf2::fromMsg(pose.pose, pose_tf);
  tf2::Transform transformed_tf = transform_tf * pose_tf;

  geometry_msgs::msg::Pose transformed_pose;
  tf2::toMsg(transformed_tf, transformed_pose);
  return transformed_pose;
}

geometry_msgs::msg::Pose PurePursuitPlugin::getCarrotPose(const tf2::Transform & transform_tf)
{
  geometry_msgs::msg::Pose carrot_pose = transformPose(transform_tf, global_plan_.poses.back());

  for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
    geometry_msgs::msg::Pose transformed_pose = transformPose(transform_tf, *pose_it);
    double dx = transformed_pose.position.x;
    double dy = transformed_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance > look_ahead_distance_) {
      carrot_pose = transformed_pose;
    } else {
      break;
    }
  }
  return carrot_pose;
}


}  // namespace robot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_motion::PurePursuitPlugin, nav2_core::Controller)