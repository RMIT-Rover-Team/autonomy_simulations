#include <algorithm>
#include <chrono>
#include <cmath>

#include "my_robot_motion/pd_motion_planner_plugin.hpp"
#include "tf2/time.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h" // purpose?? works without it also...

namespace robot_motion
{
void PDMotionPlannerPlugin::configure(
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
  next_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pd/next_pose", 10);

  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".kp", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".kd", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".step_size", rclcpp::ParameterValue(0.2));

  node_->get_parameter(plugin_name_ + ".kp", kp_);
  node_->get_parameter(plugin_name_ + ".kd", kd_);
  node_->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node_->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node_->get_parameter(plugin_name_ + ".step_size", step_size_);

}

void PDMotionPlannerPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PDMotionPlannerPlugin");
  next_pose_pub_.reset();
}

void PDMotionPlannerPlugin::activate()
{
  RCLCPP_INFO(logger_, "Activating PDMotionPlannerPlugin");
  next_pose_pub_->on_activate();
  prev_angular_error_ = 0.0;
  prev_linear_error_ = 0.0;
  last_cycle_time_ = node_->get_clock()->now();
}

void PDMotionPlannerPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PDMotionPlannerPlugin");
  next_pose_pub_->on_deactivate();
}

void PDMotionPlannerPlugin::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void PDMotionPlannerPlugin::setSpeedLimit(const double &, const bool &)
{}

geometry_msgs::msg::TwistStamped PDMotionPlannerPlugin::computeVelocityCommands(
const geometry_msgs::msg::PoseStamped & robot_pose,
const geometry_msgs::msg::Twist &, // previous velocity
nav2_core::GoalChecker *)
{

  geometry_msgs::msg::TwistStamped cmd_vel; // result of the algorithm
  cmd_vel.header.frame_id = "base_footprint"; // robot_pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "Empty Plan!");
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

  // next_pose is already expressed in base_footprint (the robot's own
  // frame), so its position IS the robot-relative error directly.
  geometry_msgs::msg::Pose next_pose = getNextPose(transform_tf);

  double dx = next_pose.position.x;
  double dy = next_pose.position.y;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose = next_pose;
  pose_stamped.header.frame_id = "base_footprint";
  pose_stamped.header.stamp = clock_->now(); // node_->get_clock()->now()??
  next_pose_pub_->publish(pose_stamped);

  double angular_error = dy;
  double linear_error = dx;

  rclcpp::Time now = clock_->now();
  double dt = (now - last_cycle_time_).seconds();
  if (dt <= 0.0) {
    // Guard against a zero/negative dt (e.g. first tick, or clock jitter)
    // which would blow up the derivative term.
    dt = 1e-3;
  }

  double angular_error_derivative = (angular_error - prev_angular_error_) / dt;
  double linear_error_derivative = (linear_error - prev_linear_error_) / dt;

  cmd_vel.header.stamp = now;
  cmd_vel.twist.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative,
    -max_angular_velocity_, max_angular_velocity_);
  cmd_vel.twist.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative,
    -max_linear_velocity_, max_linear_velocity_);

  prev_angular_error_ = angular_error;
  prev_linear_error_ = linear_error;
  last_cycle_time_ = now;

  return cmd_vel;
}

geometry_msgs::msg::Pose PDMotionPlannerPlugin::transformPose(
    const tf2::Transform & transform_tf, const geometry_msgs::msg::PoseStamped & pose)
{
  tf2::Transform pose_tf;
  tf2::fromMsg(pose.pose, pose_tf);
  tf2::Transform transformed_tf = transform_tf * pose_tf;

  geometry_msgs::msg::Pose transformed_pose;
  tf2::toMsg(transformed_tf, transformed_pose);
  return transformed_pose;
}

geometry_msgs::msg::Pose PDMotionPlannerPlugin::getNextPose(const tf2::Transform & transform_tf)
{
  geometry_msgs::msg::Pose next_pose = transformPose(transform_tf, global_plan_.poses.back());

  for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
    geometry_msgs::msg::Pose transformed_pose = transformPose(transform_tf, *pose_it);
    double dx = transformed_pose.position.x;
    double dy = transformed_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance > step_size_) {
      next_pose = transformed_pose;
    } else {
      break;
    }
  }
  return next_pose;
}

}  // namespace robot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_motion::PDMotionPlannerPlugin, nav2_core::Controller)