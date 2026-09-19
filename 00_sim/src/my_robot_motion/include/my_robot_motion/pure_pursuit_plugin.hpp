#ifndef PURE_PURSUIT_PLUGIN_HPP
#define PURE_PURSUIT_PLUGIN_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <string>

namespace robot_motion
{
class PurePursuitPlugin : public nav2_core::Controller
{
public:
    PurePursuitPlugin() = default;
    ~PurePursuitPlugin() = default;

    void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:

    std::string plugin_name_;
    rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuit")};
    rclcpp::Clock::SharedPtr clock_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> carrot_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;


    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    // Global plan, expressed in the "map" frame (the frame the planner
    // publishes it in). Nothing pre-transforms this plan; each candidate
    // pose is transformed on demand in getCarrotPose(), exactly as the
    // reference Python implementation does.
    nav_msgs::msg::Path global_plan_;

    // Transforms a single pose (expressed in the plan's "map" frame) into
    // the robot's "base_footprint" frame using transform_tf.
    geometry_msgs::msg::Pose transformPose(
        const tf2::Transform & transform_tf, const geometry_msgs::msg::PoseStamped & pose);

    // Walks global_plan_ backwards from the goal, transforming each
    // candidate pose into base_footprint via transformPose(), and returns
    // the farthest one that is still more than look_ahead_distance_ away
    // from the robot (the robot sits at the origin once expressed in its
    // own frame).
    geometry_msgs::msg::Pose getCarrotPose(const tf2::Transform & transform_tf);

};
}  // namespace robot_motion

#endif // PURE_PURSUIT_PLUGIN_HPP
