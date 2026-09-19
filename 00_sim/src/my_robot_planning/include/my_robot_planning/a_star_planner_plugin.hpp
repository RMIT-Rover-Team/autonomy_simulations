#ifndef A_STAR_PLANNER_PLUGIN_HPP
#define A_STAR_PLANNER_PLUGIN_HPP

#include <functional>
#include <memory>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

namespace robot_planning
{
struct GraphNode
{
    int x;
    int y;
    int cost;          // g(n): steps taken so far to reach this cell.
    double heuristic;   // h(n): estimated remaining distance to the goal.
    std::shared_ptr<GraphNode> prev;

    GraphNode() : GraphNode(0, 0) {}

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0), heuristic(0.0) {}

    // Used by the priority queue: the node with the lowest f(n) = cost +
    // heuristic is explored first.
    bool operator>(const GraphNode & other) const
    {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }

    // Equality is based on grid position ONLY (not cost/heuristic/prev).
    // This mirrors the reference Python implementation, where two
    // GraphNode instances at the same cell are treated as "the same node"
    // for visited-tracking purposes.
    bool operator==(const GraphNode & other) const
    {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(std::pair<int, int> const & other) const
    {
        return GraphNode(x + other.first, y + other.second);
    }
};

// Hash based on (x, y) only, consistent with operator==, so GraphNode can be
// stored in an unordered_set to track which cells have been finalized.
struct GraphNodeHash
{
    std::size_t operator()(const GraphNode & node) const
    {
        std::size_t hx = std::hash<int>()(node.x);
        std::size_t hy = std::hash<int>()(node.y);
        return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2));
    }
};

class AStarPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
    AStarPlannerPlugin() = default;
    ~AStarPlannerPlugin() = default;

    // Lifecycle methods
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // Main path planning logic method
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker) override;

private:

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_frame_, name_;

    rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_cli_;


    bool poseOnMap(const GraphNode & node);

    // Writes the cell's map cost into `cost` and returns true if the cell is
    // free of obstacles (0 <= cost < 252), false otherwise.
    bool isNavigable(const GraphNode & node, int & cost);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);

    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);

    // Admissible, consistent heuristic for a 4-connected grid with unit
    // step cost.
    double manhattanDistance(const GraphNode & node1, const GraphNode & node2);
};
}  // namespace robot_planning

#endif // A_STAR_PLANNER_PLUGIN_HPP
