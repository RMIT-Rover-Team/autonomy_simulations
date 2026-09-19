#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "my_robot_planning/dijkstra_planner_plugin.hpp"
#include "rmw/qos_profiles.h"


namespace robot_planning
{

// Lifecycle methods
void DijkstraPlannerPlugin::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock(); // .lock() allows use to store a WeakPtr into a SharedPtr
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    smooth_cli_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "/smooth_path");
    if(!smooth_cli_->wait_for_action_server(std::chrono::seconds(3))){ // waits 3 sec for /smooth_path action server to be available
        RCLCPP_ERROR(node_->get_logger(), "path smoothing not available");
    }
}

void DijkstraPlannerPlugin::cleanup()
{
    RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin: %s of type DijkstraPlannerPlugin", name_.c_str());
}
void DijkstraPlannerPlugin::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating plugin: %s of type DijkstraPlannerPlugin", name_.c_str());
}
void DijkstraPlannerPlugin::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating plugin: %s of type DijkstraPlannerPlugin", name_.c_str());
}

// Main path planning logic method
nav_msgs::msg::Path DijkstraPlannerPlugin::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
    // 4-connected neighbor directions, matching the reference Python order.
    std::vector<std::pair<int, int>> explore_directions = {
        {-1, 0}, {1, 0}, {0, 1}, {0, -1}
    };

    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;

    // Cells that have already been popped and finalized. A cell can be
    // pushed onto the queue multiple times (once per neighbor that reaches
    // it); once it has been popped and finalized once, any later duplicate
    // pop of the same cell is stale and gets skipped (lazy deletion).
    std::unordered_set<GraphNode, GraphNodeHash> visited_nodes;

    GraphNode start_node = worldToGrid(start.pose);
    GraphNode goal_node = worldToGrid(goal.pose);
    pending_nodes.push(start_node);

    while (!pending_nodes.empty() && rclcpp::ok()) {
        GraphNode current_node = pending_nodes.top();
        pending_nodes.pop();

        if (visited_nodes.find(current_node) != visited_nodes.end()) {
            continue;
        }
        visited_nodes.insert(current_node);


        if (current_node == goal_node) {
            // Goal reached: reconstruct the path by walking backwards
            // through prev pointers from goal to start.
            nav_msgs::msg::Path path;
            path.header.frame_id = global_frame_;

            GraphNode tmp_node = current_node;
            while (tmp_node.prev && rclcpp::ok()) {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = global_frame_;
                pose_stamped.pose = gridToWorld(tmp_node);
                path.poses.push_back(pose_stamped);
                tmp_node = *tmp_node.prev;
            }

            // The loop above stops right before adding the start node
            // itself (its prev is null), so add it here explicitly.
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = global_frame_;
            pose_stamped.pose = gridToWorld(tmp_node);
            path.poses.push_back(pose_stamped);

            // Poses were appended goal-first; reverse to start-to-goal order.
            std::reverse(path.poses.begin(), path.poses.end());

            nav2_msgs::action::SmoothPath::Goal smooth_goal;
            smooth_goal.path = path;
            smooth_goal.check_for_collisions = true;
            smooth_goal.smoother_id = "simple_smoother";
            smooth_goal.max_smoothing_duration.sec = 10;
            auto future = smooth_cli_->async_send_goal(smooth_goal);
            if(future.wait_for(std::chrono::seconds(5)) == std::future_status::ready){
                auto goal_handle = future.get();
                if(goal_handle){
                    auto result_future = smooth_cli_->async_get_result(goal_handle);
                    if(result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready){
                        auto res = result_future.get();
                        if(res.code == rclcpp_action::ResultCode::SUCCEEDED){
                            path = res.result->path; // smooth path
                        }
                    }
                }
            }
            return path;
        }

        // Expand neighbors: for each direction, compute the neighboring
        // cell and, if it's a valid, obstacle-free cell, queue it up with
        // its cost and a back-pointer to this node.
        for (const auto & dir : explore_directions) {
            GraphNode new_node = current_node + dir;
            if (poseOnMap(new_node)) {
                int cell_cost = 0;
                if (isNavigable(new_node, cell_cost)) {
                    new_node.cost = current_node.cost + 1 + cell_cost;
                    new_node.prev = std::make_shared<GraphNode>(current_node);
                    pending_nodes.push(new_node);
                }
            }
        }
    }

    // Queue exhausted without reaching the goal: no path exists.
    return nav_msgs::msg::Path();
}

bool DijkstraPlannerPlugin::poseOnMap(const GraphNode & node)
{
    return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
        node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
}

bool DijkstraPlannerPlugin::isNavigable(const GraphNode & node, int & cost)
{
    cost = costmap_->getCost(node.x, node.y);
    return cost < 252;
}

GraphNode DijkstraPlannerPlugin::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    // std::floor (rather than truncation) ensures negative coordinates map
    // to the correct cell, e.g. -0.3 cells is still inside cell -1.
    int grid_x = static_cast<int>(std::floor(
        (pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()));
    int grid_y = static_cast<int>(std::floor(
        (pose.position.y - costmap_->getOriginY()) / costmap_->getResolution()));
    return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose DijkstraPlannerPlugin::gridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    return pose;
}
}  // namespace robot_planning

// common pkg in ROS2. Used to dynamically load/ unload plugins at runtime.
// provides modularity, flexibility of software, enabling the extension of software with additional functionalities
// here were are extending the nav2 planner with the dijkstra planner algorithm
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_planning::DijkstraPlannerPlugin, nav2_core::GlobalPlanner)