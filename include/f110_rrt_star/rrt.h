// This file contains the class definition of tree nodes and RRT
// This library implements rrt star referenced in https://arxiv.org/pdf/1105.1186.pdf
// Author : Yash Trikannad

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"

#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
// #include <boost/algorithm/string.hpp>
#include <random>

/// Struct defining the Node object in the RRT tree.
struct TrackNode
{
    TrackNode() = default;
    TrackNode(const double x, const double y, const int parent_index) :
        x(x), y(y), cost(0.0), parent_index(parent_index)
    {}

    TrackNode(const double x, const double y, double cost, const int parent_index) :
            x(x), y(y), cost(0.0), parent_index(parent_index)
    {}

    double x;
    double y;
    double cost;
    int parent_index;
};


/// RRT Class used for searching across the graph
class RRT : public rclcpp::Node {
public:
    explicit RRT(const rclcpp::NodeOptions & options);

private:
    /// Publisher and Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    /// Visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_viz_pub_;
    visualization_msgs::msg::Marker::SharedPtr points_;
    visualization_msgs::msg::Marker::SharedPtr line_strip_;
    int unique_id_;

    /// Transformations
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_laser_to_output_;
    geometry_msgs::msg::TransformStamped tf_output_to_laser_;

    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    /// Map
    nav_msgs::msg::OccupancyGrid input_map_;
    int map_cols_;
    std::vector<size_t > new_obstacles_;
    int clear_obstacles_count_;
    int inflation_radius_;

    /// RRT Parameters
    int max_rrt_iters_;
    double max_expansion_distance_;
    int collision_checking_points_;
    double goal_tolerance_;
    double local_trackpoint_tolerance_;
    double lookahead_distance_;
    double local_lookahead_distance_;

    /// RRT* Parameter
    bool enable_rrtstar_;

    /// Car Parameters
    double high_speed_;
    double medium_speed_;
    double low_speed_;

    /// Pose (Map Frame)
    double current_x_;
    double current_y_;

    /// Global Path
    std::vector<std::array<double, 2>> global_path_;
    std::vector<std::array<double, 2>> local_path_;

    // Node parameters
    std::string output_frame_id_;
    std::string input_frame_id_;
    bool received_global_map_;
    bool publish_drive_;

    /// Near search radius for a node in the RRT* tree
    double search_radius_;

    /// The pose callback when subscribed to particle filter's inferred pose (RRT Main Loop)
    /// @param pose_msg - pointer to the incoming pose message
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);

    /// The scan callback, update your occupancy grid here
    /// @param scan_msg - pointer to the incoming scan message
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// RRT methods

    /// This method returns a sampled point from the free space
    /// (restrict so that it only samples a small region of interest around the car's current position)
    /// @return - the sampled point in free space
    std::array<double, 2> sample();

    /// This method returns the nearest node on the tree to the sampled point
    /// @param tree - the current RRT tree
    /// @param sampled_point - the sampled point in free space
    /// @return - index of nearest node on the tree
    static int nearest(const std::vector<TrackNode> &tree, const std::array<double, 2> &sampled_point);

    /// The function steer:(x,y)->z returns a point such that z is “closer”
    /// to y than x is. The point z returned by the function steer will be
    /// such that z minimizes ||z−y|| while at the same time maintaining
    /// ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
    /// basically, expand the tree towards the sample point (within a max dist)
    /// @param nearest_node - nearest node on the tree to the sampled point
    /// @param sampled_point - the sampled point in free space
    /// @return - new node created from steering
    TrackNode steer(const TrackNode &nearest_node, int nearest_node_index, const std::array<double, 2> &sampled_point);

    /// This method returns a boolean indicating if the path between the
    /// nearest node and the new node created from steering is collision free
    /// @param nearest_node - nearest node on the tree to the sampled point
    /// @param new_node - new node created from steering
    /// @return - true if in collision, false otherwise
    bool is_collided(const TrackNode &nearest_node, const TrackNode &new_node);

    /// This method checks if the latest node added to the tree is close
    /// enough (defined by goal_threshold) to the goal so we can terminate
    /// the search and find a path
    /// @param latest_added_node - latest addition to the tree
    /// @param goal_x - x coordinate of the current goal
    /// @param goal_y - y coordinate of the current goal
    /// @return - true if node close enough to the goal
    bool is_goal(const TrackNode &latest_added_node, double goal_x, double goal_y);

    /// This method traverses the tree from the node that has been determined
    /// as goal
    /// @param tree - the RRT tree
    /// @param latest_added_node - latest addition to the tree that has been
    /// determined to be close enough to the goal
    /// @return - the vector that represents the order of the nodes traversed as the found path
    static std::vector<std::array<double, 2>> find_path(const std::vector<TrackNode> &tree, const TrackNode &latest_added_node);

    /// RRT* methods

    /// This method returns the cost associated with a node
    /// @param tree - the current tree
    /// @param node - the node the cost is calculated for
    /// @return - the cost value associated with the node
    double cost(const std::vector<TrackNode> &tree, const TrackNode &node);

    /// This method returns the cost of the straight line path between two nodes (Not Implemented)
    /// @param n1 - the Node at one end of the path
    /// @param n2 - the Node at the other end of the path
    /// @return - the cost value associated with the path
    double line_cost(const TrackNode &n1, const TrackNode &n2);

    /// (Not Implemented)
    /// This method returns the set of Nodes in the neighborhood of a node. (Not Implemented)
    /// @param tree - the current tree
    /// @param node - the node to find the neighborhood for
    /// @return - the index of the nodes in the neighborhood
    /// Can use this to increase the speed of search to log(n)
    /// (S. Arya and D. M. Mount. Approximate range searching. Computational Geometry: Theory and Applications)
    std::vector<int> near(const std::vector<TrackNode> &tree, const TrackNode &node);

    /// Checks if a sample in the workspace is colliding with an obstacle
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return true if the point is colliding with an obstacle
    bool is_collided(double x_map, double y_map);

    /// Map Manipulations

    /// Returns the row major index for the map
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    int get_row_major_index(double x, double y);

    /// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    std::vector<int> get_expanded_row_major_indices(const double x_map, const double y_map);

    /// Path Tracking

    /// Returns the best way point from the global plan to track
    /// @param current_pose - current pose (x, y) of the car in map frame
    /// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
    /// @return trackpoint (x, y) in map frame
    std::array<double, 2> get_best_global_trackpoint(const std::array<double, 2>& current_pose);

    /// Returns the best way point from the local plan to track
    /// @param current_pose - current pose (x, y) of the car in map frame
    /// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
    /// @return trackpoint (x, y) in map frame
    std::pair<std::array<double, 2>, double> get_best_local_trackpoint(const std::array<double, 2>& current_pose);

    /// Returns the appropriate speed based on the steering angle
    /// @param steering_angle
    /// @return
    void publish_corrected_speed_and_steering(double steering_angle);

    /// Visualization

    /// Publish way point with respect to the given input properties
    /// @param way_point - (x, y) wrt to the frame id
    /// @param frame_id - frame represented in waypoint
    /// @param r - red intensity
    /// @param g - green intensity
    /// @param b - blue intenisty
    /// @param transparency (Do not put 0)
    /// @param scale_x
    /// @param scale_y
    /// @param scale_z
    void add_way_point_visualization(const std::array<double, 2>& point, const std::string& frame_id, double r, double g,
            double b, double transparency, double scale_x, double scale_y, double scale_z);

    /// visualize all way points in the global path
    void visualize_waypoint_data();

    /// visualize trackpoints handles the creation and deletion of trackpoints
    void visualize_trackpoints(double x, double y, double global_x, double global_y);
};

