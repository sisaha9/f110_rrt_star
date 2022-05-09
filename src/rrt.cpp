// This file contains implementations of all methods for RRT and RRT*
// Author: Yash Trikannad

#include "f110_rrt_star/rrt.h"
#include "f110_rrt_star/csv_reader.h"

#include <thread>
#include <chrono>

#include "tf2/utils.h"
#include "tf2/convert.h"
#include "nav2_map_server/map_io.hpp"

static const bool debug = true;

/// RRT Object Constructor
/// @param nh - node handle to the ros node
RRT::RRT(const rclcpp::NodeOptions & options) : rclcpp::Node("rrt_star_node", options), gen((std::random_device())())
{

    this->declare_parameter<std::string>("csv_path");
    auto csv_path = this->get_parameter("csv_path").as_string();
    this->declare_parameter<std::string>("map_path");
    auto map_path = this->get_parameter("map_path").as_string();
    auto map_status = nav2_map_server::loadMapFromYaml(map_path, input_map_);
    if (map_status)
    {
        RCLCPP_INFO(this->get_logger(), "Map Load Successful");
        map_cols_ = input_map_.info.width;
    }
    else
    {
        throw std::runtime_error("Map Load failed");
    }

    this->declare_parameter<std::string>("output_frame");
    output_frame_id_ = this->get_parameter("output_frame").as_string();
    this->declare_parameter<std::string>("input_frame");
    input_frame_id_ = this->get_parameter("input_frame_id").as_string();

    // Read Global Path
    f110::CSVReader reader(csv_path);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    global_path_ = reader.getData();

    // get transform from laser to map
    if (tf_buffer_->canTransform("map", input_frame_id_, tf2::TimePointZero))
    {
        tf_laser_to_output_ = tf_buffer_->lookupTransform("map", input_frame_id_, tf2::TimePointZero);
    }

    // Publishers and subscribers
    dynamic_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "dynamic_map",
        rclcpp::SensorDataQoS()
    );

    waypoint_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "waypoint_viz_marker",
        rclcpp::SensorDataQoS()
    );

    this->declare_parameter<bool>("publish_drive");
    publish_drive_ = this->get_parameter("publish_drive").as_bool();

    if (publish_drive_)
    {
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            rclcpp::SensorDataQoS()
        );
    }

    tree_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "tree_viz_marker",
        rclcpp::SensorDataQoS()
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&RRT::scan_callback, this, std::placeholders::_1)
    );

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose",
        rclcpp::SensorDataQoS(),
        std::bind(&RRT::pose_callback, this, std::placeholders::_1)
    );

    // Tree Visualization
    points_->header.frame_id = line_strip_->header.frame_id = output_frame_id_;
    points_->ns = "rrt_viz";
    points_->pose.orientation.w = line_strip_->pose.orientation.w = 1.0;
    points_->id = 0;
    line_strip_->id = 1;
    points_->type = visualization_msgs::msg::Marker::POINTS;
    line_strip_->type = visualization_msgs::msg::Marker::LINE_LIST;
    points_->scale.x = 0.1;
    points_->scale.y = 0.1;
    line_strip_->scale.x = 0.05;
    points_->color.g = 1.0f;
    points_->color.a = 1.0;
    line_strip_->color.r = 1.0;
    line_strip_->color.a = 1.0;

    // Waypoint Visualization
    unique_id_ = 0;

    // // Map Data
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    // RRT Parameters
    this->declare_parameter<int>("max__rrt_iters");
    max_rrt_iters_ = this->get_parameter("max_rrt_iters").as_int();
    this->declare_parameter<double>("max_expansion_distance");
    max_expansion_distance_ = this->get_parameter("max_expansion_distance").as_double();
    this->declare_parameter<int>("collision_checking_points");
    collision_checking_points_ = this->get_parameter("collision_checking_points").as_int();
    this->declare_parameter<double>("goal_tolerance");
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    this->declare_parameter<double>("lookahead_distance");
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    this->declare_parameter<double>("local_lookahead_distance");
    local_lookahead_distance_ = this->get_parameter("local_lookahead_distance").as_double();

    // Local Map Parameters
    this->declare_parameter<double>("inflation_radius");
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();

    // RRT* Parameters
    this->declare_parameter<bool>("enable_rrtstar");
    enable_rrtstar_ = this->get_parameter("enable_rrtstar").as_bool();
    this->declare_parameter<double>("search_radius");
    search_radius_ = this->get_parameter("search_radius").as_double();

    // Car Parameters
    this->declare_parameter<double>("high_speed");
    high_speed_ = this->get_parameter("high_speed").as_double();
    this->declare_parameter<double>("medium_speed");
    medium_speed_ = this->get_parameter("medium_speed").as_double();
    this->declare_parameter<double>("low_speed");
    low_speed_ = this->get_parameter("low_speed").as_double();

    RCLCPP_INFO(this->get_logger(), "Created new RRT Object.");
}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    if (tf_buffer_->canTransform(output_frame_id_, input_frame_id_, tf2::TimePointZero))
    {
        tf_laser_to_output_ = tf_buffer_->lookupTransform(output_frame_id_, input_frame_id_, tf2::TimePointZero);
    }

    const auto translation = tf_laser_to_output_.transform.translation;
    const double yaw = tf2::getYaw(tf_laser_to_output_.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msg->ranges.size()/6);
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*(start-1);

    for(int i=start; i<end; ++i)
    {
        theta+=angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        // laser hit x, y in base_link frame
        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > lookahead_distance_ || y_base_link > lookahead_distance_) continue;

        // laser hit x, y in base_link frame
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;
        
        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);
        for(const auto& index: index_of_expanded_obstacles)
        {
            if(input_map_.data[index] != 100)
            {
                input_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            }
        }
    }

    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 10)
    {
        for(const auto index: new_obstacles_)
        {
            input_map_.data[index] = 0;
        }
        
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Obstacles Cleared");
    }

    dynamic_map_pub_->publish(input_map_);
    RCLCPP_INFO(this->get_logger(), "Map Updated");
}

/// The pose callback when subscribed to particle filter's inferred pose (RRT* Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
{
    current_x_ = pose_msg->pose.position.x;
    current_y_ = pose_msg->pose.position.y;

    if(unique_id_ == 0)
    {
        visualize_waypoint_data();
    }

    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

    std::vector<TrackNode> tree;

    tree.emplace_back(TrackNode(pose_msg->pose.position.x, pose_msg->pose.position.y, 0.0, -1));

    int count = 0;
    while(count < max_rrt_iters_)
    {
        count++;

        // Sample a Node
        const auto sample_node = sample();

        // Check if it is not occupied
        if(is_collided(sample_node[0], sample_node[1]))
        {
            RCLCPP_DEBUG(this->get_logger(), "Sample Node Colliding");
            continue;
        }

        // Find the nearest node in the tree to the sample node
        const int nearest_node_id = nearest(tree, sample_node);

        // Steer the tree from the nearest node towards the sample node
        TrackNode new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);

        const auto current_node_index = tree.size();

        // Check if the segment joining the two nodes is in collision
        if(is_collided(tree[nearest_node_id], new_node))
        {
            RCLCPP_DEBUG(this->get_logger(), "Sample Node Edge Colliding");
            continue;
        }
        else if(enable_rrtstar_)
        {
            new_node.cost = cost(tree, new_node);
            const auto near_neighbour_indices = near(tree, new_node);

            std::vector<bool> is_near_neighbor_collided;
            int best_neighbor = new_node.parent_index;

            for(const int near_node_index: near_neighbour_indices)
            {
                if(is_collided(tree[near_node_index], new_node))
                {
                    is_near_neighbor_collided.push_back(true);
                    continue;
                }
                is_near_neighbor_collided.push_back(false);

                double cost = tree[near_node_index].cost + line_cost(tree[near_node_index], new_node);

                if(cost < new_node.cost)
                {
                    new_node.cost = cost;
                    new_node.parent_index = near_node_index;
                    best_neighbor = near_node_index;
                }
            }

            for(int i=0; i<near_neighbour_indices.size(); i++)
            {
                if(is_near_neighbor_collided[i] || i == best_neighbor)
                {
                    continue;
                }
                if(tree[near_neighbour_indices[i]].cost > new_node.cost + line_cost(
                        new_node, tree[near_neighbour_indices[i]]))
                {
                    RCLCPP_DEBUG(this->get_logger(), "Rewiring Parents");
                    tree[near_neighbour_indices[i]].parent_index = current_node_index;
                }
            }
        }
        tree.emplace_back(new_node);

        RCLCPP_DEBUG(this->get_logger(), "Sample Node Edge Found");
        if(is_goal(new_node, trackpoint[0], trackpoint[1]))
        {
            RCLCPP_DEBUG(this->get_logger(), "Goal reached. Backtracking ...");
            local_path_ = find_path(tree, new_node);
            RCLCPP_INFO(this->get_logger(), "Path Found");

            const auto trackpoint_and_distance =
                    get_best_local_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

            const auto local_trackpoint = trackpoint_and_distance.first;
            const double distance = trackpoint_and_distance.second;

            geometry_msgs::msg::PoseStamped goal_way_point;
            goal_way_point.pose.position.x = local_trackpoint[0];
            goal_way_point.pose.position.y = local_trackpoint[1];
            goal_way_point.pose.position.z = 0;
            goal_way_point.pose.orientation.x = 0;
            goal_way_point.pose.orientation.y = 0;
            goal_way_point.pose.orientation.z = 0;
            goal_way_point.pose.orientation.w = 1;

            geometry_msgs::msg::PoseStamped goal_way_point_car_frame;
            tf2::doTransform(goal_way_point, goal_way_point_car_frame, tf_output_to_laser_);

            const double steering_angle = 2*(goal_way_point_car_frame.pose.position.y)/pow(distance, 2);

            if (publish_drive_)
            {
                publish_corrected_speed_and_steering(steering_angle);
            }

            visualize_trackpoints(goal_way_point.pose.position.x, goal_way_point.pose.position.y,
                                  trackpoint[0], trackpoint[1]);

            break;
        }
    }
}

/// This method returns a sampled point from the free space
/// (restrict so that it only samples a small region of interest around the car's current position)
/// @return - the sampled point in free space
std::array<double, 2> RRT::sample()
{
    std::uniform_real_distribution<>::param_type x_param(0, lookahead_distance_);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_distance_, lookahead_distance_);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::msg::PoseStamped sample_point;
    sample_point.pose.position.x = x_dist(gen);
    sample_point.pose.position.y = y_dist(gen);
    sample_point.pose.position.z = 0;
    sample_point.pose.orientation.x = 0;
    sample_point.pose.orientation.y = 0;
    sample_point.pose.orientation.z = 0;
    sample_point.pose.orientation.w = 1;
    geometry_msgs::msg::PoseStamped sample_point_output;
    tf2::doTransform(sample_point, sample_point_output, tf_laser_to_output_);

    return {sample_point_output.pose.position.x, sample_point_output.pose.position.y};
}

/// This method returns the nearest node id on the tree to the sampled point
/// @param tree - the current RRT tree
/// @param sampled_point - the sampled point in free space
/// @return - id of nearest node on the tree
int RRT::nearest(const std::vector<TrackNode> &tree, const std::array<double, 2> &sampled_point)
{
    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();
    for(size_t i=0; i< tree.size(); i++)
    {
        const auto distance_sqr = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if(distance_sqr < nearest_node_distance)
        {
            nearest_node = i;
            nearest_node_distance = distance_sqr;
        }
    }
    return nearest_node;
}

/// The function steer:(x,y)->z returns a point such that z is “closer”
/// to y than x is. The point z returned by the function steer will be
/// such that z minimizes ||z−y|| while at the same time maintaining
/// ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
/// basically, expand the tree towards the sample point (within a max dist)
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param sampled_point - the sampled point in free space
/// @return - new node created from steering
TrackNode RRT::steer(const TrackNode &nearest_node, const int nearest_node_index, const std::array<double, 2> &sampled_point)
{
    const double x = sampled_point[0] - nearest_node.x;
    const double y = sampled_point[1] - nearest_node.y;
    const double distance = pow(x, 2) + pow(y, 2);

    TrackNode new_node{};
    if(distance < max_expansion_distance_)
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        const double theta = atan2(y, x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}

/// This method returns a boolean indicating if the path between the
/// nearest node and the new node created from steering is collision free
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param new_node - new node created from steering
/// @return - true if in collision, false otherwise
bool RRT::is_collided(const TrackNode &nearest_node, const TrackNode &new_node)
{
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(is_collided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}

/// This method checks if the latest node added to the tree is close
/// enough (defined by goal_threshold) to the goal so we can terminate
/// the search and find a path
/// @param latest_added_node - latest addition to the tree
/// @param goal_x - x coordinate of the current goal in map frame
/// @param goal_y - y coordinate of the current goal in map frame
/// @return - true if node close enough to the goal
bool RRT::is_goal(const TrackNode &latest_added_node, double goal_x, double goal_y)
{
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2)+pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance_;
}

/// This method traverses the tree from the node that has been determined
/// as goal
/// @param tree - the RRT tree
/// @param latest_added_node - latest addition to the tree that has been
/// determined to be close enough to the goal
/// @return - the vector that represents the order of the nodes traversed as the found path
std::vector<std::array<double, 2>> RRT::find_path(const std::vector<TrackNode> &tree, const TrackNode &latest_added_node)
{
    std::vector<std::array<double, 2>> found_path;
    TrackNode current_node = latest_added_node;

    while(current_node.parent_index != -1)
    {
        std::array<double, 2> local_trackpoint{current_node.x, current_node.y};
        found_path.emplace_back(local_trackpoint);
        current_node = tree[current_node.parent_index];
    }

    return found_path;
}

/// Checks if a sample in the workspace is colliding with an obstacle
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return true if the point is colliding with an obstacle
bool RRT::is_collided(const double x_map, const double y_map)
{
    
    const auto index = get_row_major_index(x_map, y_map);
    return input_map_.data[index] == 100;
}

/// This method returns the cost associated with a node
/// @param tree - the current tree
/// @param node - the node the cost is calculated for
/// @return - the cost value associated with the node
double RRT::cost(const std::vector<TrackNode> &tree, const TrackNode &new_node)
{
    return tree[new_node.parent_index].cost + line_cost(tree[new_node.parent_index], new_node);
}

/// This method returns the cost of the straight line path between two nodes
/// @param n1 - the Node at one end of the path
/// @param n2 - the Node at the other end of the path
/// @return - the cost value associated with the path (Euclidean Distance)
double RRT::line_cost(const TrackNode &n1, const TrackNode &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

/// This method returns the set of Nodes in the neighborhood of a node. (Not Implemented)
/// @param tree - the current tree
/// @param node - the node to find the neighborhood for
/// @return - the index of the nodes in the neighborhood
/// Can use this to increase the speed of search to log(n)
/// (S. Arya and D. M. Mount. Approximate range searching. Computational Geometry: Theory and Applications, 17:135–163, 2000.)
std::vector<int> RRT::near(const std::vector<TrackNode> &tree, const TrackNode &node)
{
    std::vector<int> near_neighbor_indices;
    for(int i=0; i<tree.size(); i++)
    {
        const double distance = sqrt(pow(node.x - tree[i].x, 2) + pow(node.y - tree[i].y, 2));
        if(distance < search_radius_)
        {
            near_neighbor_indices.push_back(i);
        }
    }
    return near_neighbor_indices;
}

/// Returns the row major index for the map
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
int RRT::get_row_major_index(const double x_map, const double y_map)
{
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    return y_index*map_cols_ + x_index;
}

/// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
std::vector<int> RRT::get_expanded_row_major_indices(const double x_map, const double y_map)
{
    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    for(int i=-inflation_radius_+x_index; i<inflation_radius_+1+x_index; i++)
    {
        for(int j=-inflation_radius_+y_index; j<inflation_radius_+1+y_index; j++)
        {
            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }
    return expanded_row_major_indices;
}

/// Returns the best way point from the global plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose)
{
    if (tf_buffer_->canTransform(input_frame_id_, output_frame_id_, tf2::TimePointZero))
    {
        tf_output_to_laser_ = tf_buffer_->lookupTransform(input_frame_id_, output_frame_id_, tf2::TimePointZero);
    }

    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for(int i=0; i<global_path_.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped goal_way_point;
        goal_way_point.pose.position.x = global_path_[i][0];
        goal_way_point.pose.position.y = global_path_[i][1];
        goal_way_point.pose.position.z = 0;
        goal_way_point.pose.orientation.x = 0;
        goal_way_point.pose.orientation.y = 0;
        goal_way_point.pose.orientation.z = 0;
        goal_way_point.pose.orientation.w = 1;
        geometry_msgs::msg::PoseStamped goal_way_point_output;
        tf2::doTransform(goal_way_point, goal_way_point_output, tf_output_to_laser_);

        if(goal_way_point_output.pose.position.x < 0) continue;

        double distance = std::abs(lookahead_distance_ -
                sqrt(pow(goal_way_point_output.pose.position.x, 2)+ pow(goal_way_point_output.pose.position.y, 2)));

        if(distance < best_trackpoint_distance)
        {
            const auto row_major_index = get_row_major_index(global_path_[i][0], global_path_[i][1]);
            if (input_map_.data[row_major_index] == 100) continue;
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    return global_path_[best_trackpoint_index];
}

/// Returns the best way point from the local plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::pair<std::array<double, 2>, double> RRT::get_best_local_trackpoint(const std::array<double, 2>& current_pose)
{
    std::array<double, 2> closest_point{};
    double closest_distance_to_current_pose = std::numeric_limits<double>::max();
    double closest_distance = std::numeric_limits<double>::max();
    for(const auto& trackpoint : local_path_)
    {
        double distance = sqrt(pow(trackpoint[0] - current_pose[0], 2)
                               + pow(trackpoint[1] - current_pose[1], 2));

        double diff_distance = std::abs(local_lookahead_distance_ - distance);
        if(diff_distance < closest_distance)
        {
            closest_distance_to_current_pose = distance;
            closest_distance = diff_distance;
            closest_point = trackpoint;
        }
    }
    return {closest_point, closest_distance_to_current_pose};
}

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
void RRT::add_way_point_visualization(const std::array<double, 2>& way_point, const std::string& frame_id, double r,
        double g, double b, double transparency = 0.5, double scale_x=0.2, double scale_y=0.2, double scale_z=0.2)
{
    visualization_msgs::msg::Marker::SharedPtr way_point_marker;
    way_point_marker->header.frame_id = frame_id;
    way_point_marker->header.stamp = this->now();
    way_point_marker->ns = "pure_pursuit";
    way_point_marker->id = unique_id_;
    way_point_marker->type = visualization_msgs::msg::Marker::SPHERE;
    way_point_marker->action = visualization_msgs::msg::Marker::ADD;
    way_point_marker->pose.position.x = way_point[0];
    way_point_marker->pose.position.y = way_point[1];
    way_point_marker->pose.position.z = 0;
    way_point_marker->pose.orientation.x = 0.0;
    way_point_marker->pose.orientation.y = 0.0;
    way_point_marker->pose.orientation.z = 0.0;
    way_point_marker->pose.orientation.w = 1.0;
    way_point_marker->scale.x = 0.2;
    way_point_marker->scale.y = 0.2;
    way_point_marker->scale.z = 0.2;
    way_point_marker->color.a = transparency;
    way_point_marker->color.r = r;
    way_point_marker->color.g = g;
    way_point_marker->color.b = b;
    waypoint_viz_pub_->publish(*way_point_marker);
    unique_id_++;
}

/// visualize all way points in the global path
void RRT::visualize_waypoint_data()
{
    const size_t increment = global_path_.size()/100;
    for(size_t i=0; i<global_path_.size(); i=i+increment)
    {
        add_way_point_visualization(global_path_[i], output_frame_id_, 1.0, 0.0, 1.0, 0.5);
    }
    RCLCPP_INFO(this->get_logger(), "Published All Global WayPoints.");
}

/// visualize trackpoints handles the creation and deletion of trackpoints
void RRT::visualize_trackpoints(double x_local, double y_local, double x_global, double y_global)
{
    visualization_msgs::msg::Marker::SharedPtr line;

    line->header.frame_id    = output_frame_id_;
    line->header.stamp       = this->now();
    line->lifetime           = rclcpp::Duration(0.1);
    line->id                 = 1;
    line->ns                 = "rrt";
    line->type               = visualization_msgs::msg::Marker::LINE_STRIP;
    line->action             = visualization_msgs::msg::Marker::ADD;
    line->pose.orientation.w = 1.0;
    line->scale.x            = 0.05;
    line->color.r            = 0.0f;
    line->color.g            = 0.0f;
    line->color.b            = 1.0f;
    line->color.a            = 1.0f;

    geometry_msgs::msg::Point current;
    geometry_msgs::msg::Point local;
    geometry_msgs::msg::Point global;

    current.x = current_x_;
    current.y = current_y_;
    current.z =  0;

    local.x = x_local;
    local.y = y_local;
    local.z = 0;

    global.x = x_global;
    global.y = y_global;
    global.z = 0;

    line->points.push_back(current);
    line->points.push_back(local);
    line->points.push_back(global);

    tree_viz_pub_->publish(*line);
    unique_id_++;
}

/// Returns the appropriate speed based on the steering angle
/// @param steering_angle
/// @return
void RRT::publish_corrected_speed_and_steering(double steering_angle)
{
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg;
    drive_msg->header.stamp = this->now();
    drive_msg->header.frame_id = output_frame_id_;

    drive_msg->drive.steering_angle = steering_angle;
    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg->drive.speed = low_speed_;
            if (steering_angle > 0.4)
            {
                drive_msg->drive.steering_angle = 0.4;
            }
        }
        else
        {
            drive_msg->drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg->drive.speed = low_speed_;
            if (steering_angle < -0.4)
            {
                drive_msg->drive.steering_angle = -0.4;
            }
        }
        else
        {
            drive_msg->drive.speed = medium_speed_;
        }
    }
    else
    {
        drive_msg->drive.speed = high_speed_;
    }

    drive_pub_->publish(*drive_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options{};
    auto node = std::make_shared<RRT>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}