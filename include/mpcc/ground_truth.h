/**
 * @brief This header file allows us to poll and get ground truth position information from
 * the F1/10th Gym simulator.
 * @author Prachit Amin
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomListener : public rclcpp::Node
{
public:
    OdomListener() : Node("odom_listener")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&OdomListener::odom_callback, this, std::placeholders::_1));
    }
    double x, y, z, qx, qy, qz, qw; // linear terms
    double vx, vy, vz, wx, wy, wz;  // speed terms

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};


/**
 * @brief Stores the pose values from the sim into corresponding values in the class.
 * @param msg Odometry message from the sim
 */
void OdomListener::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    vz = msg->twist.twist.linear.z;

    wx = msg->twist.twist.angular.x;
    wy = msg->twist.twist.angular.y;
    wz = msg->twist.twist.angular.z;

    // RCLCPP_INFO(this->get_logger(), "Position -> x: [%f], y: [%f], z: [%f]", x, y, z);
    // RCLCPP_INFO(this->get_logger(), "Orientation -> qx: [%f], qy: [%f], qz: [%f], qw: [%f]", qx, qy, qz, qw);
}