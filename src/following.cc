/**
 * @brief FTG approach for baseline ego_agent to traverse the map.
 * @author Prachit Amin
 */

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "mpcc/pid.h"

class OdomDifferenceNode : public rclcpp::Node
{
public:
    OdomDifferenceNode()
        : Node("odom_difference_node")
    {
        // Subscribe to ego and opponent odometry topics
        ego_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&OdomDifferenceNode::egoCallback, this, std::placeholders::_1));

        opp_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/opp_racecar/odom", 10,
            std::bind(&OdomDifferenceNode::oppCallback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    geometry_msgs::msg::Point ego_position_;
    geometry_msgs::msg::Point opp_position_;

    PID pidThrottle;
    PID pidHeading;
    
    double dx;
    double dy;

    double phi;
    double distance;

    void egoCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ego_position_ = msg->pose.pose.position;
    }

    void oppCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        opp_position_ = msg->pose.pose.position;
    }

    // make publish odometry:
    // runs on fixed timer
    // create_timer -> get syntax
    // make a callback to this function
    // in this function:
    // take positions and calculateDifference()
    // calculate speed and angle
    // publish in the timer

    void calculateDifference()
    {
        // Compute the Euclidean distance
        dx = ego_position_.x - opp_position_.x;
        dy = ego_position_.y - opp_position_.y;

        phi = atan(dx / dy);
        distance = std::sqrt(dx * dx + dy * dy);

        pidHeading.calculateError(0, phi); // maintain straight on heading with opponent car
        pidThrottle.calculateError(1, distance); // maintain distance of 1m in simulation

        publishMessage();
    }

    void publishMessage()
    {
        // add clamping
        ackermann_msgs::msg::AckermannDriveStamped msg;
        double speed = pidThrottle.P(0.1) + pidThrottle.I(0.01) + pidThrottle.D(0.0);
        double angle = pidHeading.P(0.01) + pidHeading.I(0.0) + pidHeading.D(0.0);
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = speed;
        msg.drive.steering_angle = angle;
        RCLCPP_INFO(this->get_logger(), "\nSpeed: %.2f\nAngle: %.2f", speed, angle);
        // time stamp at at publish messages;
        
        drive_publisher_->publish(msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomDifferenceNode>());
    rclcpp::shutdown();
    return 0;
}