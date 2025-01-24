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

class CarFollowing : public rclcpp::Node
{
public:
    CarFollowing()
        : Node("odom_difference_node")
    {
        // Subscribe to ego and opponent odometry topics
        ego_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&CarFollowing::egoCallback, this, std::placeholders::_1));

        opp_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/opp_racecar/odom", 10,
            std::bind(&CarFollowing::oppCallback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CarFollowing::publishMessage, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    geometry_msgs::msg::Point ego_position_;
    geometry_msgs::msg::Point opp_position_;

    PID pidThrottle = PID(50.0);
    PID pidHeading = PID(50.0);

    double s_kP_dec = this->declare_parameter("steering_kP", 0.0);
    double s_kI_dec = this->declare_parameter("steering_kI", 0.0);
    double s_kD_dec = this->declare_parameter("steering_kD", 0.0);
    double t_kP_dec = this->declare_parameter("throttle_kP", 0.0);
    double t_kI_dec = this->declare_parameter("throttle_kI", 0.0);
    double t_kD_dec = this->declare_parameter("throttle_kD", 0.0);

    double s_kP = this->get_parameter("steering_kP").as_double();
    double s_kI = this->get_parameter("steering_kI").as_double();
    double s_kD = this->get_parameter("steering_kD").as_double();

    double t_kP = this->get_parameter("throttle_kP").as_double();
    double t_kI = this->get_parameter("throttle_kI").as_double();
    double t_kD= this->get_parameter("throttle_kD").as_double();

    

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

    void calculateDifference()
    {
        // Compute the Euclidean distance
        dx = ego_position_.x - opp_position_.x;
        dy = ego_position_.y - opp_position_.y;

        phi = atan(dx / dy);
        distance = std::sqrt(dx * dx + dy * dy);

        pidHeading.calculateError(0, phi); // maintain straight on heading with opponent car
        pidThrottle.calculateError(1, distance); // maintain distance of 1m in simulation
    }

    void publishMessage()
    {
        calculateDifference();
        // add clamping
        ackermann_msgs::msg::AckermannDriveStamped msg;
        double speed = pidThrottle.P(t_kP) + pidThrottle.I(t_kI) + pidThrottle.D(t_kD);
        double angle = pidHeading.P(s_kP) + pidHeading.I(s_kI) + pidHeading.D(t_kD);
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = speed;
        msg.drive.steering_angle = angle;
        // RCLCPP_INFO(this->get_logger(), "dx: %.2f\ndy: %.2f", dx, dy);
        RCLCPP_INFO(this->get_logger(), "\nEgo Speed: %0.2f\nOpp Heading: %0.2f\n-----\n", speed, angle);
        drive_publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarFollowing>());
    rclcpp::shutdown();
    return 0;
}