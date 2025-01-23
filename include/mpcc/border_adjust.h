/**
 * @brief Header that contains just the function to adjust the borders used to build the MPCC problem.
 * @author Prachit Amin
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mpcc/ground_truth.h"

/**
 * @brief Subscribes to lidar data and updates a local map of the environment
 */
class Border : public rclcpp::Node
{
public:
    Border() : Node("map_updater")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Border::filter, this));
    }

private:
    void filter(sensor_msgs::msg::LaserScan::SharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

void Border::filter(sensor_msgs::msg::LaserScan::SharedPtr& msg) {
    std::vector<float> ranges = msg->ranges;
}