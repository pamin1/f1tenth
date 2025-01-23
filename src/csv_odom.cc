/**
 * @brief Odometry data collection to created localized map of waypoints on known track.
 * @author Prachit Amin
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <fstream>

class OdometrySubscriber : public rclcpp::Node
{
public:
    OdometrySubscriber()
        : Node("odometry_subscriber"), starting_position_set_(false), threshold_distance_(1.0), required_time_(5.0)
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",
            10,
            std::bind(&OdometrySubscriber::odometry_callback, this, std::placeholders::_1));
        last_log_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Odometry subscriber node started");
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        if (!starting_position_set_)
        {
            // Record the starting position and the time it was set
            starting_position_x_ = current_x;
            starting_position_y_ = current_y;
            start_time_ = this->now();
            starting_position_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting position set to: (%.2f, %.2f)", starting_position_x_, starting_position_y_);
            return;
        }

        // Calculate the Euclidean distance from the starting position
        double distance = std::sqrt(std::pow(current_x - starting_position_x_, 2) +
                                    std::pow(current_y - starting_position_y_, 2));

        auto now = this->now();
        if ((now - last_log_time_).seconds() >= 0.25)
        {
            RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f), Distance to start: %.2f meters",
                        current_x, current_y, distance);

            std::ofstream file("data.csv", std::ios_base::app);
            file << current_x << ", " << current_y << "\n";
            file.close();
            last_log_time_ = now;
        }

        // Check conditions: distance < threshold and elapsed time > required_time
        double elapsed_time = (this->now() - start_time_).seconds();
        if (distance < threshold_distance_ && elapsed_time > required_time_)
        {
            RCLCPP_INFO(this->get_logger(), "Robot is within %.2f meters of the starting point and %.2f seconds have elapsed. Triggering action!",
                        threshold_distance_, required_time_);
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    double starting_position_x_;
    double starting_position_y_;
    bool starting_position_set_;
    double threshold_distance_;
    double required_time_;
    rclcpp::Time start_time_;
    rclcpp::Time last_log_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometrySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
