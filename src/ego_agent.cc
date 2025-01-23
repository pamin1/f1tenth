/**
 * @brief FTG approach for baseline ego_agent to traverse the map.
 * @author Prachit Amin
 */

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ReactiveFollowGap : public rclcpp::Node
{
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10,
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    float min;
    float inc;
    float angle;

    float rayClose;
    std::string lidarscan_topic = "/opp_scan";
    std::string drive_topic = "/opp_drive";
    /// TODO: create ROS subscribers and publishers

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry> &odom_subscriber_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    void publishMessage(float maxDist, float &heading)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        float speed = std::min(1.0, maxDist / 2.0);
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        if (heading < -1 || heading > 1)
        {
            heading = 0;
        }
        msg.drive.steering_angle = heading;
        msg.drive.speed = speed;
        drive_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "\nSpeed: %0.2f\nHeading: %0.2f\n-----\n", maxDist, heading);
    }

    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window (if not in sim)
        // 2.Rejecting high values (eg. > 3m)
        int count = 0;
        int sum = 0;
        std::vector<float> res(msg->ranges.size());
        for (auto &i : msg->ranges)
        {
            // auto &val = msg->ranges[i];
            if (i > 6.0)
            {
                res[count] = 6.0;
            }
            else
            {
                res[count] = i;
            }
            count++;
        }

        return res;
    }

    int find_max_gap(const std::vector<float> &ranges, int &gap_start, int &gap_end)
    {
        int range_size = ranges.size();
        int max_gap_size = 0;
        int current_start = -1;
        gap_start = 0;
        gap_end = 0;

        for (int i = 0; i < range_size; i++)
        {
            if (ranges[i] > 1.5)
            {
                if (current_start == -1)
                {
                    current_start = i;
                }
            }
            else if (current_start != -1)
            {
                int gap_size = i - current_start;
                if (gap_size > max_gap_size)
                {
                    max_gap_size = gap_size;
                    gap_start = current_start;
                    gap_end = i;
                }
                current_start = -1;
            }
        }
        RCLCPP_INFO(this->get_logger(), "gap_start: %d", gap_start);
        RCLCPP_INFO(this->get_logger(), "gap_end: %d", gap_end);
        return gap_start, gap_end;
    }

    int find_best_point(const std::vector<float> &ranges, int gap_start, int gap_end)
    {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there

        int best_point = gap_start;
        float max_distance = 0.0;
        for (int i = gap_start; i < gap_end; i++)
        {
            if (ranges[i] > max_distance)
            {
                max_distance = ranges[i];
                best_point = i;
            }
        }
        return best_point;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        // pass by reference with const, don't copy ranges into range.
        // preprocess_lidar can either take in the msg directly (recommend) or pass in ranges, min/max and increment

        std::vector<float> range = preprocess_lidar(msg);
        // std::cout << "Range Values:\n";
        // for (auto &i : range)
        // {
        //     std::cout << i << ", ";
        // }
        // std::cout << "\n";
        /// TODO:
        // Find closest point to LiDAR
        auto closest = std::min_element(range.begin(), range.end());
        auto closest_point_idx = std::find(range.begin(), range.end(), *closest);

        // Eliminate all points inside 'bubble' (set them to zero)
        int max = msg->angle_max;
        min = msg->angle_min;
        inc = msg->angle_increment;
        angle = min + (inc * *closest_point_idx);

        rayClose = *closest;

        fixRange(range, closest_point_idx);
        // Find max length gap
        int gap_start = 0;
        int gap_end = range.size() - 1;

        gap_start, gap_end = find_max_gap(range, gap_start, gap_end);
        // RCLCPP_INFO(this->get_logger(), "gap_start: %d", gap_start);
        // RCLCPP_INFO(this->get_logger(), "gap_end: %d", gap_end);

        // Find the best point in the gap
        int bestPoint = gap_start + (gap_end - gap_start) / 2;
        // RCLCPP_INFO(this->get_logger(), "bestPoint: %f", range[bestPoint]);
        // RCLCPP_INFO(this->get_logger(), "increment: %d", inc);

        float maxDist = range[bestPoint];
        // Publish Drive message
        float heading = min + (bestPoint * inc);

        publishMessage(maxDist, heading);
    }

    void fixRange(std::vector<float> range, std::vector<float>::iterator closest_point_idx)
    {
        for (int i = 0; i < range.size(); i++)
        {
            if (getHypotenuse(rayClose, range[i], angle) < 0.2)
            {
                range[i] = 0.0;
            }
            angle += inc * *closest_point_idx;
        }
    }

    float getComponents(float ray, float angle)
    {
        float X = ray * sin(angle);
        float Y = ray * cos(angle);
        return X, Y;
    }

    float getHypotenuse(float firstRay, float secondRay, float angle)
    {
        float x1, y1 = getComponents(firstRay, angle);
        float x2, y2 = getComponents(secondRay, angle);

        float xb = std::pow(x2 - x1, 2);
        float yb = std::pow(y2 - y1, 2);

        return std::sqrt(xb + yb);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}