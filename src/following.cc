/**
 * @brief FTG approach for baseline algorithm to follow an opponent car
 * @author Prachit Amin
 */

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
    bool validDetection;
    std::vector<float> differences;

    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    void publishMessage(float maxDist, float &heading)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        // msg.header.stamp = this->get_clock()->now();

        float speed = 1;
       
        msg.drive.steering_angle = heading;
        msg.drive.speed = speed;

        drive_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "\nSpeed: %0.2f\nHeading: %0.2f\n-----\n", maxDist, heading);
    }

    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        int count = 0;
        int sum = 0;
        std::vector<float> res(msg->ranges.size());
        for (auto &i : msg->ranges)
        {
            if (i > 3) // rejecting data points greater than 3.0
            {
                res[count] = 3;
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
        // RCLCPP_INFO(this->get_logger(), "gap_start: %d", gap_start);
        // RCLCPP_INFO(this->get_logger(), "gap_end: %d", gap_end);
        return gap_start, gap_end;
    }

    int find_best_point(const std::vector<float> &ranges, int gap_start, int gap_end)
    {
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

        std::vector<float> range = preprocess_lidar(msg);
        std::vector<float> orig = msg->ranges; // copy vector for detection purposes

        // modifying orig to get a narrowed vision of obstacles in front of the car
        int mid = std::floor(orig.size() / 2);
        size_t width = 35; // Total of 2*width + 1 elements (center + both sides)

        // Ensure bounds are valid
        size_t start = (mid >= width) ? mid - width : 0;
        size_t end = std::min(mid + width + 1, orig.size());

        // Initialize a new vector using a range constructor
        std::vector<float> new_vec(orig.begin() + start, orig.begin() + end);

        double sum = std::accumulate(new_vec.begin(), new_vec.end(), 0.0);
        double average = sum / new_vec.size(); // naive approach to check if there is an obstacle
        if (average < 1.5)
        {
            // RCLCPP_INFO(this->get_logger(), "CAR DETECTED!");
            validDetection = true;
        }

        auto closest = std::min_element(range.begin(), range.end());
        auto closest_point_idx = std::find(range.begin(), range.end(), *closest);

        // back to FTG
        // Eliminate all points inside 'bubble' (set them to zero)
        int max = msg->angle_max;
        min = msg->angle_min;
        inc = msg->angle_increment;

        // RCLCPP_INFO(this->get_logger(), "min: %0.2f\nmax: %0.2f\ninc: %0.2f", min, max, inc);

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
