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
#include "sensor_msgs/msg/image.hpp"
#include "ultralytics_ros/msg/yolo_result.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/*
 TODO: Test throttle and steering control on physical implementation
       Collect rosbag dataset
       Improve throttle control loop with and without detections
       Make the code more modular; less functions definitions in this file
*/

class CarFollowing : public rclcpp::Node
{
public:
  CarFollowing()
      : Node("odom_difference_node")
  {
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10,
        std::bind(&CarFollowing::lidarCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(25), std::bind(&CarFollowing::publishMessage, this));
  }

private:
  // inputs:  yolo topic, image depth data, lidar
  // outputs: ackermann message with throttle and steering control
  void yoloCallback(const ultralytics_ros::msg::YoloResult::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishMessage();

  // Follow the Gap methods
  std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg);
  std::tuple<int, int> find_max_gap(const std::vector<float> &ranges, int &gap_start, int &gap_end);
  int find_best_point(const std::vector<float> &ranges, int gap_start, int gap_end);
  void fixRange(std::vector<float> range, std::vector<float>::iterator closest_point_idx);
  float getHypotenuse(float firstRay, float secondRay, float angle);
  std::tuple<float, float> getComponents(float ray, float angle);

  // publishing on timer to wait for input topic data to populate
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
  rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr yoloSub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

  std::string lidarscan_topic = "/opp_scan";
  std::string drive_topic = "/opp_drive";

  float min;
  float inc;
  float angle;
  float rayClose;
  float maxDist;
  float heading;
  int bboxCenterX;
  int bboxCenterY;
  double depth;
  bool foundDetections = false;
};

void CarFollowing::yoloCallback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Entered callback");
  for (size_t i = 0; i < msg->detections.detections.size(); i++)
  {
    foundDetections = true;
    const auto &detection = msg->detections.detections[i];
    for (size_t j = 0; j < detection.results.size(); j++)
    {
      const auto &result = detection.results[j];
      std::string class_id = result.hypothesis.class_id;
      float score = result.hypothesis.score;

      bboxCenterX = detection.bbox.center.position.x;
      bboxCenterY = detection.bbox.center.position.y;

      RCLCPP_INFO(this->get_logger(),
                  "Detected object: %s with confidence %.2f",
                  class_id.c_str(), score);
      RCLCPP_INFO(this->get_logger(),
                  "Bounding box center: (x=%d, y=%d)",
                  bboxCenterX, bboxCenterY);
    }
  }
}

void CarFollowing::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  float *depths = reinterpret_cast<float *>(&msg->data[0]);
  int bboxCenter = bboxCenterX + msg->width * bboxCenterY;
  depth = depths[bboxCenter];
}

void CarFollowing::publishMessage()
{
  ackermann_msgs::msg::AckermannDriveStamped msg;

  double speed = 0;

  // simple control loop to restrain throttle if another car was detected
  if (foundDetections)
  {
    speed = 0.1;
  }
  else
  {
    speed = std::min(1.0, maxDist / 2.0);
  }

  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
  msg.drive.speed = speed;
  msg.drive.steering_angle = heading;

  drive_publisher_->publish(msg);
}

std::vector<float> CarFollowing::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
{
  // Preprocess the LiDAR scan array. Expert implementation includes:
  // 1.Setting each value to the mean over some window (if not in sim)
  // 2.Rejecting high values (eg. > 3m)
  int count = 0;
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

std::tuple<int, int> CarFollowing::find_max_gap(const std::vector<float> &ranges, int &gap_start, int &gap_end)
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
  return std::make_tuple(gap_start, gap_end);
}

int CarFollowing::find_best_point(const std::vector<float> &ranges, int gap_start, int gap_end)
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

void CarFollowing::fixRange(std::vector<float> range, std::vector<float>::iterator closest_point_idx)
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

std::tuple<float, float> CarFollowing::getComponents(float ray, float angle)
{
  float X = ray * sin(angle);
  float Y = ray * cos(angle);
  return std::make_tuple(X, Y);
}

float CarFollowing::getHypotenuse(float firstRay, float secondRay, float angle)
{
  float x1, y1, x2, y2;
  std::tie(x1, y1) = getComponents(firstRay, angle);
  std::tie(x2, y2) = getComponents(secondRay, angle);

  float xb = std::pow(x2 - x1, 2);
  float yb = std::pow(y2 - y1, 2);

  return std::sqrt(xb + yb);
}

void CarFollowing::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
  // pass by reference with const, don't copy ranges into range.

  std::vector<float> range = preprocess_lidar(msg);
  auto closest = std::min_element(range.begin(), range.end());
  auto closest_point_idx = std::find(range.begin(), range.end(), *closest);

  // Eliminate all points inside 'bubble' (set them to zero)
  min = msg->angle_min;
  inc = msg->angle_increment;
  angle = min + (inc * *closest_point_idx);

  rayClose = *closest;
  fixRange(range, closest_point_idx);

  // Find max length gap
  int gap_start = 0;
  int gap_end = range.size() - 1;

  std::tie(gap_start, gap_end) = find_max_gap(range, gap_start, gap_end);
  int bestPoint = gap_start + (gap_end - gap_start) / 2;
  maxDist = range[bestPoint];

  // Publish Drive message
  heading = min + (bestPoint * inc);

  publishMessage();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarFollowing>());
  rclcpp::shutdown();
  return 0;
}