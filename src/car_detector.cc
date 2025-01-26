#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include "ultralytics_ros/msg/yolo_result.hpp"

class CarDetector : public rclcpp::Node
{
public:
    CarDetector() : Node("yolo_node")
    {
        // Subscribe to image topic
        yoloSub_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
            "/yolo_result", 10, std::bind(&CarDetector::yoloCallback, this, std::placeholders::_1));
    }

private:
    void yoloCallback(const ultralytics_ros::msg::YoloResult::SharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Entered callback");
        for (const auto &detection : msg->detections)
        {
            for (const auto &result : detection.results)
            {
                // Extract class ID and score
                std::string class_id = result.hypothesis.class_id;
                float score = result.hypothesis.score;

                // Extract bounding box information
                bbox_center_x = detection.bbox.center.position.x;
                bbox_center_y = detection.bbox.center.position.y;

                // Log detection details
                RCLCPP_INFO(this->get_logger(), "Detected object: %s with confidence %.2f", class_id.c_str(), score);
                RCLCPP_INFO(this->get_logger(), "Bounding box center: (x=%.2f, y=%.2f)", bbox_center_x, bbox_center_y);
            }
        }
    }
    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr yoloSub_;
    float bbox_center_x;
    float bbox_center_y;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarDetector>());
    rclcpp::shutdown();
    return 0;
}