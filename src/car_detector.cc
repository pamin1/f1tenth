#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include "ultralytics_ros/msg/yolo_result.hpp"

class CarDetector : public rclcpp::Node
{
public:
  CarDetector() : Node("yolo_node")
  {
    yoloSub_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
        "/yolo_result", 10, std::bind(&CarDetector::yoloCallback, this, std::placeholders::_1));

    depthSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/depth", 10, std::bind(&CarDetector::depthCallback, this, std::placeholders::_1));

    depthPub_ = this->create_publisher<std_msgs::msg::Float32>(
        "depth", 10);
  }

  float getDepth()
  {
    return depth;
  }

private:
  void yoloCallback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Entered callback");
    for (size_t i = 0; i < msg->detections.detections.size(); i++)
    {
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
                    "Bounding box center: (x=%.2f, y=%.2f)",
                    bboxCenterX, bboxCenterY);
      }
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    float *depths = reinterpret_cast<float *>(&msg->data[0]);
    int bboxCenter = bboxCenterX + msg->width * bboxCenterY;
    depth = depths[bboxCenter];
  }

  void publishDepth() {
    std_msgs::msg::Float32 msg;
    msg.data = depth;
    depthPub_->publish(msg);
  }
  rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr yoloSub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depthPub_;

  int bboxCenterX;
  int bboxCenterY;
  float depth = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarDetector>());
  rclcpp::shutdown();
  return 0;
}