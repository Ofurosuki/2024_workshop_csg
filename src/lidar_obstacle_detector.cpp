#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#ifndef M_PI
#define M_PI 3.141592
#endif

class ObstacleDetectionNode : public rclcpp::Node {
 public:
  ObstacleDetectionNode() : Node("obstacle_detection_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10,
        std::bind(&ObstacleDetectionNode::pointCloudCallback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/whill/controller/cmd_vel", 10);
  }

 private:
  int points_threthold = 2000;
  float dist_threthold = 1.5;  // meter
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL data
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    bool obstacle_detected = false;
    int count = 0;
    ////////////////////////////////////////////////////////////////////////////
    /// loop through all the points in the point cloud//////////////////////////
    for (const auto &point : pcl_cloud.points) {
      if (point.x > 0) {
        if (sqrt(point.x * point.x + point.y * point.y) < dist_threthold) {
          count++;
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    RCLCPP_INFO(this->get_logger(), "Detected %d points", count);
    if (count > points_threthold) {
      obstacle_detected = true;
    }
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    if (!obstacle_detected) {
      cmd_vel_msg.linear.x = 300.0;
      cmd_vel_msg.angular.z = 0.0;
    } else {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
    }
    cmd_vel_publisher_->publish(cmd_vel_msg);
    // Publish the result
    auto output_msg = std_msgs::msg::Bool();
    output_msg.data = obstacle_detected;
    publisher_->publish(output_msg);
    count = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
