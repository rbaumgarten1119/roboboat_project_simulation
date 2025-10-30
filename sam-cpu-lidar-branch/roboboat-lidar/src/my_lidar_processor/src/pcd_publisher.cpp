// pcd_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("pcd_publisher") {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("pcd_dir", "/tmp/pcd_samples");
    this->declare_parameter<std::string>("output_topic", "/velodyne_points");
    this->declare_parameter<int>("rate_hz", 2);

    // Load parameters
    this->get_parameter("pcd_dir", pcd_dir_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("rate_hz", rate_hz_);

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(10));

    // Timer to cycle through files
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / std::max(1, rate_hz_)),
      std::bind(&PCDPublisher::on_timer, this));

    // Scan the directory for .pcd files
    for (const auto &entry : std::filesystem::directory_iterator(pcd_dir_)) {
      if (entry.path().extension() == ".pcd") {
        file_list_.push_back(entry.path().string());
      }
    }
    std::sort(file_list_.begin(), file_list_.end());

    if (file_list_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No .pcd files found in %s", pcd_dir_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Found %zu PCD files in %s",
                  file_list_.size(), pcd_dir_.c_str());
    }
  }

private:
  void on_timer() {
    if (file_list_.empty()) return;

    const std::string &file = file_list_[idx_ % file_list_.size()];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load %s", file.c_str());
      idx_++;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %s with %lu points",
                file.c_str(), cloud->points.size());

    // Convert to ROS2 message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = "velodyne";  // matches RViz fixed frame

    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published %s", file.c_str());

    idx_++;
  }

  // Parameters
  std::string pcd_dir_, output_topic_;
  int rate_hz_;

  // ROS objects
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // File management
  std::vector<std::string> file_list_;
  size_t idx_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDPublisher>());
  rclcpp::shutdown();
  return 0;
}
