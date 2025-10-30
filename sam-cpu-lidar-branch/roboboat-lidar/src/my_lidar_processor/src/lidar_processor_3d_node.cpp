#include <memory> // adds shared_ptr , lets mutiple objects have ownership of a pointer. Useful for us since there are multiple subscriptions and publishers
#include "rclcpp/rclcpp.hpp" // ROS2 C++ Client Library
#include "sensor_msgs/msg/point_cloud2.hpp" // ROS2 message type for point clouds
#include "pcl_conversions/pcl_conversions.h" // Used for conversions between ROS2's PointCloud2 and PCL's PointCloud
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h" // Downsampling
#include "pcl/filters/passthrough.h" // Cropping filter
#include "pcl/segmentation/sac_segmentation.h" // Plane segmentation
#include "pcl/filters/extract_indices.h" // Remove water plane points
#include "pcl/segmentation/extract_clusters.h" // Object clustering
#include "pcl/common/centroid.h" // For computing cluster centroids

/*
Here is what I think is happening to our data each step of the way:
1. Receive the LiDAR data from subscriber - configurable topic from parameter
2. Voxelgrid - reduces the number of points by dividing space into cubes, then each voxel is represented by a single point
3. Passthrough filter - crops points, if there is a problem in the display of our pointcloud it might be cropped too much
4. Removing water plane with RANSAC, see the two seperate code blocks, one identifies the water plane then removes it
5. Clustering - groups points into clusters, where each object is its own object that the LiDAR sees
6. Exports Pointcloud back to ROS2 PointCloud2 and publishes on configurable topic
*/

class LidarProcessor3DNode : public rclcpp::Node
{
public:
    LidarProcessor3DNode()
    : Node("lidar_processor_3d_node")
    {
        // Declare parameters (with default values that match YAML)
        this->declare_parameter<std::string>("input_topic", "input_pointcloud");
        this->declare_parameter<std::string>("output_topic", "output_pointcloud");
        this->declare_parameter<std::string>("frame_id", "velodyne");
        this->declare_parameter<double>("voxel_leaf_size", 0.1); // meters
        this->declare_parameter<double>("passthrough_z_min", -1.0);
        this->declare_parameter<double>("passthrough_z_max", 2.0);
        this->declare_parameter<int>("sor_mean_k", 50); // unused for now, placeholder
        this->declare_parameter<double>("sor_stddev_mul", 1.0); // unused for now, placeholder
        this->declare_parameter<int>("queue_size", 10);

        // Load parameters into member variables
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("passthrough_z_min", passthrough_z_min_);
        this->get_parameter("passthrough_z_max", passthrough_z_max_);
        this->get_parameter("sor_mean_k", sor_mean_k_);
        this->get_parameter("sor_stddev_mul", sor_stddev_mul_);
        this->get_parameter("queue_size", queue_size_);

        // Subscribes to input topic (remappable via YAML or launch file)
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::QoS(queue_size_),
            std::bind(&LidarProcessor3DNode::pointCloudCallback, this, std::placeholders::_1));

        // Publishes to output topic (remappable via YAML or launch file)
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::QoS(queue_size_));

        RCLCPP_INFO(this->get_logger(),
                    "Lidar Processor Node started with voxel=%.2f, passthrough_z=[%.2f, %.2f]",
                    voxel_leaf_size_, passthrough_z_min_, passthrough_z_max_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Converts between pointclouds (i.e PointCloud2 and PointCloud)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // Apply voxel grid filter (downsample)
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        voxel.filter(*downsampled);

        // Apply passthrough filter (crop by height in Z) - We might need to edit this if we cant see enough
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(downsampled);
        pass.setFilterFieldName("z"); // crop along z-axis
        pass.setFilterLimits(passthrough_z_min_, passthrough_z_max_); 
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>());
        pass.filter(*cropped);

        // Segment dominant plane (water surface) using RANSAC
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1); // adjust for noise level
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cropped);
        seg.segment(*inliers, *coefficients);

        // Extract everything except the water plane
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cropped);
        extract.setIndices(inliers);
        extract.setNegative(true); // keep points NOT in the plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_water(new pcl::PointCloud<pcl::PointXYZI>());
        extract.filter(*no_water);

        // Cluster extraction for object analysis
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(no_water);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5); // meters between points in the same cluster
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(no_water);
        ec.extract(cluster_indices);

        int cluster_id = 0;
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr object(new pcl::PointCloud<pcl::PointXYZI>());
            for (int idx : indices.indices)
                object->points.push_back(no_water->points[idx]);

            // Compute centroid for this object
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*object, centroid);

            RCLCPP_INFO(this->get_logger(),
                        "Cluster %d centroid: [%.2f, %.2f, %.2f]",
                        cluster_id, centroid[0], centroid[1], centroid[2]);

            cluster_id++;
        }

        // Convert final processed cloud back to ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*no_water, output);
        output.header = msg->header; // Preserve frame + timestamp
        output.header.frame_id = frame_id_; // Use configured frame

        // Publish
        publisher_->publish(output);
    }

    // Parameters
    std::string input_topic_, output_topic_, frame_id_;
    double voxel_leaf_size_;
    double passthrough_z_min_, passthrough_z_max_;
    int sor_mean_k_;
    double sor_stddev_mul_;
    int queue_size_;

    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor3DNode>());
    rclcpp::shutdown();
    return 0;
}
