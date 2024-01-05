#include "pointcloud_downsampling.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

namespace pointcloud_downsampling
{
    std::string subscribed_topic, published_topic, pub_cloud_frame;
    float leaf_size_x, leaf_size_y, leaf_size_z;
    bool downsample_all_data;

    PointcloudDownsampling::PointcloudDownsampling(const rclcpp::NodeOptions &options)
        : Node("pointcloud_downsampling", options)
    {
        RCLCPP_INFO(get_logger(), "Start PointcloudDownsampling!");

        getParams();
        
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(published_topic, 10);

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            subscribed_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointcloudDownsampling::voxelFiltering, this, std::placeholders::_1));
    }

    PointcloudDownsampling::~PointcloudDownsampling() {}

    void PointcloudDownsampling::voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // Voxel filtering
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        sor.setDownsampleAllData(downsample_all_data);
        sor.filter(*pcl_cloud_filtered);

        sensor_msgs::msg::PointCloud2 ros2_cloud_filtered;
        pcl::toROSMsg(*pcl_cloud_filtered, ros2_cloud_filtered);
        RCLCPP_DEBUG(get_logger(), "before filter: %d", msg->width);
        RCLCPP_DEBUG(get_logger(), "after filter: %d", ros2_cloud_filtered.width);
        publish_result(ros2_cloud_filtered);
    }

    void PointcloudDownsampling::publish_result(sensor_msgs::msg::PointCloud2 msg)
    {
        msg.header.frame_id = "livox_frame";
        msg.header.stamp = get_clock()->now();
        msg.is_dense = true;
        pointcloud_pub_->publish(msg);
    }

    void PointcloudDownsampling::getParams()
    {
        declare_parameter<std::string>("sub_topic", "/cloud_registered_body");
        declare_parameter<std::string>("pub_topic", "/cloud_registered_body_downsampling");
        declare_parameter<std::string>("pub_cloud_frame", "livox_frame");
        declare_parameter<float>("leaf_size_x", 0.2);
        declare_parameter<float>("leaf_size_y", 0.2);
        declare_parameter<float>("leaf_size_z", 0.2);
        declare_parameter<bool>("downsample_all_data", false);

        get_parameter_or<std::string>("sub_topic", subscribed_topic, "/cloud_registered_body");
        get_parameter_or<std::string>("pub_topic", published_topic, "/cloud_registered_body_downsampling");
        get_parameter_or<std::string>("pub_cloud_frame", pub_cloud_frame, "livox_frame");
        get_parameter_or<float>("leaf_size_x", leaf_size_x, 0.2);
        get_parameter_or<float>("leaf_size_y", leaf_size_y, 0.2);
        get_parameter_or<float>("leaf_size_z", leaf_size_z, 0.2);
        get_parameter_or<bool>("downsample_all_data", downsample_all_data, false);

        RCLCPP_INFO(this->get_logger(), "subscribed_topic %s", subscribed_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "published_topic %s", published_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "leaf_size_x %f", leaf_size_x);
        RCLCPP_INFO(this->get_logger(), "leaf_size_y %f", leaf_size_y);
        RCLCPP_INFO(this->get_logger(), "leaf_size_z %f", leaf_size_z);
        RCLCPP_INFO(this->get_logger(), "downsample_all_data %s", downsample_all_data ? "true" : "false");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_downsampling::PointcloudDownsampling)
