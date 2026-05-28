#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class CloudFilterNode : public rclcpp::Node
{
public:
    CloudFilterNode() : Node("cloud_filter_node")
    {
        x_min_ = declare_parameter("x_min", 0.0);
        x_max_ = declare_parameter("x_max", 1.8);
        y_min_ = declare_parameter("y_min", -0.7);
        y_max_ = declare_parameter("y_max", 0.7);
        z_min_ = declare_parameter("z_min", -0.6);
        z_max_ = declare_parameter("z_max", 1.5);

        voxel_leaf_ = declare_parameter("voxel_leaf", 0.02);

        sor_enabled_ = declare_parameter("sor_enabled", true);
        sor_mean_k_  = declare_parameter("sor_mean_k", 20);
        sor_stddev_  = declare_parameter("sor_stddev", 1.0);

        cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudFilterNode::cloud_callback, this, std::placeholders::_1)
        );

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud", 10
        );

        RCLCPP_INFO(this->get_logger(),
            "Cloud filter: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f] voxel=%.2f SOR=%s",
            x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
            voxel_leaf_, sor_enabled_ ? "on" : "off");
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double voxel_leaf_;
    bool sor_enabled_;
    int sor_mean_k_;
    double sor_stddev_;

    void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        PointCloudT::Ptr cloud_raw(new PointCloudT);
        cloud_raw->reserve(msg->points.size());
        for (auto &pt : msg->points) {
            cloud_raw->push_back(PointT(pt.x, pt.y, pt.z));
        }

        if (cloud_raw->empty()) return;

        PointCloudT::Ptr cloud_proc(new PointCloudT);

        // 1. Passthrough
        pcl::PassThrough<PointT> pass;

        pass.setInputCloud(cloud_raw);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min_, x_max_);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min_, y_max_);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud_proc);

        if (cloud_proc->empty()) return;

        // 2. Voxel downsampling
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_proc);
        vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        vg.filter(*cloud_proc);

        if (cloud_proc->empty()) return;

        // 3. SOR — denoise
        if (sor_enabled_) {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud_proc);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*cloud_proc);
        }

        // Publish (RANSAC removal moved to shape_detect after fusion)
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud_proc, out_msg);
        out_msg.header.stamp = this->now();
        out_msg.header.frame_id = "base_link";
        cloud_pub_->publish(out_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
