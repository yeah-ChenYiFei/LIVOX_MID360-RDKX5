#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class CloudFilterNode : public rclcpp::Node
{
public:
    CloudFilterNode() : Node("cloud_filter_node")
    {
        cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudFilterNode::cloud_callback, this, std::placeholders::_1)
        );

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Cloud filter node started (CustomMsg input)");
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // Manually convert CustomMsg -> PCL (no pcl::fromROSMsg for CustomMsg)
        PointCloudT::Ptr cloud_raw(new PointCloudT);
        cloud_raw->reserve(msg->points.size());
        for (auto &pt : msg->points) {
            cloud_raw->push_back(PointT(pt.x, pt.y, pt.z));
        }

        if (cloud_raw->empty()) return;

        RCLCPP_DEBUG(get_logger(), "raw: %ld pts", cloud_raw->size());

        PointCloudT::Ptr cloud_proc(new PointCloudT);

        // 1. Passthrough
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_raw);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.15, 1.6);  // ring bottom at ~30-40cm, top ~1.4m
        pass.filter(*cloud_proc);

        RCLCPP_DEBUG(get_logger(), "after passthrough: %ld pts", cloud_proc->size());

        // 2. Voxel
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_proc);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*cloud_proc);

        RCLCPP_DEBUG(get_logger(), "after voxel: %ld pts", cloud_proc->size());

        // 3. RANSAC ground removal
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);
        seg.setInputCloud(cloud_proc);
        seg.segment(*plane_inliers, *plane_coeff);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_proc);
        extract.setIndices(plane_inliers);
        extract.setNegative(true);
        extract.filter(*cloud_proc);

        RCLCPP_DEBUG(get_logger(), "after RANSAC: %ld pts (removed %ld ground)",
                    cloud_proc->size(), plane_inliers->indices.size());

        // 4. SOR disabled for thin ring
        // pcl::StatisticalOutlierRemoval<PointT> sor;
        // sor.setInputCloud(cloud_proc);
        // sor.setMeanK(30);
        // sor.setStddevMulThresh(1.0);
        // sor.filter(*cloud_proc);

        // Publish
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
