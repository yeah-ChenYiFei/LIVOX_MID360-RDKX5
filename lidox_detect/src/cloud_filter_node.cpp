#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// 【修改点1】：替换为 pcl_conversions 头文件
#include <pcl_conversions/pcl_conversions.h> 

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class CloudFilterNode : public rclcpp::Node
{
public:
    CloudFilterNode() : Node("cloud_filter_node")
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",
            10,
            std::bind(&CloudFilterNode::cloud_callback, this, std::placeholders::_1)
        );

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud",
            10
        );

        RCLCPP_INFO(this->get_logger(), "✅ Cloud filter node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudT::Ptr cloud_raw(new PointCloudT);
        
        // 【修改点2】：使用 pcl::fromROSMsg
        pcl::fromROSMsg(*msg, *cloud_raw);  
        if (cloud_raw->empty()) return;

        PointCloudT::Ptr cloud_proc(new PointCloudT);

        // 1. 直通滤波
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_raw);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-6.0, 6.0);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-6.0, 6.0);
        pass.filter(*cloud_proc);

        pass.setInputCloud(cloud_proc);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.1, 2.5);  // 圆环顶部 2.2m，留余量到 2.5m
        pass.filter(*cloud_proc);

        // 2. 体素降采样
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_proc);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*cloud_proc);

        // 3. RANSAC 地面分割
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud_proc);
        seg.segment(*plane_inliers, *plane_coeff);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_proc);
        extract.setIndices(plane_inliers);
        extract.setNegative(true);
        extract.filter(*cloud_proc);

        // 4. 统计去噪（薄圆环点数稀疏，禁用 SOR 避免误删）
        // pcl::StatisticalOutlierRemoval<PointT> sor;
        // sor.setInputCloud(cloud_proc);
        // sor.setMeanK(30);
        // sor.setStddevMulThresh(1.0);
        // sor.filter(*cloud_proc);

        // 转回 ROS2 消息
        sensor_msgs::msg::PointCloud2 out_msg;
        
        // 【修改点3】：使用 pcl::toROSMsg
        pcl::toROSMsg(*cloud_proc, out_msg);  
        out_msg.header = msg->header;
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
