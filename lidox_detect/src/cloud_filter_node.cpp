#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


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
        // ── passthrough bounds ──────────────────────────
        x_min_ = declare_parameter("x_min", 0.0);
        x_max_ = declare_parameter("x_max", 1.8);
        y_min_ = declare_parameter("y_min", -0.7);
        y_max_ = declare_parameter("y_max", 0.7);
        z_min_ = declare_parameter("z_min", -0.6);
        z_max_ = declare_parameter("z_max", 1.5);

        // ── voxel ───────────────────────────────────────
        voxel_leaf_ = declare_parameter("voxel_leaf", 0.02);

        // ── SOR (sparse mesh removal) ──────────────────
        sor_enabled_ = declare_parameter("sor_enabled", true);
        sor_mean_k_  = declare_parameter("sor_mean_k", 20);
        sor_stddev_  = declare_parameter("sor_stddev", 1.0);

        // ── RANSAC ──────────────────────────────────────
        ransac_dist_       = declare_parameter("ransac_dist_thresh", 0.03);
        ransac_ground_nz_  = declare_parameter("ransac_ground_nz_min", 0.7);
        ransac_wall_ratio_ = declare_parameter("ransac_wall_min_ratio", 0.30);
        wall_removal_en_   = declare_parameter("wall_removal_enabled", true);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudFilterNode::cloud_callback, this, std::placeholders::_1)
        );

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud", 10
        );

        RCLCPP_INFO(this->get_logger(),
            "Cloud filter: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f] voxel=%.2f SOR=%s wall_removal=%s",
            x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
            voxel_leaf_, sor_enabled_ ? "on" : "off", wall_removal_en_ ? "on" : "off");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double voxel_leaf_;
    bool sor_enabled_;
    int sor_mean_k_;
    double sor_stddev_;
    double ransac_dist_, ransac_ground_nz_, ransac_wall_ratio_;
    bool wall_removal_en_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudT::Ptr cloud_raw(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud_raw);

        if (cloud_raw->empty()) return;

        PointCloudT::Ptr cloud_proc(new PointCloudT);

        // ── 1. Passthrough ──────────────────────────────
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

        // ── 2. Voxel downsampling ───────────────────────
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_proc);
        vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        vg.filter(*cloud_proc);

        if (cloud_proc->empty()) return;

        // ── 3. RANSAC ground removal (horizontal planes) ─
        {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_dist_);
            seg.setInputCloud(cloud_proc);
            seg.segment(*inliers, *coeff);

            if (!inliers->indices.empty() && coeff->values.size() >= 4) {
                float nz = std::fabs(coeff->values[2]);
                if (nz > ransac_ground_nz_) {
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud_proc);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*cloud_proc);
                }
            }
        }

        if (cloud_proc->empty()) return;

        // ── 4. RANSAC wall removal (large planar surfaces) ──
        if (wall_removal_en_) {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_dist_);
            seg.setInputCloud(cloud_proc);
            seg.segment(*inliers, *coeff);

            if (!inliers->indices.empty()) {
                double ratio = static_cast<double>(inliers->indices.size()) / cloud_proc->size();
                if (ratio > ransac_wall_ratio_) {
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud_proc);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*cloud_proc);
                    RCLCPP_DEBUG(get_logger(),
                        "Removed large plane: %ld pts (%.0f%% of cloud) normal=(%.2f,%.2f,%.2f)",
                        inliers->indices.size(), ratio * 100.0,
                        coeff->values[0], coeff->values[1], coeff->values[2]);
                }
            }
        }

        if (cloud_proc->empty()) return;

        // ── 5. SOR — remove sparse mesh points ──────────
        if (sor_enabled_) {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud_proc);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*cloud_proc);
        }

        // ── Publish ─────────────────────────────────────
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
