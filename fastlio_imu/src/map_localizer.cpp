#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

typedef pcl::PointXYZI PointType;

class MapLocalizer : public rclcpp::Node {
public:
    MapLocalizer() : Node("map_localizer") {
        this->declare_parameter("map_file", "");
        this->declare_parameter("map_voxel", 0.5);
        this->declare_parameter("scan_voxel", 0.2);
        this->declare_parameter("icp_max_corr_dist", 2.0);
        this->declare_parameter("icp_max_iter", 50);
        this->declare_parameter("icp_fitness_thresh", 0.3);
        this->declare_parameter("min_scan_points", 100);

        std::string map_file = this->get_parameter("map_file").as_string();
        if (map_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "map_file parameter required — node will be idle");
            return;
        }

        pcl::PointCloud<PointType>::Ptr raw(new pcl::PointCloud<PointType>);
        if (pcl::io::loadPCDFile(map_file, *raw) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", map_file.c_str());
            return;
        }

        map_voxel_ = this->get_parameter("map_voxel").as_double();
        map_cloud_ = downsample(raw, map_voxel_);
        RCLCPP_INFO(this->get_logger(), "Map: %s → %zu points (voxel=%.2fm)",
                    map_file.c_str(), map_cloud_->size(), map_voxel_);

        scan_voxel_ = this->get_parameter("scan_voxel").as_double();
        icp_max_corr_dist_ = this->get_parameter("icp_max_corr_dist").as_double();
        icp_max_iter_ = this->get_parameter("icp_max_iter").as_int();
        icp_fitness_thresh_ = this->get_parameter("icp_fitness_thresh").as_double();
        min_scan_points_ = this->get_parameter("min_scan_points").as_int();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&MapLocalizer::odomCallback, this, std::placeholders::_1));
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&MapLocalizer::cloudCallback, this, std::placeholders::_1));

        localized_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/localized_odometry", 10);

        map_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "MapLocalizer ready");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = msg;
        has_odom_ = true;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!map_ready_) return;

        nav_msgs::msg::Odometry::SharedPtr odom;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!has_odom_) return;
            odom = latest_odom_;
        }

        pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *scan);

        if ((int)scan->size() < min_scan_points_) return;

        pcl::PointCloud<PointType>::Ptr scan_ds = downsample(scan, scan_voxel_);

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputSource(scan_ds);
        icp.setInputTarget(map_cloud_);
        icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
        icp.setMaximumIterations(icp_max_iter_);
        icp.setTransformationEpsilon(1e-6);

        pcl::PointCloud<PointType> aligned;
        icp.align(aligned);

        Eigen::Isometry3d T_odom = odomToIsometry(*odom);

        if (icp.hasConverged() && icp.getFitnessScore() < icp_fitness_thresh_) {
            Eigen::Matrix4f T_icp_f = icp.getFinalTransformation();
            Eigen::Matrix4d T_icp = T_icp_f.cast<double>();
            Eigen::Isometry3d T_correction;
            T_correction.matrix() = T_icp;

            T_odom = T_correction * T_odom;
        }

        nav_msgs::msg::Odometry out;
        out.header = msg->header;
        out.header.frame_id = "map";
        out.child_frame_id = "base_link";
        out.pose.pose.position.x = T_odom.translation().x();
        out.pose.pose.position.y = T_odom.translation().y();
        out.pose.pose.position.z = T_odom.translation().z();
        Eigen::Quaterniond q(T_odom.rotation());
        out.pose.pose.orientation.w = q.w();
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();
        out.twist = odom->twist;

        localized_pub_->publish(out);
    }

    pcl::PointCloud<PointType>::Ptr downsample(
            const pcl::PointCloud<PointType>::Ptr& cloud, double leaf) {
        pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*out);
        return out;
    }

    Eigen::Isometry3d odomToIsometry(const nav_msgs::msg::Odometry& msg) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z);
        T.linear() = Eigen::Quaterniond(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z).toRotationMatrix();
        return T;
    }

    bool map_ready_ = false;
    pcl::PointCloud<PointType>::Ptr map_cloud_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localized_pub_;

    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    bool has_odom_ = false;
    std::mutex odom_mutex_;

    double map_voxel_, scan_voxel_, icp_max_corr_dist_, icp_fitness_thresh_;
    int icp_max_iter_, min_scan_points_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLocalizer>());
    rclcpp::shutdown();
    return 0;
}
