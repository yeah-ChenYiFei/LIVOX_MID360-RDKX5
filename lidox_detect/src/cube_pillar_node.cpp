#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl/segmentation/euclidean_cluster_extraction.h>
#include <pcl/features/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class CubePillarNode : public rclcpp::Node
{
public:
    CubePillarNode() : Node("cube_pillar_node")
    {
        // TF 监听：base_link -> map 坐标变换（FAST-LIO 坐标系）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅滤波后点云
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud",
            10,
            std::bind(&CubePillarNode::cloud_callback, this, std::placeholders::_1)
        );

        // 发布方形柱中心（map 坐标系）
        pillar_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/pillar_center",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Cube Pillar Detector Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pillar_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 1. 欧式聚类分割障碍物
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.15);
        ec.setMinClusterSize(80);
        ec.setMaxClusterSize(8000);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 遍历每个聚类，筛选竖直方形柱
        for (auto &indices : cluster_indices)
        {
            PointCloudT::Ptr cluster_cloud(new PointCloudT);
            for (auto idx : indices.indices)
                cluster_cloud->push_back((*cloud)[idx]);

            // 2. PCA 求特征值，判断是否为竖直柱
            Eigen::Vector3f eigenvalues;
            Eigen::Matrix3f eigenvectors;
            pcl::PCA<PointT> pca;
            pca.setInputCloud(cluster_cloud);
            pca.getEigenValues(eigenvalues);
            pca.getEigenVectors(eigenvectors);

            // 排序 λ1 >= λ2 >= λ3
            std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<float>());
            float l1 = eigenvalues(0), l2 = eigenvalues(1), l3 = eigenvalues(2);

            // 竖直柱判定：Z方向拉伸、XY扁平
            bool is_vertical_pillar = (l2 / l1 < 0.35) && (l1 / l3 > 8.0);
            if (!is_vertical_pillar) continue;

            // 3. 求聚类中心点（base_link 系）
            Eigen::Vector3f centroid;
            pcl::computeCentroid(*cluster_cloud, centroid);
            float px = centroid.x();
            float py = centroid.y();
            float pz = centroid.z();

            // 4. 坐标变换到 map 坐标系
            geometry_msgs::msg::PointStamped pt_base, pt_map;
            pt_base.header = msg->header;
            pt_base.point.x = px;
            pt_base.point.y = py;
            pt_base.point.z = pz;

            try
            {
                auto transform = tf_buffer_->lookupTransform(
                    "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
                tf2::doTransform(pt_base, pt_map, transform);
            }
            catch (...)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed");
                continue;
            }

            // 5. 发布柱子中心
            geometry_msgs::msg::PoseStamped pillar_msg;
            pillar_msg.header.frame_id = "map";
            pillar_msg.header.stamp = this->now();
            pillar_msg.pose.position.x = pt_map.point.x;
            pillar_msg.pose.position.y = pt_map.point.y;
            pillar_msg.pose.position.z = pt_map.point.z;
            pillar_msg.pose.orientation.w = 1.0;

            pillar_pub_->publish(pillar_msg);
            RCLCPP_INFO(this->get_logger(),
                "Detect pillar map pos: x=%.2f y=%.2f",
                pt_map.point.x, pt_map.point.y);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubePillarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}