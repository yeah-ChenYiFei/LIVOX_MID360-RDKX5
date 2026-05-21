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

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class RingDetectNode : public rclcpp::Node
{
public:
    RingDetectNode() : Node("ring_detect_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud",
            10,
            std::bind(&RingDetectNode::cloud_callback, this, std::placeholders::_1)
        );

        ring_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/ring_center",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Ring Detect Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ring_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 1. 欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.15);
        ec.setMinClusterSize(30);  // 圆环点稀疏，降低最小聚类点数
        ec.setMaxClusterSize(8000);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (auto &indices : cluster_indices)
        {
            PointCloudT::Ptr cluster_cloud(new PointCloudT);
            for (auto idx : indices.indices)
                cluster_cloud->push_back((*cloud)[idx]);

            // 2. PCA 判定形状
            pcl::PCA<PointT> pca;
            pca.setInputCloud(cluster_cloud);
            Eigen::Vector3f eigenvalues;
            pca.getEigenValues(eigenvalues);

            std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<float>());
            float l1 = eigenvalues(0), l2 = eigenvalues(1), l3 = eigenvalues(2);

            // 圆环：XY平面近似圆形 (l1≈l2) + Z方向扁平 (l3小)
            bool is_ring_like = (l2 / l1 > 0.4) && (l3 / l1 < 0.35);
            if (!is_ring_like) continue;

            // 3. 空心检测：统计到中心 XY 距离分布
            Eigen::Vector3f centroid;
            pcl::computeCentroid(*cluster_cloud, centroid);

            int inner_count = 0;   // 内圈点数（应少）
            int ring_count  = 0;   // 环带点数（应多）
            const float inner_r = 0.35f;  // 内径/2=0.45，留余量
            const float outer_r = 0.65f;  // 外径/2=0.60，留余量

            for (auto &pt : cluster_cloud->points)
            {
                float dx = pt.x - centroid.x();
                float dy = pt.y - centroid.y();
                float dist_xy = std::sqrt(dx * dx + dy * dy);

                if (dist_xy < inner_r)
                    inner_count++;
                else if (dist_xy < outer_r)
                    ring_count++;
            }

            int total = cluster_cloud->size();
            float inner_ratio = static_cast<float>(inner_count) / total;
            float ring_ratio  = static_cast<float>(ring_count)  / total;

            if (inner_ratio > 0.2f || ring_ratio < 0.4f) continue;

            // 4. TF 变换到 map 系
            geometry_msgs::msg::PointStamped pt_base, pt_map;
            pt_base.header = msg->header;
            pt_base.point.x = centroid.x();
            pt_base.point.y = centroid.y();
            pt_base.point.z = centroid.z();

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

            // 5. 发布圆环中心
            geometry_msgs::msg::PoseStamped ring_msg;
            ring_msg.header.frame_id = "map";
            ring_msg.header.stamp = this->now();
            ring_msg.pose.position.x = pt_map.point.x;
            ring_msg.pose.position.y = pt_map.point.y;
            ring_msg.pose.position.z = pt_map.point.z;
            ring_msg.pose.orientation.w = 1.0;

            ring_pub_->publish(ring_msg);
            RCLCPP_INFO(this->get_logger(),
                "Ring detected: x=%.2f y=%.2f z=%.2f",
                pt_map.point.x, pt_map.point.y, pt_map.point.z);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RingDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
