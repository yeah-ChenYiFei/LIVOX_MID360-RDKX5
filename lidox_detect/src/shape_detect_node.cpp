/**
 * shape_detect_node — merged ring + pillar detector for Livox MID-360 / RDK X5.
 *
 * Subscribes: /lidox/filtered_cloud
 * Publishes:  /lidox/ring_center   (PoseStamped in map frame)
 *             /lidox/pillar_center (PoseStamped in map frame)
 *
 * Euclidean clustering runs ONCE per frame; each cluster is checked
 * against both ring and pillar PCA criteria.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// ── helper: PCA eigenvalues sorted desc ──────────────────────
static Eigen::Vector3f sorted_eigenvalues(const PointCloudT::Ptr &cloud) {
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector3f ev = pca.getEigenValues();
    std::sort(ev.data(), ev.data() + 3, std::greater<float>());
    return ev;  // λ1 >= λ2 >= λ3
}


class ShapeDetectNode : public rclcpp::Node {
public:
    ShapeDetectNode() : Node("shape_detect_node") {
        // ── parameters ───────────────────────────────────────
        cluster_tol_       = declare("cluster_tolerance",       0.20);
        min_cluster_ring_  = declare("min_cluster_size_ring",   10);
        min_cluster_pillar_= declare("min_cluster_size_pillar", 80);
        max_cluster_       = declare("max_cluster_size",        8000);

        ring_l2l1_min_     = declare("ring_l2_l1_min",         0.4);
        ring_l3l1_max_     = declare("ring_l3_l1_max",         0.35);
        ring_inner_r_      = declare("ring_inner_radius",      0.35);
        ring_outer_r_      = declare("ring_outer_radius",      0.65);
        ring_inner_max_    = declare("ring_inner_ratio_max",   0.2);
        ring_band_min_     = declare("ring_band_ratio_min",    0.4);

        pillar_l2l1_max_   = declare("pillar_l2_l1_max",       0.35);
        pillar_l1l3_min_   = declare("pillar_l1_l3_min",       8.0);

        // ── TF ───────────────────────────────────────────────
        tf_buffer_  = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── pub / sub ────────────────────────────────────────
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud", rclcpp::SensorDataQoS(),
            std::bind(&ShapeDetectNode::on_cloud, this, std::placeholders::_1));

        ring_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/ring_center", 10);
        pillar_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/pillar_center", 10);

        RCLCPP_INFO(get_logger(), "ShapeDetectNode started (ring + pillar merged)");
    }

private:
    // ── parameter helper ─────────────────────────────────────
    template <typename T>
    T declare(const std::string &name, T default_val) {
        return declare_parameter<T>(name, default_val);
    }

    // ── cloud callback ───────────────────────────────────────
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 1. Euclidean clustering (ONCE)
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tol_);
        ec.setMinClusterSize(min_cluster_ring_);   // use the smaller min
        ec.setMaxClusterSize(max_cluster_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 2. classify each cluster
        for (auto &indices : cluster_indices) {
            PointCloudT::Ptr cluster(new PointCloudT);
            for (auto idx : indices.indices)
                cluster->push_back((*cloud)[idx]);

            int n = static_cast<int>(cluster->size());
            Eigen::Vector3f ev = sorted_eigenvalues(cluster);
            float l1 = ev(0), l2 = ev(1), l3 = ev(2);

            // centroid in base_link
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            // dispatch
            if (is_ring(n, l1, l2, l3, cluster, centroid))
                publish_ring(msg->header, centroid);
            if (is_pillar(n, l1, l2, l3))
                publish_pillar(msg->header, centroid);
        }
    }

    // ── ring criteria ────────────────────────────────────────
    bool is_ring(int n, float l1, float l2, float l3,
                 const PointCloudT::Ptr &cluster, const Eigen::Vector4f &centroid) {
        if (n < min_cluster_ring_) return false;

        // PCA shape: XY circular, Z flat
        if (!(l2 / l1 > ring_l2l1_min_)) return false;
        if (!(l3 / l1 < ring_l3l1_max_)) return false;

        // hollow-centre check
        int inner = 0, band = 0;
        for (auto &pt : cluster->points) {
            float d = std::hypot(pt.x - centroid.x(), pt.y - centroid.y());
            if (d < ring_inner_r_)      inner++;
            else if (d < ring_outer_r_) band++;
        }
        float total = static_cast<float>(n);
        if (inner / total > ring_inner_max_) return false;
        if (band  / total < ring_band_min_)  return false;

        return true;
    }

    // ── pillar criteria ──────────────────────────────────────
    bool is_pillar(int n, float l1, float l2, float l3) {
        if (n < min_cluster_pillar_) return false;

        // PCA shape: XY flat, Z stretched
        if (!(l2 / l1 < pillar_l2l1_max_)) return false;
        if (!(l1 / l3 > pillar_l1l3_min_)) return false;

        return true;
    }

    // ── publish helpers (base_link → map) ────────────────────
    void publish_ring(const std_msgs::msg::Header &header,
                      const Eigen::Vector4f &centroid) {
        auto msg = transform_to_map(header, centroid);
        if (msg) {
            ring_pub_->publish(*msg);
            RCLCPP_INFO(get_logger(), "Ring:  map %.2f %.2f %.2f",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
    }

    void publish_pillar(const std_msgs::msg::Header &header,
                        const Eigen::Vector4f &centroid) {
        auto msg = transform_to_map(header, centroid);
        if (msg) {
            pillar_pub_->publish(*msg);
            RCLCPP_INFO(get_logger(), "Pillar: map %.2f %.2f %.2f",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
    }

    // ── TF transform ─────────────────────────────────────────
    std::optional<geometry_msgs::msg::PoseStamped>
    transform_to_map(const std_msgs::msg::Header &header,
                     const Eigen::Vector4f &centroid) {
        geometry_msgs::msg::PointStamped in, out;
        in.header = header;   // use cloud timestamp, not TimePointZero
        in.point.x = centroid.x();
        in.point.y = centroid.y();
        in.point.z = centroid.z();

        try {
            auto xf = tf_buffer_->lookupTransform(
                "map", header.frame_id.empty() ? "base_link" : header.frame_id,
                header.stamp, tf2::durationFromSec(0.2));
            tf2::doTransform(in, out, xf);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "TF failed: %s", e.what());
            return std::nullopt;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp    = now();
        pose.pose.position.x = out.point.x;
        pose.pose.position.y = out.point.y;
        pose.pose.position.z = out.point.z;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    // ── members ──────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  ring_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pillar_pub_;
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;

    // parameters
    double cluster_tol_, ring_l2l1_min_, ring_l3l1_max_;
    double ring_inner_r_, ring_outer_r_, ring_inner_max_, ring_band_min_;
    double pillar_l2l1_max_, pillar_l1l3_min_;
    int    min_cluster_ring_, min_cluster_pillar_, max_cluster_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShapeDetectNode>());
    rclcpp::shutdown();
    return 0;
}
