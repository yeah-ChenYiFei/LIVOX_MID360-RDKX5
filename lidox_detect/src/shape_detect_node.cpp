/**
 * shape_detect_node — ring (RANSAC circle2D) + pillar (PCA) detector.
 *
 * Subscribes: /lidox/filtered_cloud
 * Publishes:  /lidox/ring_center   (PoseStamped, base_link)
 *             /lidox/pillar_center (PoseStamped, base_link)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#include <random>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

static Eigen::Vector3f sorted_eigenvalues(const PointCloudT::Ptr &cloud) {
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector3f ev = pca.getEigenValues();
    std::sort(ev.data(), ev.data() + 3, std::greater<float>());
    return ev;
}


class ShapeDetectNode : public rclcpp::Node {
public:
    ShapeDetectNode() : Node("shape_detect_node") {
        // ── parameters ──────────────────────────────────────
        cluster_tol_        = declare("cluster_tolerance",        0.20);
        min_cluster_ring_   = declare("min_cluster_size_ring",    10);
        min_cluster_pillar_ = declare("min_cluster_size_pillar",  80);
        max_cluster_        = declare("max_cluster_size",         8000);

        // ring – RANSAC circle2D
        ring_fit_tol_   = declare("ring_fit_tolerance",     0.03);
        ring_inner_r_   = declare("ring_inner_radius",      0.35);
        ring_outer_r_   = declare("ring_outer_radius",      0.65);
        ring_inlier_min_= declare("ring_inlier_ratio_min",  0.50);

        // pillar – PCA
        pillar_l2l1_max_ = declare("pillar_l2_l1_max",      0.35);
        pillar_l1l3_min_ = declare("pillar_l1_l3_min",      8.0);

        // ── pub / sub ──────────────────────────────────────
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud", rclcpp::SensorDataQoS(),
            std::bind(&ShapeDetectNode::on_cloud, this, std::placeholders::_1));

        ring_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/ring_center", 10);
        pillar_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/pillar_center", 10);

        RCLCPP_INFO(get_logger(), "ShapeDetectNode started (RANSAC circle2D ring + PCA pillar)");
    }

private:
    template <typename T>
    T declare(const std::string &name, T default_val) {
        return declare_parameter<T>(name, default_val);
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // Euclidean clustering
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tol_);
        ec.setMinClusterSize(min_cluster_ring_);
        ec.setMaxClusterSize(max_cluster_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(get_logger(), "cloud %ld pts -> %ld clusters",
                    cloud->size(), cluster_indices.size());

        for (auto &indices : cluster_indices) {
            PointCloudT::Ptr cluster(new PointCloudT);
            for (auto idx : indices.indices)
                cluster->push_back((*cloud)[idx]);

            int n = static_cast<int>(cluster->size());

            // centroid (for pillar)
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            // ── ring: RANSAC circle2D in XY plane ──────
            Eigen::Vector3f ring_center;
            float ring_radius;
            if (is_ring(cluster, ring_center, ring_radius)) {
                RCLCPP_INFO(get_logger(),
                    "RING: n=%d center=(%.2f,%.2f,%.2f) r=%.3f",
                    n, ring_center.x(), ring_center.y(), ring_center.z(), ring_radius);
                publish_ring(ring_center);
            }

            // ── pillar: PCA (Z stretched) ─────────────
            Eigen::Vector3f ev = sorted_eigenvalues(cluster);
            float l1 = ev(0), l2 = ev(1), l3 = ev(2);
            if (is_pillar(n, l1, l2, l3))
                publish_pillar(centroid);
        }
    }

    // ── ring: hand-written RANSAC circle2D (no pcl_segmentation dep) ──
    bool is_ring(const PointCloudT::Ptr &cluster,
                 Eigen::Vector3f &center, float &radius) {
        int n = static_cast<int>(cluster->size());
        if (n < min_cluster_ring_) return false;

        std::mt19937 rng(42);
        int best_inliers = 0;
        float best_cx = 0, best_cy = 0, best_r = 0;

        auto fit_circle = [](float x1, float y1, float x2, float y2,
                             float x3, float y3,
                             float &cx, float &cy, float &r) -> bool {
            float d = 2.0f * (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
            if (std::fabs(d) < 1e-6f) return false;  // colinear
            float s1 = x1*x1 + y1*y1;
            float s2 = x2*x2 + y2*y2;
            float s3 = x3*x3 + y3*y3;
            cx = (s1*(y2-y3) + s2*(y3-y1) + s3*(y1-y2)) / d;
            cy = (s1*(x3-x2) + s2*(x1-x3) + s3*(x2-x1)) / d;
            r  = std::hypot(cx - x1, cy - y1);
            return r > ring_inner_r_ && r < ring_outer_r_;
        };

        int iter = std::min(200, n * 5);
        for (int k = 0; k < iter; k++) {
            int i1 = rng() % n;
            int i2 = rng() % n, i3 = rng() % n;
            if (i1 == i2 || i1 == i3 || i2 == i3) continue;

            float cx, cy, r;
            if (!fit_circle(cluster->points[i1].x, cluster->points[i1].y,
                            cluster->points[i2].x, cluster->points[i2].y,
                            cluster->points[i3].x, cluster->points[i3].y,
                            cx, cy, r))
                continue;

            int cnt = 0;
            float tol_sq = ring_fit_tol_ * ring_fit_tol_;
            for (int i = 0; i < n; i++) {
                float dx = cluster->points[i].x - cx;
                float dy = cluster->points[i].y - cy;
                float dr = std::fabs(std::hypot(dx, dy) - r);
                if (dr * dr < tol_sq) cnt++;
            }

            if (cnt > best_inliers) {
                best_inliers = cnt;
                best_cx = cx;
                best_cy = cy;
                best_r  = r;
            }
        }

        float ratio = static_cast<float>(best_inliers) / n;
        if (ratio < ring_inlier_min_) return false;

        // compute Z from best-fit inliers
        float tol_sq = ring_fit_tol_ * ring_fit_tol_;
        float z_sum = 0;
        int zi_cnt = 0;
        for (int i = 0; i < n; i++) {
            float dx = cluster->points[i].x - best_cx;
            float dy = cluster->points[i].y - best_cy;
            float dr = std::fabs(std::hypot(dx, dy) - best_r);
            if (dr * dr < tol_sq) {
                z_sum += cluster->points[i].z;
                zi_cnt++;
            }
        }
        center.x() = best_cx;
        center.y() = best_cy;
        center.z() = zi_cnt > 0 ? z_sum / zi_cnt : 0;
        radius = best_r;
        return true;
    }

    // ── pillar: PCA ───────────────────────────────────────
    bool is_pillar(int n, float l1, float l2, float l3) {
        if (n < min_cluster_pillar_) return false;
        if (!(l2 / l1 < pillar_l2l1_max_)) return false;
        if (!(l1 / l3 > pillar_l1l3_min_)) return false;
        return true;
    }

    // ── publish ───────────────────────────────────────────
    void publish_ring(const Eigen::Vector3f &center) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = center.x();
        pose.pose.position.y = center.y();
        pose.pose.position.z = center.z();
        pose.pose.orientation.w = 1.0;
        ring_pub_->publish(pose);
    }

    void publish_pillar(const Eigen::Vector4f &centroid) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = centroid.x();
        pose.pose.position.y = centroid.y();
        pose.pose.position.z = centroid.z();
        pose.pose.orientation.w = 1.0;
        pillar_pub_->publish(pose);
        RCLCPP_INFO(get_logger(), "Pillar: base_link %.2f %.2f %.2f",
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    // ── members ───────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  ring_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pillar_pub_;

    // cluster
    double cluster_tol_;
    int    min_cluster_ring_, min_cluster_pillar_, max_cluster_;

    // ring: RANSAC circle2D
    double ring_fit_tol_, ring_inner_r_, ring_outer_r_, ring_inlier_min_;

    // pillar: PCA
    double pillar_l2l1_max_, pillar_l1l3_min_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShapeDetectNode>());
    rclcpp::shutdown();
    return 0;
}
