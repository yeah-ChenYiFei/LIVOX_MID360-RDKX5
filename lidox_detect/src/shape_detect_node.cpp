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
        ring_fit_tol_   = declare("ring_fit_tolerance",     0.02);
        ring_inner_r_   = declare("ring_inner_radius",      0.35);
        ring_outer_r_   = declare("ring_outer_radius",      0.65);
        ring_inlier_min_= declare("ring_inlier_ratio_min",  0.70);
        ring_max_pts_   = declare("ring_max_points",        100);

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

    // ── ring: RANSAC circle2D on XY, XZ, YZ projections ──────
    bool is_ring(const PointCloudT::Ptr &cluster,
                 Eigen::Vector3f &center, float &radius) {
        int n = static_cast<int>(cluster->size());
        if (n < min_cluster_ring_) return false;
        if (n > ring_max_pts_) return false;

        float best_ratio = 0;
        Eigen::Vector3f best_center(0, 0, 0);
        float best_r = 0;

        // try all 3 projection planes
        try_plane(0, 1, 2, cluster, n, best_center, best_r, best_ratio);  // XY
        try_plane(0, 2, 1, cluster, n, best_center, best_r, best_ratio);  // XZ
        try_plane(1, 2, 0, cluster, n, best_center, best_r, best_ratio);  // YZ

        if (best_ratio < ring_inlier_min_) return false;

        center = best_center;
        radius = best_r;
        return true;
    }

    // ax_a, ax_b = which axes form the circle plane; ax_z = orthogonal axis
    void try_plane(int ax_a, int ax_b, int ax_z,
                   const PointCloudT::Ptr &cluster, int n,
                   Eigen::Vector3f &best_center, float &best_r, float &best_ratio) {
        std::mt19937 rng(42);
        int best_inliers = 0;
        float best_ca = 0, best_cb = 0, best_cz = 0, best_radius = 0;

        auto get_val = [](const pcl::PointXYZ &p, int axis) -> float {
            return *(&p.x + axis);
        };

        auto fit_circle = [](float a1, float b1, float a2, float b2,
                             float a3, float b3,
                             float &ca, float &cb, float &r) -> bool {
            float d = 2.0f * (a1*(b2-b3) + a2*(b3-b1) + a3*(b1-b2));
            if (std::fabs(d) < 1e-6f) return false;
            float s1 = a1*a1 + b1*b1;
            float s2 = a2*a2 + b2*b2;
            float s3 = a3*a3 + b3*b3;
            ca = (s1*(b2-b3) + s2*(b3-b1) + s3*(b1-b2)) / d;
            cb = (s1*(a3-a2) + s2*(a1-a3) + s3*(a2-a1)) / d;
            r  = std::hypot(ca - a1, cb - b1);
            return true;
        };

        int iter = std::min(200, n * 5);
        for (int k = 0; k < iter; k++) {
            int i1 = rng() % n;
            int i2 = rng() % n, i3 = rng() % n;
            if (i1 == i2 || i1 == i3 || i2 == i3) continue;

            float ca, cb, r;
            float a1 = get_val(cluster->points[i1], ax_a);
            float b1 = get_val(cluster->points[i1], ax_b);
            float a2 = get_val(cluster->points[i2], ax_a);
            float b2 = get_val(cluster->points[i2], ax_b);
            float a3 = get_val(cluster->points[i3], ax_a);
            float b3 = get_val(cluster->points[i3], ax_b);

            if (!fit_circle(a1, b1, a2, b2, a3, b3, ca, cb, r))
                continue;
            if (r < ring_inner_r_ || r > ring_outer_r_) continue;

            int cnt = 0;
            float tol_sq = ring_fit_tol_ * ring_fit_tol_;
            for (int i = 0; i < n; i++) {
                float da = get_val(cluster->points[i], ax_a) - ca;
                float db = get_val(cluster->points[i], ax_b) - cb;
                float dr = std::fabs(std::hypot(da, db) - r);
                if (dr * dr < tol_sq) cnt++;
            }

            if (cnt > best_inliers) {
                best_inliers = cnt;
                best_ca = ca;
                best_cb = cb;
                best_radius = r;
                // avg Z from inliers
                float z_sum = 0;
                int z_cnt = 0;
                for (int i = 0; i < n; i++) {
                    float da = get_val(cluster->points[i], ax_a) - ca;
                    float db = get_val(cluster->points[i], ax_b) - cb;
                    float dr = std::fabs(std::hypot(da, db) - r);
                    if (dr * dr < tol_sq) {
                        z_sum += get_val(cluster->points[i], ax_z);
                        z_cnt++;
                    }
                }
                best_cz = z_cnt > 0 ? z_sum / z_cnt : 0;
            }
        }

        float ratio = static_cast<float>(best_inliers) / n;
        if (ratio > best_ratio) {
            best_ratio = ratio;
            // map back to 3D
            float vals[3] = {0, 0, 0};
            vals[ax_a] = best_ca;
            vals[ax_b] = best_cb;
            vals[ax_z] = best_cz;
            best_center = Eigen::Vector3f(vals[0], vals[1], vals[2]);
            best_r = best_radius;
        }
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
    int ring_max_pts_;

    // pillar: PCA
    double pillar_l2l1_max_, pillar_l1l3_min_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShapeDetectNode>());
    rclcpp::shutdown();
    return 0;
}
