/**
 * shape_detect_node — ring (RANSAC circle2D) + pillar (PCA) detector
 * with multi-frame point cloud accumulation via odometry.
 *
 * Subscribes: /lidox/filtered_cloud  (PointCloud2, base_link)
 *             /Odometry              (Odometry)
 * Publishes:  /lidox/ring_center     (PoseStamped, base_link)
 *             /lidox/pillar_center   (PoseStamped, base_link)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <random>
#include <algorithm>
#include <deque>
#include <mutex>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// ── multi-frame accumulation ──────────────────────────
struct CloudFrame {
    PointCloudT::Ptr cloud;
    Eigen::Isometry3d pose;   // T_world_base at this frame
    double stamp;
};

// Temporal consistency tracking
struct TrackedRing {
    Eigen::Vector3f center;
    int hits = 0;
    int misses = 0;
};

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
        // ── cluster parameters ──────────────────────────
        cluster_tol_         = declare("cluster_tolerance",        0.12);
        min_cluster_ring_    = declare("min_cluster_size_ring",    40);
        min_cluster_pillar_  = declare("min_cluster_size_pillar",  80);
        max_cluster_         = declare("max_cluster_size",         8000);

        // ── ring: RANSAC circle2D ────────────────────────
        ring_fit_tol_    = declare("ring_fit_tolerance",     0.08);
        ring_inner_r_    = declare("ring_inner_radius",      0.40);
        ring_outer_r_    = declare("ring_outer_radius",      0.70);
        ring_inlier_min_ = declare("ring_inlier_ratio_min",  0.45);
        ring_max_pts_    = declare("ring_max_points",        800);

        // ── pillar: PCA ──────────────────────────────────
        pillar_l2l1_max_ = declare("pillar_l2_l1_max",      0.35);
        pillar_l1l3_min_ = declare("pillar_l1_l3_min",      8.0);

        // ── multi-frame accumulation ────────────────────
        accumulate_window_ = declare("accumulate_window",   1.5);
        accumulate_voxel_  = declare("accumulate_voxel",    0.02);

        // ── RANSAC (after fusion, before clustering) ────
        ground_removal_en_ = declare("ground_removal_enabled",  true);
        wall_removal_en_   = declare("wall_removal_enabled",    true);
        ransac_dist_       = declare("ransac_dist_thresh",      0.03);
        ransac_ground_nz_  = declare("ransac_ground_nz_min",   0.7);
        ransac_wall_ratio_ = declare("ransac_wall_min_ratio",  0.30);

        // ── pub / sub ────────────────────────────────────
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidox/filtered_cloud", rclcpp::SensorDataQoS(),
            std::bind(&ShapeDetectNode::on_cloud, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&ShapeDetectNode::on_odom, this, std::placeholders::_1));

        ring_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/ring_center", 10);
        pillar_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lidox/pillar_center", 10);
        merged_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidox/merged_cloud", 10);

        RCLCPP_INFO(get_logger(),
            "ShapeDetectNode: multi-frame acc window=%.1fs voxel=%.2f",
            accumulate_window_, accumulate_voxel_);
    }

private:
    template <typename T>
    T declare(const std::string &name, T default_val) {
        return declare_parameter<T>(name, default_val);
    }

    // ── odometry ────────────────────────────────────
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        T.linear() = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z).toRotationMatrix();

        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = T;
        has_pose_ = true;
    }

    // ── cloud + accumulation ────────────────────────
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        double now = this->now().seconds();

        // push current frame into buffer
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            CloudFrame cf;
            cf.cloud = cloud;
            cf.pose  = latest_pose_;
            cf.stamp = now;
            buffer_.push_back(cf);
        }

        // evict old frames
        while (!buffer_.empty() && (now - buffer_.front().stamp) > accumulate_window_)
            buffer_.pop_front();

        if (buffer_.empty()) return;

        // transform & merge
        PointCloudT::Ptr merged = merge_buffer();
        if (merged->empty()) return;

        // voxel downsample merged cloud
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(merged);
        vg.setLeafSize(accumulate_voxel_, accumulate_voxel_, accumulate_voxel_);
        PointCloudT::Ptr merged_ds(new PointCloudT);
        vg.filter(*merged_ds);
        if (merged_ds->empty()) return;

        // publish merged cloud for RViz2 debugging
        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(*merged_ds, merged_msg);
        merged_msg.header.stamp = this->now();
        merged_msg.header.frame_id = "base_link";
        merged_pub_->publish(merged_msg);

        detect_and_publish(merged_ds, now);
    }

    PointCloudT::Ptr merge_buffer() {
        const auto &latest = buffer_.back();
        Eigen::Isometry3d T_w_b = latest.pose;  // world←base at latest frame
        Eigen::Isometry3d T_b_w = T_w_b.inverse();

        PointCloudT::Ptr merged(new PointCloudT);

        for (auto &cf : buffer_) {
            if (cf.cloud->empty()) continue;

            if (has_pose_ && cf.cloud != latest.cloud) {
                // transform cf.cloud (base_link@t_i) → base_link@t_latest
                Eigen::Isometry3d T_delta = T_b_w * cf.pose;  // base←base_old
                PointCloudT::Ptr aligned(new PointCloudT);
                pcl::transformPointCloud(*cf.cloud, *aligned, T_delta.matrix());
                *merged += *aligned;
            } else {
                *merged += *cf.cloud;
            }
        }

        return merged;
    }

    void detect_and_publish(PointCloudT::Ptr &cloud, double now) {
        // ── RANSAC ground removal (after fusion — ring is dense enough to survive) ──
        if (ground_removal_en_ && cloud->size() > 50) {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_dist_);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coeff);

            if (!inliers->indices.empty() && coeff->values.size() >= 4) {
                float nz = std::fabs(coeff->values[2]);
                if (nz > ransac_ground_nz_) {
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*cloud);
                }
            }
        }

        if (cloud->empty()) return;

        // ── RANSAC wall removal (large planar surfaces) ──
        if (wall_removal_en_ && cloud->size() > 50) {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_dist_);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coeff);

            if (!inliers->indices.empty()) {
                double ratio = static_cast<double>(inliers->indices.size()) / cloud->size();
                if (ratio > ransac_wall_ratio_) {
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*cloud);
                }
            }
        }

        if (cloud->empty()) return;

        // Euclidean clustering
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tol_);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(max_cluster_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        std::vector<Eigen::Vector3f> frame_ring_centers;

        for (auto &indices : cluster_indices) {
            PointCloudT::Ptr cluster(new PointCloudT);
            for (auto idx : indices.indices)
                cluster->push_back((*cloud)[idx]);

            int n = static_cast<int>(cluster->size());

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            // ── ring: RANSAC circle2D ──────────────────
            Eigen::Vector3f ring_center;
            float ring_radius;
            if (is_ring(cluster, ring_center, ring_radius)) {
                RCLCPP_INFO(get_logger(),
                    "RING: n=%d center=(%.2f,%.2f,%.2f) r=%.3f",
                    n, ring_center.x(), ring_center.y(), ring_center.z(), ring_radius);
                frame_ring_centers.push_back(ring_center);
            }

            // ── pillar: PCA ────────────────────────────
            Eigen::Vector3f ev = sorted_eigenvalues(cluster);
            float l1 = ev(0), l2 = ev(1), l3 = ev(2);
            if (is_pillar(n, l1, l2, l3))
                publish_pillar(centroid);
        }

        // Temporal consistency
        update_tracked_rings(frame_ring_centers);
        publish_confirmed_ring();
    }

    // ── ring detection ──────────────────────────────
    bool is_ring(const PointCloudT::Ptr &cluster,
                 Eigen::Vector3f &center, float &radius) {
        int n = static_cast<int>(cluster->size());
        if (n < min_cluster_ring_) return false;
        if (n > ring_max_pts_) return false;

        Eigen::Vector3f ev = sorted_eigenvalues(cluster);
        if (ev(2) / ev(0) > 0.20f) return false;

        float best_ratio = 0;
        Eigen::Vector3f best_center(0, 0, 0);
        float best_r = 0;

        try_plane(0, 1, 2, cluster, n, best_center, best_r, best_ratio);  // XY
        try_plane(0, 2, 1, cluster, n, best_center, best_r, best_ratio);  // XZ
        try_plane(1, 2, 0, cluster, n, best_center, best_r, best_ratio);  // YZ

        if (best_ratio < ring_inlier_min_) {
            RCLCPP_DEBUG(get_logger(),
                "RING reject RANSAC ratio: n=%d best_r=%.3f best_ratio=%.3f < min=%.2f",
                n, best_r, best_ratio, ring_inlier_min_);
            return false;
        }

        center = best_center;
        radius = best_r;
        return true;
    }

    void try_plane(int ax_a, int ax_b, int ax_z,
                   const PointCloudT::Ptr &cluster, int n,
                   Eigen::Vector3f &best_center, float &best_r, float &best_ratio) {
        std::mt19937 rng(std::random_device{}());
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
            float vals[3] = {0, 0, 0};
            vals[ax_a] = best_ca;
            vals[ax_b] = best_cb;
            vals[ax_z] = best_cz;
            best_center = Eigen::Vector3f(vals[0], vals[1], vals[2]);
            best_r = best_radius;
        }
    }

    // ── pillar: PCA ─────────────────────────────────
    bool is_pillar(int n, float l1, float l2, float l3) {
        if (n < min_cluster_pillar_) return false;
        if (!(l2 / l1 < pillar_l2l1_max_)) return false;
        if (!(l1 / l3 > pillar_l1l3_min_)) return false;
        return true;
    }

    // ── publish ─────────────────────────────────────
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
    }

    // ── temporal consistency ────────────────────────
    void update_tracked_rings(const std::vector<Eigen::Vector3f>& centers) {
        std::vector<bool> used(centers.size(), false);

        for (auto &tr : tracked_) {
            float best_dist = temporal_tol_;
            int best_idx = -1;
            for (size_t i = 0; i < centers.size(); i++) {
                if (used[i]) continue;
                float d = (tr.center - centers[i]).norm();
                if (d < best_dist) { best_dist = d; best_idx = i; }
            }
            if (best_idx >= 0) {
                tr.center = tr.center * 0.7f + centers[best_idx] * 0.3f;
                tr.hits++;
                tr.misses = 0;
                used[best_idx] = true;
            } else {
                tr.misses++;
            }
        }

        for (size_t i = 0; i < centers.size(); i++) {
            if (!used[i])
                tracked_.push_back({centers[i], 1, 0});
        }

        tracked_.erase(
            std::remove_if(tracked_.begin(), tracked_.end(),
                [this](const TrackedRing &tr) { return tr.misses >= stale_frames_; }),
            tracked_.end());
    }

    void publish_confirmed_ring() {
        const TrackedRing *best = nullptr;
        for (auto &tr : tracked_) {
            if (tr.hits < confirm_frames_) continue;
            if (!best || tr.hits > best->hits) best = &tr;
        }
        if (best) {
            RCLCPP_INFO(get_logger(),
                "PUBLISH ring: center=(%.2f,%.2f,%.2f) hits=%d",
                best->center.x(), best->center.y(), best->center.z(), best->hits);
            publish_ring(best->center);
        }
    }

    // ── members ─────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  ring_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pillar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    merged_pub_;

    // cluster
    double cluster_tol_;
    int    min_cluster_ring_, min_cluster_pillar_, max_cluster_;

    // ring
    double ring_fit_tol_, ring_inner_r_, ring_outer_r_, ring_inlier_min_;
    int ring_max_pts_;

    // pillar
    double pillar_l2l1_max_, pillar_l1l3_min_;

    // multi-frame accumulation
    double accumulate_window_, accumulate_voxel_;
    std::deque<CloudFrame> buffer_;
    Eigen::Isometry3d latest_pose_ = Eigen::Isometry3d::Identity();
    bool has_pose_ = false;
    std::mutex pose_mutex_;

    // RANSAC (after fusion)
    bool ground_removal_en_, wall_removal_en_;
    double ransac_dist_, ransac_ground_nz_, ransac_wall_ratio_;

    // temporal consistency
    std::vector<TrackedRing> tracked_;
    float temporal_tol_ = 0.15f;
    int confirm_frames_ = 3;
    int stale_frames_ = 5;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShapeDetectNode>());
    rclcpp::shutdown();
    return 0;
}
