#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>
#include <thread>
#include <chrono>

#include "fastlio_pgo/Scancontext.h"

using namespace std::chrono_literals;

typedef pcl::PointXYZI PointType;

class SCPGONode : public rclcpp::Node {
public:
    SCPGONode() : Node("sc_pgo_node"), last_optimized_pub_time_(0) {
        // 参数声明
        this->declare_parameter("keyframe_distance", 0.5);
        this->declare_parameter("keyframe_time", 1.0);
        this->declare_parameter("keyframe_angle", 0.35);   // 提高默认值，避免静止时误插入
        this->declare_parameter("loop_closure_interval", 1.0);
        this->declare_parameter("sc_distance_thresh", 0.4);
        this->declare_parameter("sc_num_candidates", 5);
        this->declare_parameter("voxel_size", 0.5);
        this->declare_parameter("loop_min_index_diff", 30);  // 新增：回环最小索引差
        this->declare_parameter("icp_max_translation", 0.1); // 新增：ICP最大有效平移(m)
        this->declare_parameter("icp_max_rotation", 0.1);    // 新增：ICP最大有效旋转(rad)

        // 获取参数
        keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
        keyframe_time_ = this->get_parameter("keyframe_time").as_double();
        keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
        loop_closure_interval_ = this->get_parameter("loop_closure_interval").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        loop_min_index_diff_ = this->get_parameter("loop_min_index_diff").as_int();
        icp_max_translation_ = this->get_parameter("icp_max_translation").as_double();
        icp_max_rotation_ = this->get_parameter("icp_max_rotation").as_double();

        // 初始化 Scan Context 管理器
        sc_manager_.setSCdistThres(this->get_parameter("sc_distance_thresh").as_double());

        // 初始化 GTSAM (ISAM2)
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 1;
        isam_ = new gtsam::ISAM2(params);
        next_node_id_ = 0;

        // 创建订阅者：里程计 (nav_msgs/Odometry) 和点云
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&SCPGONode::poseCallback, this, std::placeholders::_1));
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&SCPGONode::cloudCallback, this, std::placeholders::_1));

        // 发布器
        optimized_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/optimized_path", 10);
        optimized_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_scpgo", 10);

        // 启动回环检测线程
        loop_thread_ = std::thread(&SCPGONode::loopClosureThread, this);

        // 启动定时器，定期发布优化里程计（即使没有回环也输出最新估计）
        pub_timer_ = this->create_wall_timer(100ms, std::bind(&SCPGONode::publishLatestOdom, this));

        RCLCPP_INFO(this->get_logger(), "SC-PGO node started.");
    }

    ~SCPGONode() {
        loop_running_ = false;
        if (loop_thread_.joinable()) loop_thread_.join();
        delete isam_;
    }

private:
    struct KeyFrame {
        int id;
        double stamp;
        pcl::PointCloud<PointType>::Ptr cloud;
        Eigen::Isometry3d pose;
        gtsam::Pose3 gtsam_pose;
    };

    // 里程计回调：只存储当前里程计，不再转发到优化话题
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_odom_ = msg;
    }

    // 点云回调
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cloud);

        nav_msgs::msg::Odometry::SharedPtr odom;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom = current_odom_;
        }
        if (!odom) return;

        if (isKeyframe(odom)) {
            KeyFrame kf;
            kf.id = next_keyframe_id_++;
            kf.stamp = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
            kf.cloud = downsampleCloud(cloud);
            kf.pose = Eigen::Isometry3d::Identity();
            kf.pose.translation() = Eigen::Vector3d(odom->pose.pose.position.x,
                                                    odom->pose.pose.position.y,
                                                    odom->pose.pose.position.z);
            kf.pose.linear() = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                  odom->pose.pose.orientation.x,
                                                  odom->pose.pose.orientation.y,
                                                  odom->pose.pose.orientation.z).toRotationMatrix();
            kf.gtsam_pose = gtsam::Pose3(gtsam::Rot3(kf.pose.rotation()),
                                         gtsam::Point3(kf.pose.translation()));

            // 保存关键帧
            {
                std::lock_guard<std::mutex> lock(kf_mutex_);
                keyframes_.push_back(kf);
            }

            // 生成 Scan Context
            sc_manager_.makeAndSaveScancontextAndKeys(*kf.cloud);

            // 添加因子：第一个关键帧添加先验因子，后续添加里程计因子
            if (keyframes_.size() == 1) {
                node_ids_[kf.id] = next_node_id_++;
                gtsam::NonlinearFactorGraph prior_graph;
                auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(1e-6));
                auto prior_factor = gtsam::PriorFactor<gtsam::Pose3>(
                    node_ids_[kf.id], gtsam::Pose3::Identity(), prior_noise);
                prior_graph.add(prior_factor);
                gtsam::Values prior_values;
                prior_values.insert(node_ids_[kf.id], kf.gtsam_pose);
                isam_->update(prior_graph, prior_values);
                RCLCPP_INFO(this->get_logger(), "Added prior factor for keyframe %d", kf.id);
            } else {
                auto& prev = keyframes_[keyframes_.size()-2];
                auto& curr = keyframes_.back();
                gtsam::Pose3 rel_pose = prev.gtsam_pose.between(curr.gtsam_pose);
                addFactor(prev.id, curr.id, rel_pose);
            }

            RCLCPP_INFO(this->get_logger(), "Added keyframe %d", kf.id);

            // 新关键帧加入后，立即优化并发布（实时性）
            optimizeAndPublish();
        }
    }

    bool isKeyframe(const nav_msgs::msg::Odometry::SharedPtr& odom) {
        static nav_msgs::msg::Odometry last_kf_odom;
        static double last_kf_time = 0.0;

        double now = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
        if (keyframes_.empty()) {
            last_kf_odom = *odom;
            last_kf_time = now;
            return true;
        }

        double dx = odom->pose.pose.position.x - last_kf_odom.pose.pose.position.x;
        double dy = odom->pose.pose.position.y - last_kf_odom.pose.pose.position.y;
        double dz = odom->pose.pose.position.z - last_kf_odom.pose.pose.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        Eigen::Quaterniond q1(last_kf_odom.pose.pose.orientation.w,
                              last_kf_odom.pose.pose.orientation.x,
                              last_kf_odom.pose.pose.orientation.y,
                              last_kf_odom.pose.pose.orientation.z);
        Eigen::Quaterniond q2(odom->pose.pose.orientation.w,
                              odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y,
                              odom->pose.pose.orientation.z);
        double angle = q1.angularDistance(q2);
        double dt = now - last_kf_time;

        if (dist > keyframe_distance_ || dt > keyframe_time_ || angle > keyframe_angle_) {
            last_kf_odom = *odom;
            last_kf_time = now;
            return true;
        }
        return false;
    }

    pcl::PointCloud<PointType>::Ptr downsampleCloud(const pcl::PointCloud<PointType>::Ptr& cloud) {
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*filtered);
        return filtered;
    }

    // 添加因子（里程计或回环），使用更合理的噪声模型
    void addFactor(int from_id, int to_id, const gtsam::Pose3& rel_pose) {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;

        if (node_ids_.find(from_id) == node_ids_.end()) {
            node_ids_[from_id] = next_node_id_++;
            // 找到对应的关键帧位姿
            for (auto& kf : keyframes_) {
                if (kf.id == from_id) {
                    new_values.insert(node_ids_[from_id], kf.gtsam_pose);
                    break;
                }
            }
        }
        if (node_ids_.find(to_id) == node_ids_.end()) {
            node_ids_[to_id] = next_node_id_++;
            for (auto& kf : keyframes_) {
                if (kf.id == to_id) {
                    new_values.insert(node_ids_[to_id], kf.gtsam_pose);
                    break;
                }
            }
        }

        // 改进噪声模型：平移0.05m，旋转0.02rad
        auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.02, 0.02, 0.02).finished());
        auto factor = gtsam::BetweenFactor<gtsam::Pose3>(
            node_ids_[from_id], node_ids_[to_id], rel_pose, noise);
        new_factors.add(factor);

        isam_->update(new_factors, new_values);
    }

    void loopClosureThread() {
        rclcpp::Rate rate(1.0 / loop_closure_interval_);
        while (rclcpp::ok() && loop_running_) {
            rate.sleep();

            std::lock_guard<std::mutex> lock(kf_mutex_);
            if (keyframes_.size() < 10) continue;

            int curr_idx = keyframes_.size() - 1;
            auto loop_info = sc_manager_.detectLoopClosureID();
            int candidate_idx = loop_info.first;
            float score = loop_info.second;
            (void)score;

            // 检查候选帧与当前帧的索引差，避免近距离匹配
            if (candidate_idx >= 0 && candidate_idx < curr_idx - loop_min_index_diff_) {
                const auto& curr_kf = keyframes_[curr_idx];
                const auto& cand_kf = keyframes_[candidate_idx];
                Eigen::Matrix4f rel_pose_eigen;
                if (icpVerification(curr_kf.cloud, cand_kf.cloud, rel_pose_eigen)) {
                    // 检查 ICP 结果是否过小（噪声导致）
                    Eigen::Vector3f trans = rel_pose_eigen.block<3,1>(0,3);
                    float rot_angle = std::acos(std::min(1.0f, (rel_pose_eigen.block<3,3>(0,0).trace() - 1.0f) / 2.0f));
                    if (trans.norm() < icp_max_translation_ && rot_angle < icp_max_rotation_) {
                        RCLCPP_WARN(this->get_logger(), "ICP transformation too small, ignoring loop closure (trans=%.3f, rot=%.3f)", trans.norm(), rot_angle);
                        continue;
                    }
                    Eigen::Matrix4d rel_pose_d = rel_pose_eigen.cast<double>();
                    gtsam::Pose3 rel_pose_gtsam(
                        gtsam::Rot3(rel_pose_d.block<3,3>(0,0)),
                        gtsam::Point3(rel_pose_d(0,3), rel_pose_d(1,3), rel_pose_d(2,3))
                    );
                    addFactor(cand_kf.id, curr_kf.id, rel_pose_gtsam);
                    optimizeAndPublish();
                    RCLCPP_INFO(this->get_logger(), "Loop closed between %d and %d", cand_kf.id, curr_kf.id);
                }
            }
        }
    }

    bool icpVerification(const pcl::PointCloud<PointType>::Ptr& src,
                         const pcl::PointCloud<PointType>::Ptr& tgt,
                         Eigen::Matrix4f& rel_pose) {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputSource(src);
        icp.setInputTarget(tgt);
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        pcl::PointCloud<PointType> aligned;
        icp.align(aligned);
        if (icp.hasConverged()) {
            rel_pose = icp.getFinalTransformation();
            return true;
        }
        return false;
    }

    void optimizeAndPublish() {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam::Values result = isam_->calculateEstimate();

        // 更新关键帧位姿
        for (auto& kf : keyframes_) {
            auto it = node_ids_.find(kf.id);
            if (it != node_ids_.end()) {
                gtsam::Pose3 optimized = result.at<gtsam::Pose3>(it->second);
                kf.gtsam_pose = optimized;
                kf.pose.translation() = Eigen::Vector3d(optimized.translation().x(),
                                                        optimized.translation().y(),
                                                        optimized.translation().z());
                kf.pose.linear() = optimized.rotation().matrix();
            }
        }

        // 发布优化轨迹（Path）
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        for (const auto& kf : keyframes_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = kf.pose.translation().x();
            pose_stamped.pose.position.y = kf.pose.translation().y();
            pose_stamped.pose.position.z = kf.pose.translation().z();
            Eigen::Quaterniond q(kf.pose.rotation());
            pose_stamped.pose.orientation.w = q.w();
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            path_msg.poses.push_back(pose_stamped);
        }
        optimized_path_pub_->publish(path_msg);

        // 记录最新优化位姿，供定时器发布
        if (!keyframes_.empty()) {
            latest_optimized_pose_ = keyframes_.back().pose;
            last_optimized_pub_time_ = this->now().seconds();
        }
    }

    // 定时发布最新优化里程计（即使没有新回环，也能输出最新估计）
    void publishLatestOdom() {
        if (latest_optimized_pose_.translation().norm() == 0 && latest_optimized_pose_.rotation().determinant() == 0)
            return; // 尚未初始化
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = latest_optimized_pose_.translation().x();
        odom_msg.pose.pose.position.y = latest_optimized_pose_.translation().y();
        odom_msg.pose.pose.position.z = latest_optimized_pose_.translation().z();
        Eigen::Quaterniond q(latest_optimized_pose_.rotation());
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        // 可复制原始里程计的协方差或置零
        optimized_odom_pub_->publish(odom_msg);
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::mutex odom_mutex_;

    std::vector<KeyFrame> keyframes_;
    std::mutex kf_mutex_;
    int next_keyframe_id_ = 0;

    SCManager sc_manager_;

    gtsam::ISAM2* isam_;
    std::unordered_map<int, int> node_ids_;
    int next_node_id_ = 0;
    std::mutex graph_mutex_;

    std::thread loop_thread_;
    bool loop_running_ = true;

    // 参数
    double keyframe_distance_, keyframe_time_, keyframe_angle_, loop_closure_interval_, voxel_size_;
    int loop_min_index_diff_;
    double icp_max_translation_, icp_max_rotation_;

    // 最新优化后的位姿（用于定时发布）
    Eigen::Isometry3d latest_optimized_pose_ = Eigen::Isometry3d::Identity();
    double last_optimized_pub_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SCPGONode>());
    rclcpp::shutdown();
    return 0;
}