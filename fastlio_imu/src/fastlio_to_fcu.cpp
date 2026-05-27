#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "fastlio_imu/msg/fcu_state.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

class OdomToFCU : public rclcpp::Node {
public:
    OdomToFCU() : Node("odom_to_fcu") {
        // High-frequency raw odometry from FAST-LIO
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&OdomToFCU::odom_callback, this, std::placeholders::_1));

        // Low-frequency optimized odometry from SC-PGO (drift correction only)
        pgo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry_scpgo", 10,
            std::bind(&OdomToFCU::pgo_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&OdomToFCU::imu_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fcu/odom_pose", 10);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fcu/odom_twist", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fcu/imu", 10);
        fcu_state_pub_ = this->create_publisher<fastlio_imu::msg::FcuState>("/fcu/state", 10);

        RCLCPP_INFO(this->get_logger(),
            "OdomToFCU: /Odometry (high-freq) + /Odometry_scpgo (drift correction)");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = *msg;
        has_imu_ = true;
        imu_pub_->publish(*msg);
    }

    // SC-PGO callback: compute drift correction offset (low frequency)
    void pgo_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        nav_msgs::msg::Odometry raw;
        {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            if (!has_raw_odom_) return;
            raw = raw_odom_;
        }

        Eigen::Isometry3d T_raw = odomToIsometry(raw);
        Eigen::Isometry3d T_pgo = odomToIsometry(*msg);
        Eigen::Isometry3d corr = T_pgo * T_raw.inverse();

        std::lock_guard<std::mutex> lock(corr_mutex_);
        T_correction_ = corr;
        has_correction_ = true;

        RCLCPP_DEBUG(this->get_logger(),
            "PGO correction update: dp=(%.3f,%.3f,%.3f)",
            corr.translation().x(), corr.translation().y(), corr.translation().z());
    }

    // Raw odometry callback: publish at full rate with drift correction applied
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            raw_odom_ = *msg;
            has_raw_odom_ = true;
        }

        // Apply PGO correction if available
        nav_msgs::msg::Odometry corrected;
        {
            std::lock_guard<std::mutex> lock(corr_mutex_);
            corrected = has_correction_ ? applyCorrection(*msg, T_correction_) : *msg;
        }

        // --- compute speed (used by both watchdogs) ---
        double lvx = corrected.twist.twist.linear.x;
        double lvy = corrected.twist.twist.linear.y;
        double lvz = corrected.twist.twist.linear.z;
        double speed = std::sqrt(lvx*lvx + lvy*lvy + lvz*lvz);

        // --- velocity watchdog: reject physically impossible speeds ---
        static constexpr double MAX_SPEED = 5.0;
        if (has_valid_twist_ && speed > MAX_SPEED) {
            corrected.twist.twist = last_valid_twist_;
            lvx = corrected.twist.twist.linear.x;
            lvy = corrected.twist.twist.linear.y;
            lvz = corrected.twist.twist.linear.z;
            speed = std::sqrt(lvx*lvx + lvy*lvy + lvz*lvz);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Rejected speed %.1fm/s, keeping last valid twist", speed);
        } else {
            last_valid_twist_ = corrected.twist.twist;
            has_valid_twist_ = true;
        }

        // --- stationary detection for adaptive position watchdog ---
        static constexpr double STATIONARY_SPEED = 0.05;
        static constexpr int    STATIONARY_FRAMES = 5;
        static constexpr double JUMP_FLYING      = 1.0;
        static constexpr double JUMP_STATIONARY  = 0.15;

        if (speed < STATIONARY_SPEED) {
            stationary_cnt_++;
        } else {
            stationary_cnt_ = 0;
        }
        bool is_stationary = (stationary_cnt_ >= STATIONARY_FRAMES);
        double max_jump = is_stationary ? JUMP_STATIONARY : JUMP_FLYING;

        // --- position jump watchdog (adaptive) ---
        double dx = corrected.pose.pose.position.x - last_valid_pose_.position.x;
        double dy = corrected.pose.pose.position.y - last_valid_pose_.position.y;
        double dz = corrected.pose.pose.position.z - last_valid_pose_.position.z;
        double jump = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (has_valid_pose_ && jump > max_jump) {
            corrected.pose.pose.position = last_valid_pose_.position;
            corrected.pose.pose.orientation = last_valid_pose_.orientation;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Rejected jump %.3fm (>%.3f %s), keeping last valid pose",
                jump, max_jump, is_stationary ? "stationary" : "flying");
        } else {
            last_valid_pose_.position = corrected.pose.pose.position;
            last_valid_pose_.orientation = corrected.pose.pose.orientation;
            has_valid_pose_ = true;
        }

        // Publish pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose = corrected.pose.pose;
        pose_pub_->publish(pose_msg);

        // Publish twist
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header = msg->header;
        twist_msg.twist = corrected.twist.twist;
        twist_pub_->publish(twist_msg);

        // Build FcuState
        fastlio_imu::msg::FcuState fcu_msg;
        fcu_msg.header = msg->header;
        fcu_msg.pose = corrected.pose.pose;
        fcu_msg.twist.linear = corrected.twist.twist.linear;

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_) {
                fcu_msg.twist.angular = latest_imu_.angular_velocity;
            } else {
                fcu_msg.twist.angular = corrected.twist.twist.angular;
            }
        }

        fcu_state_pub_->publish(fcu_msg);
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

    nav_msgs::msg::Odometry applyCorrection(const nav_msgs::msg::Odometry& raw,
                                             const Eigen::Isometry3d& T_corr) {
        Eigen::Isometry3d T_raw = odomToIsometry(raw);
        Eigen::Isometry3d T_out = T_corr * T_raw;

        nav_msgs::msg::Odometry out = raw;
        out.pose.pose.position.x = T_out.translation().x();
        out.pose.pose.position.y = T_out.translation().y();
        out.pose.pose.position.z = T_out.translation().z();
        Eigen::Quaterniond q(T_out.rotation());
        out.pose.pose.orientation.w = q.w();
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();

        // Rotate velocity into corrected frame
        Eigen::Vector3d v(raw.twist.twist.linear.x,
                          raw.twist.twist.linear.y,
                          raw.twist.twist.linear.z);
        Eigen::Vector3d vc = T_corr.rotation() * v;
        out.twist.twist.linear.x = vc.x();
        out.twist.twist.linear.y = vc.y();
        out.twist.twist.linear.z = vc.z();

        return out;
    }

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pgo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<fastlio_imu::msg::FcuState>::SharedPtr fcu_state_pub_;

    // IMU
    sensor_msgs::msg::Imu latest_imu_;
    bool has_imu_ = false;
    std::mutex imu_mutex_;

    // Raw odometry
    nav_msgs::msg::Odometry raw_odom_;
    bool has_raw_odom_ = false;
    std::mutex raw_mutex_;

    // SC-PGO correction
    Eigen::Isometry3d T_correction_ = Eigen::Isometry3d::Identity();
    bool has_correction_ = false;
    std::mutex corr_mutex_;

    // Position jump watchdog
    bool has_valid_pose_ = false;
    geometry_msgs::msg::Pose last_valid_pose_;
    int stationary_cnt_ = 0;

    // Velocity watchdog
    bool has_valid_twist_ = false;
    geometry_msgs::msg::Twist last_valid_twist_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToFCU>());
    rclcpp::shutdown();
    return 0;
}
