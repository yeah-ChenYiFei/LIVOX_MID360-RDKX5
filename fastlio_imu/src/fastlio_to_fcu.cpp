#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "fastlio_imu/msg/fcu_state.hpp" 
#include <mutex>    

class OdomToFCU : public rclcpp::Node { 
public:
  OdomToFCU() : Node("odom_to_fcu") {
    // 1. 订阅 Odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry_scpgo", 10, std::bind(&OdomToFCU::odom_callback, this, std::placeholders::_1));
    
    // 2. 订阅 IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 10, std::bind(&OdomToFCU::imu_callback, this, std::placeholders::_1));

    // 3. 初始化发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fcu/odom_pose", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fcu/odom_twist", 10);
    
    // 用来转发原始 IMU
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fcu/imu", 10);

    // 发布融合后的自定义消息
    fcu_state_pub_ = this->create_publisher<fastlio_imu::msg::FcuState>("/fcu/state", 10);
  }

private:
  // --- 回调函数 1：接收并缓存 IMU 数据 ---
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 加锁写数据（防止多线程冲突，虽然 spin 一般是单线程，但好习惯）
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 缓存最新的 IMU 数据
    latest_imu_ = *msg;
    has_imu_data_ = true;

    // 顺便转发原始 IMU (如果需要)
    imu_pub_->publish(*msg);
  }

  // --- 回调函数 2：接收 Odometry 并触发融合发布 ---
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 1. 先转发 Pose 和 Twist (原有功能)
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "map"; 
    pose_msg.pose = msg->pose.pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = msg->header;
    twist_msg.twist = msg->twist.twist;
    twist_pub_->publish(twist_msg);

    // 2. 核心逻辑：尝试融合数据
    fastlio_imu::msg::FcuState fcu_msg;
    
    // 写入 Odometry 的信息 (位置、姿态、线速度)
    fcu_msg.header = msg->header;
    fcu_msg.pose = msg->pose.pose;
    fcu_msg.twist.linear = msg->twist.twist.linear; // 线速度来自 FAST-LIO

    // 写入 IMU 的信息 (角速度)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      
      if (has_imu_data_) {
        // 如果已经收到了 IMU 数据，则融合
        fcu_msg.twist.angular = latest_imu_.angular_velocity;
        // 注意：这里的时间戳是 Odometry 的，角速度用的是最新的 IMU 值
        // 这样做在数据延迟较小的情况下是可行的
      } else {
        // 如果还没收到 IMU 数据，暂时用 Odometry 自带的(可能为0)
        fcu_msg.twist.angular = msg->twist.twist.angular;
        RCLCPP_WARN_ONCE(this->get_logger(), "IMU data not received yet, publishing odom-only angular vel.");
      }
    }

    // 3. 最终发布融合消息
    fcu_state_pub_->publish(fcu_msg);
  }

  // --- 成员变量 ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<fastlio_imu::msg::FcuState>::SharedPtr fcu_state_pub_;

  // 用于数据同步的缓存变量
  sensor_msgs::msg::Imu latest_imu_; // 存储最新的 IMU 数据
  bool has_imu_data_ = false;        // 标志位：是否收到过 IMU 数据
  std::mutex mutex_;                 // 互斥锁，保证线程安全
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToFCU>());
  rclcpp::shutdown();
  return 0;
}
