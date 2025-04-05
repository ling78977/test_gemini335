#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "common/pcl_map_viewer.h"
#include "common/point_types.h"
#include "pcl_conversions/pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "test_mapping/ndt_inc.h"
// #include "nav_msgs/nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/tf2_ros/transform_broadcaster.h"
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
struct Options {
  Options() {}
  double kf_distance_ = 0.5; // 关键帧距离
  double kf_angle_deg_ = 30; // 旋转角度
  sad::IncNdt3d::Options ndt3d_options_;
};
class IncrementalNDTLIONode : public rclcpp::Node {
public:
  IncrementalNDTLIONode() : rclcpp::Node("test_node"), options_(Options()) {
    RCLCPP_INFO(this->get_logger(), "Start Mapping");
    this->ndt_ = sad::IncNdt3d(options_.ndt3d_options_);
    // viewer_ = std::make_shared<PCLMapViewer>(0.5);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->pointdcloud_subscriber_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth/pointclouds",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
                            rmw_qos_profile_sensor_data),
                        rmw_qos_profile_sensor_data),
            std::bind(&IncrementalNDTLIONode::pointdcloudCallback, this,
                      std::placeholders::_1));
  }
  /**
   * 往LO中增加一个点云
   * @param scan  当前帧点云
   * @param pose 估计pose
   */
  void AddCloud(sad::CloudPtr scan, SE3 &pose, bool use_guess = false) {
    if (first_frame_) {
      pose = SE3();
      last_kf_pose_ = pose;
      this->ndt_.AddCloud(scan);
      first_frame_ = false;
      //   sad::CloudPtr scan_world(new sad::PointCloudType);
      //   pcl::transformPointCloud(*scan, *scan_world,
      //   pose.matrix().cast<float>()); viewer_->SetPoseAndCloud(pose,
      //   scan_world);
      return;
    }
    // 此时local map位于NDT内部，直接配准即可
    SE3 guess;
    ndt_.SetSource(scan);
    if (estimated_poses_.size() < 2) {
      ndt_.AlignNdt(guess);
    } else {
      if (!use_guess) {
        // 从最近两个pose来推断
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        guess = T1 * (T2.inverse() * T1);
      } else {
        guess = pose;
      }

      ndt_.AlignNdt(guess);
    }
    pose = guess;
    estimated_poses_.emplace_back(pose);

    sad::CloudPtr scan_world(new sad::PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, guess.matrix().cast<float>());

    if (IsKeyframe(pose)) {
      last_kf_pose_ = pose;
      cnt_frame_ = 0;
      // 放入ndt内部的local map
      this->ndt_.AddCloud(scan_world);
    }
    // 旋转部分
    SO3 rotation = pose.so3();
    Mat3d R = rotation.matrix();       // 3x3旋转矩阵
    Vec3d axis_angle = rotation.log(); // 轴角表示

    // 平移部分
    Vec3d tran = pose.translation();

    // 打印
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "旋转矩阵: " << axis_angle.transpose());
    RCLCPP_INFO_STREAM(this->get_logger(), "平移向量: " << tran.transpose());
    // if (viewer_ != nullptr) {
    //     viewer_->SetPoseAndCloud(pose, scan_world);
    // }
    // 创建并发布TF变换
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = tran[0];
    t.transform.translation.y = tran[1];
    t.transform.translation.z = tran[2];
    t.transform.rotation.x = rotation.unit_quaternion().x();
    t.transform.rotation.y = rotation.unit_quaternion().y();
    t.transform.rotation.z = rotation.unit_quaternion().z();
    t.transform.rotation.w = rotation.unit_quaternion().w();

    tf_broadcaster_->sendTransform(t);
    cnt_frame_++;
  }

  /// 存储地图(viewer里）
  //   void SaveMap(const std::string &map_path);

private:
  /// 判定是否为关键帧
  bool IsKeyframe(const SE3 &current_pose) {
    if (cnt_frame_ > 15) {
      return true;
    }
    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() >
               options_.kf_angle_deg_ * sad::math::kDEG2RAD;
  }
  void pointdcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->AddCloud(sad::VoxelCloud(this->PointCloud2ToCloudPtr(msg)),
                   this->pose_);
  }
  inline sad::CloudPtr
  PointCloud2ToCloudPtr(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sad::CloudPtr cloud(new sad::PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
  }

private:
  Options options_;
  bool first_frame_ = true;
  std::vector<SE3>
      estimated_poses_; // 所有估计出来的pose，用于记录轨迹和预测下一个帧
  SE3 last_kf_pose_; // 上一关键帧的位姿
  int cnt_frame_ = 0;
  SE3 pose_;
  sad::IncNdt3d ndt_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointdcloud_subscriber_;
  std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<IncrementalNDTLIONode>();
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
