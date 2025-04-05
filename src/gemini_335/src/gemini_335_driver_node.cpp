#include "libobsensor/ObSensor.hpp"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace gemini_335_driver {

typedef enum {
  DEPTH_640_480_15 = 0,
  DEPTH_640_480_30,

} DepthResolutionFPS;

typedef enum {
  COLOR_640_480_15 = 0,
  COLOR_640_480_30,
  COLOR_1280_720_15,
  COLOR_1280_720_30,

} ColorResolutionFPS;

typedef struct Param {
  int width = 640;
  int height = 480;
  int fps = 15;
} Param;

class MinimalNode : public rclcpp::Node {
public:
  MinimalNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("gemini335_driver_node", options) {
    RCLCPP_INFO(this->get_logger(), "Start Gemini335!");
    image_msg_.data.reserve(640 * 480 * 3);
    std::shared_ptr<ob::Pipeline> stream_pipe = nullptr;
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
                                    rmw_qos_profile_default),
                                rmw_qos_profile_default));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "depth/pointclouds", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
                                             rmw_qos_profile_sensor_data),
                                         rmw_qos_profile_sensor_data));
    try {
      stream_pipe = std::make_shared<ob::Pipeline>();
    } catch (ob::Error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", e.getMessage());
      return;
    }
    if (!getParams()) {
      RCLCPP_ERROR(this->get_logger(), "getParams failed !");
      return;
    }
    auto device = stream_pipe->getDevice();
    if (is_hardwire_d2d_) {
      if (device->isPropertySupported(OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                                      OB_PERMISSION_WRITE)) {
        // 参数：true 打开硬件 D2D，false 关闭硬件 D2D
        device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, true);
        RCLCPP_INFO(this->get_logger(), "Open hardware D2D sucessed !");
      } else {
        RCLCPP_WARN(this->get_logger(), "device not support hardware D2D");
      }
    }
    if (device->isGlobalTimestampSupported()) {
      RCLCPP_INFO(this->get_logger(),
                  "This Device Is Supported GlobalTimestamp.");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "This Device Is Supported GlobalTimestamp!");
    }
    auto depth_profiles = stream_pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
    depthProfile = depth_profiles->getVideoStreamProfile(
        depth_params_.width, depth_params_.height, OB_FORMAT_Y16,
        depth_params_.fps);
    if (depthProfile == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "depthProfile is nullptr");
      return;
    }
    std::shared_ptr<ob::Config> stream_config = std::make_shared<ob::Config>();
    stream_config->enableStream(depthProfile);
    if (is_use_color_frame_) {
      this->image_publisher_ = std::make_shared<image_transport::Publisher>(
          image_transport::create_publisher(this, "/image_raw",
                                            rmw_qos_profile_sensor_data));
      image_msg_.data.reserve(color_params_.height * color_params_.width * 3);
      auto color_profiles = stream_pipe->getStreamProfileList(OB_SENSOR_COLOR);
      std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
      colorProfile = color_profiles->getVideoStreamProfile(
          color_params_.width, color_params_.height, OB_FORMAT_RGB,
          color_params_.fps);
      if (colorProfile == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "colorProfile is nullptr");
        return;
      }
      stream_config->enableStream(colorProfile);
    }
    auto imu_pipe = std::make_shared<ob::Pipeline>(device);
    auto gyro_profiles = imu_pipe->getStreamProfileList(OB_SENSOR_GYRO);
    auto accel_profiles = imu_pipe->getStreamProfileList(OB_SENSOR_ACCEL);

    std::shared_ptr<ob::GyroStreamProfile> gyro_profile = nullptr;
    std::shared_ptr<ob::AccelStreamProfile> accel_profile = nullptr;

    gyro_profile = gyro_profiles->getGyroStreamProfile(OB_GYRO_FS_1000dps,
                                                       OB_SAMPLE_RATE_100_HZ);
    accel_profile = accel_profiles->getAccelStreamProfile(
        OB_ACCEL_FS_4g, OB_SAMPLE_RATE_100_HZ);
    if (accel_profile == nullptr || gyro_profile == nullptr) {
      RCLCPP_ERROR(this->get_logger(),
                   "accel_profile or gyro_profile is nullptr");
      return;
    }
    gyro_intrinsics_ = gyro_profile->getIntrinsic();
    accel_intrinsics_ = accel_profile->getIntrinsic();
    std::shared_ptr<ob::Config> imu_config = std::make_shared<ob::Config>();
    imu_config->enableStream(gyro_profile);
    imu_config->enableStream(accel_profile);
    imu_pipe->start(imu_config, [&](std::shared_ptr<ob::FrameSet> frameset) {
      auto count = frameset->frameCount();

      // std::cout << "ros2time: " << this->get_clock()->now().nanoseconds() <<
      // std::endl;
      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 1.0;
      imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0,  0.0, 0.0, 0.0};
      imu_msg.linear_acceleration_covariance = {liner_accel_cov_, 0.0, 0.0, 0.0,
                                                liner_accel_cov_, 0.0, 0.0, 0.0,
                                                liner_accel_cov_};
      imu_msg.angular_velocity_covariance = {angular_vel_cov_, 0.0, 0.0, 0.0,
                                             angular_vel_cov_, 0.0, 0.0, 0.0,
                                             angular_vel_cov_};

      imu_msg.header.frame_id = "imu_frame";
      for (uint i = 0; i < count; i++) {
        std::shared_ptr<ob::Frame> frame = frameset->getFrame(i);
        if (frame->type() == OBFrameType::OB_FRAME_ACCEL) {
          imu_msg.header.stamp =
              this->fromUsToROSTime(frame->globalTimeStampUs());
          auto accelframe = frame->as<ob::AccelFrame>();
          imu_msg.linear_acceleration.x =
              accelframe->value().x - gyro_intrinsics_.bias[0];
          imu_msg.linear_acceleration.y =
              accelframe->value().y - gyro_intrinsics_.bias[1];
          imu_msg.linear_acceleration.z =
              accelframe->value().z - gyro_intrinsics_.bias[2];
        } else {
          // RCLCPP_INFO_STREAM(this->get_logger(),frame->globalTimeStampUs());
          auto gyroframe = frame->as<ob::GyroFrame>();
          imu_msg.angular_velocity.x =
              gyroframe->value().x - accel_intrinsics_.bias[0];
          imu_msg.angular_velocity.y =
              gyroframe->value().y - accel_intrinsics_.bias[1];
          imu_msg.angular_velocity.z =
              gyroframe->value().z - accel_intrinsics_.bias[2];
        }
      }
      imu_pub_->publish(imu_msg);
    });
    stream_pipe->start(stream_config, [&](std::shared_ptr<ob::FrameSet>
                                              frameset) {
      auto count = frameset->frameCount();
      for (int i = 0; i < count; i++) {
        auto frame = frameset->getFrame(i);
        auto timestamp = fromUsToROSTime(frame->globalTimeStampUs());
        if (frame->type() == OBFrameType::OB_FRAME_DEPTH) {
          auto depth_frame = frame->as<ob::DepthFrame>();
          float depth_scale = depth_frame->getValueScale();
          depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
          depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
          if (!is_set_filter_camera_param_) {
            depth_point_cloud_filter_.setCameraParam(
                stream_pipe->getCameraParam());
            is_set_filter_camera_param_ = true;
            continue;
          }
          auto result_frame = depth_point_cloud_filter_.process(depth_frame);
          if (!result_frame) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process depth frame");
            return;
          }
          auto point_size = result_frame->dataSize() / sizeof(OBPoint);
          auto *points = static_cast<OBPoint *>(result_frame->data());
          auto width = depth_frame->width();
          auto height = depth_frame->height();
          auto point_cloud_msg =
              std::make_unique<sensor_msgs::msg::PointCloud2>();
          sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
          modifier.setPointCloud2FieldsByString(1, "xyz");
          modifier.resize(width * height);
          point_cloud_msg->width = depth_frame->width();
          point_cloud_msg->height = depth_frame->height();
          point_cloud_msg->row_step =
              point_cloud_msg->width * point_cloud_msg->point_step;
          point_cloud_msg->data.resize(point_cloud_msg->height *
                                       point_cloud_msg->row_step);
          sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
          const static float MIN_DISTANCE = 20.0;    // 2cm
          const static float MAX_DISTANCE = 10000.0; // 10m
          const static float min_depth = MIN_DISTANCE / depth_scale;
          const static float max_depth = MAX_DISTANCE / depth_scale;
          size_t valid_count = 0;
          for (size_t i = 0; i < point_size; i+=130) {
            bool valid_point =
                points[i].z >= min_depth && points[i].z <= max_depth;
            if (valid_point || ordered_pc_) {
              *iter_x = static_cast<float>(points[i].z / 1000.0);
              *iter_y = static_cast<float>(-points[i].x / 1000.0);
              *iter_z = static_cast<float>(-points[i].y / 1000.0);
              
              // iter_x+=500,iter_y+=500,iter_y+=500;
              // RCLCPP_INFO_STREAM(this->get_logger(),"x: "<<*iter_x<<" y: "<<*iter_y<<" z: "<<*iter_z);
              ++iter_x, ++iter_y, ++iter_z;
              valid_count++;
            }
          }
          if(++polling_start>500){
            polling_start=0;
            // savePointsToPly(result_frame,std::to_string(this->get_clock()->now().nanoseconds())+".ply");
            // RCLCPP_INFO_STREAM(this->get_logger(),"paocun");
          }
          
          if (valid_count == 0) {
            RCLCPP_WARN(this->get_logger(), "No valid point in point cloud");
            return;
          }
          if (!ordered_pc_) {
            point_cloud_msg->is_dense = true;
            point_cloud_msg->width = valid_count;
            point_cloud_msg->height = 1;
            modifier.resize(valid_count);
          }
          point_cloud_msg->header.stamp = timestamp;
          point_cloud_msg->header.frame_id = "laser_frame";
          // RCLCPP_INFO_STREAM(this->get_logger(),point_cloud_msg->data.size());
          point_cloud_pub_->publish(std::move(point_cloud_msg));
        } else if (frame->type() == OBFrameType::OB_FRAME_COLOR) {
          auto color_frame = frame->as<ob::VideoFrame>();
          cv::Mat rawMat(color_frame->height(), color_frame->width(), CV_8UC3,
                         color_frame->data());
          cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", rawMat)
              .toImageMsg(image_msg_);
          image_msg_.header.stamp = timestamp;
          image_msg_.header.frame_id = "camera_color_optical_frame";
          this->image_publisher_->publish(image_msg_);
        }
      }
    });

    rclcpp::spin(this->get_node_base_interface());
  }
  void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->data();
    for(int i = 0; i < pointsSize; i+=500) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

private:
  bool is_hardwire_d2d_ = true;
  bool is_first_depth_frame_ = true;
  bool is_use_color_frame_ = true;
  bool ordered_pc_ = false;
  bool is_set_filter_camera_param_ = false;
  uint16_t polling_start=0;

  Param depth_params_;
  Param color_params_;
  std::shared_ptr<image_transport::Publisher> image_publisher_;
  sensor_msgs::msg::Image image_msg_;
  OBGyroIntrinsic gyro_intrinsics_;
  OBAccelIntrinsic accel_intrinsics_;
  OBCameraParam camera_params_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  double liner_accel_cov_ = 0.0001;
  double angular_vel_cov_ = 0.0001;
  ob::PointCloudFilter depth_point_cloud_filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  bool getParams() {
    int depth_param;
    int color_param;
    try {
      is_hardwire_d2d_ = declare_parameter<bool>("is_hardwire_d2d", true);
      is_use_color_frame_ = declare_parameter<bool>("is_use_color_frame", true);
      depth_param = declare_parameter<int>("depth_param", 0);
      color_param = declare_parameter<int>("color_param", 0);
      depth_param = depth_param >= 0 ? depth_param : 0;
      color_param = color_param >= 0 ? color_param : 0;
      depth_param = depth_param <= 5 ? depth_param : 5;
      color_param = color_param <= 3 ? color_param : 3;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      return false;
    }
    switch (depth_param) {
    case DEPTH_640_480_15:
      depth_params_.width = 640;
      depth_params_.height = 480;
      depth_params_.fps = 15;
      break;
    case DEPTH_640_480_30:
      depth_params_.width = 640;
      depth_params_.height = 480;
      depth_params_.fps = 30;
      break;
    default:
      depth_params_.width = 640;
      depth_params_.height = 480;
      depth_params_.fps = 15;
      break;
    }
    switch (color_param) {
    case COLOR_640_480_15:
      color_params_.width = 640;
      color_params_.height = 480;
      color_params_.fps = 15;
      break;
    case COLOR_640_480_30:
      color_params_.width = 640;
      color_params_.height = 480;
      color_params_.fps = 30;
      break;
    case COLOR_1280_720_15:
      color_params_.width = 1280;
      color_params_.height = 720;
      color_params_.fps = 15;
      break;
    case COLOR_1280_720_30:
      color_params_.width = 1280;
      color_params_.height = 720;
      color_params_.fps = 30;
      break;
    default:
      color_params_.width = 640;
      color_params_.height = 480;
      color_params_.fps = 15;
      break;
    }

    return true;
  }

  rclcpp::Time fromUsToROSTime(uint64_t us) {
    auto total = static_cast<uint64_t>(us * 1e3);
    uint64_t sec = total / 1000000000;
    uint64_t nano_sec = total % 1000000000;
    rclcpp::Time stamp(sec, nano_sec);
    return stamp;
  }
};
} // namespace gemini_335_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gemini_335_driver::MinimalNode)
