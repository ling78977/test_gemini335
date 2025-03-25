#include "libobsensor/ObSensor.hpp"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include "sensor_msgs/msg/imu.hpp"
namespace gemini_335_driver
{

  typedef enum
  {
    DEPTH_640_480_15 = 0,
    DEPTH_640_480_30,

  } DepthResolutionFPS;

  typedef enum
  {
    COLOR_640_480_15 = 0,
    COLOR_640_480_30,
    COLOR_1280_720_15,
    COLOR_1280_720_30,

  } ColorResolutionFPS;

  typedef struct Param
  {
    int width = 640;
    int height = 480;
    int fps = 15;
  } Param;

  class MinimalNode : public rclcpp::Node
  {
  public:
    MinimalNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("gemini335_driver_node", options)
    {
      RCLCPP_INFO(this->get_logger(), "Start Gemini335!");
      // 创建一个Pipeline对象，用于管理depth流
      std::shared_ptr<ob::Pipeline> stream_pipe = nullptr;
      try
      {
        stream_pipe = std::make_shared<ob::Pipeline>();
      }
      catch (ob::Error &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.getMessage());
        return;
      }
      if (!getParams())
      {
        RCLCPP_ERROR(this->get_logger(), "getParams failed !");
        return;
      }

      auto device = stream_pipe->getDevice();

      if (is_hardwire_d2d_)
      {
        if (device->isPropertySupported(OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                                        OB_PERMISSION_WRITE))
        {
          // 参数：true 打开硬件 D2D，false 关闭硬件 D2D
          device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, true);
          RCLCPP_INFO(this->get_logger(), "Open hardware D2D sucessed !");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "device not support hardware D2D");
        }
      }
      if (device->isGlobalTimestampSupported())
      {
        RCLCPP_INFO(this->get_logger(), "This Device Is Supported GlobalTimestamp.");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "This Device Is Supported GlobalTimestamp!");
      }

      // auto depth_profiles = stream_pipe->getStreamProfileList(OB_SENSOR_DEPTH);

      // std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
      // depthProfile = depth_profiles->getVideoStreamProfile(
      //     depth_params_.width, depth_params_.height, OB_FORMAT_Y16,
      //     depth_params_.fps);

      // if (depthProfile == nullptr)
      // {
      //   RCLCPP_ERROR(this->get_logger(), "depthProfile is nullptr");
      //   return;
      // }
      // std::shared_ptr<ob::Config> stream_config = std::make_shared<ob::Config>();

      // stream_config->enableStream(depthProfile);

      // if (is_use_color_frame_)
      // {
      //   auto color_profiles = stream_pipe->getStreamProfileList(OB_SENSOR_COLOR);
      //   std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
      //   colorProfile = color_profiles->getVideoStreamProfile(
      //       color_params_.width, color_params_.height, OB_FORMAT_MJPEG,
      //       color_params_.fps);
      //   if (colorProfile == nullptr)
      //   {
      //     RCLCPP_ERROR(this->get_logger(), "colorProfile is nullptr");
      //     return;
      //   }

      //   stream_config->enableStream(colorProfile);
      // }

      auto imu_pipe = std::make_shared<ob::Pipeline>(device);
      auto gyro_profiles = imu_pipe->getStreamProfileList(OB_SENSOR_GYRO);
      auto accel_profiles = imu_pipe->getStreamProfileList(OB_SENSOR_ACCEL);

      std::shared_ptr<ob::GyroStreamProfile> gyro_profile = nullptr;
      std::shared_ptr<ob::AccelStreamProfile> accel_profile = nullptr;

      gyro_profile = gyro_profiles->getGyroStreamProfile(OB_GYRO_FS_1000dps,
                                                         OB_SAMPLE_RATE_100_HZ);
      accel_profile = accel_profiles->getAccelStreamProfile(
          OB_ACCEL_FS_4g, OB_SAMPLE_RATE_100_HZ);
      if (accel_profile == nullptr || gyro_profile == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "accel_profile or gyro_profile is nullptr");
        return;
      }
      std::shared_ptr<ob::Config> imu_config = std::make_shared<ob::Config>();
      imu_config->enableStream(gyro_profile);
      imu_config->enableStream(accel_profile);
      imu_pipe->start(imu_config, [&](std::shared_ptr<ob::FrameSet> frameset)
                      {
                        auto count = frameset->frameCount();
                        std::cout << "ros2time: " << this->get_clock()->now().nanoseconds() << std::endl;
                        for(int i=0;i<count;i++){
                          auto frame=frameset->getFrame(i);
                          std::cout<<"    time: "<<frame->globalTimeStampUs()<<"\n";
                          std::cout<<"type: "<<frame->type()<<"\n";
                        }
                      });

      rclcpp::spin(this->get_node_base_interface());
    }

    ob::FrameCallback streamCallback(const std::shared_ptr<ob::FrameSet> frameset)
    {  auto count = frameset->frameCount();
                        std::cout << "ros2time: " << this->get_clock()->now().nanoseconds() << std::endl;
                        for(int i=0;i<count;i++){
                          auto frame=frameset->getFrame(i);
                          std::cout<<"    time: "<<frame->globalTimeStampUs()<<"\n";
                          std::cout<<"type: "<<frame->type()<<"\n";
                        }
    }

    ob::FrameCallback imuCallback(const std::shared_ptr<ob::FrameSet> frameSet)
    {
      //     uint64_t timeStamp = frame->timeStamp();
      // auto gyroFrame = frame->as<ob::GyroFrame>();
      // OBGyroValue value = gyroFrame->value();
      // std::cout << "Gyro Frame: {tsp = " << timeStamp
      // << “，temperature = " << gyroFrame->temperature()
      // << ", data["<< value.x << ", " << value.y << ", " << value.z
      // << "]rad/s" << std::endl;

      auto count = frameSet->frameCount();
      RCLCPP_INFO(this->get_logger(), "this count: %d", &count);
    }

  private:
    bool is_hardwire_d2d_ = true;
    bool is_first_depth_frame_ = true;
    bool is_use_color_frame_ = true;

    Param depth_params_;
    Param color_params_;

    bool getParams()
    {
      int depth_param;
      int color_param;
      try
      {
        is_hardwire_d2d_ = declare_parameter<bool>("is_hardwire_d2d", true);
        is_use_color_frame_ = declare_parameter<bool>("is_use_color_frame", true);
        depth_param = declare_parameter<int>("depth_param", 0);
        color_param = declare_parameter<int>("color_param", 2);
        depth_param = depth_param >= 0 ? depth_param : 0;
        color_param = color_param >= 0 ? color_param : 0;
        depth_param = depth_param <= 5 ? depth_param : 5;
        color_param = color_param <= 3 ? color_param : 3;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        return false;
      }
      switch (depth_param)
      {
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
      switch (color_param)
      {
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
  };
} // namespace gemini_335_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gemini_335_driver::MinimalNode)
