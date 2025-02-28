// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SL_TYPES_HPP_
#define SL_TYPES_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <zed_msgs/msg/pos_track_status.hpp>
#include <zed_msgs/srv/set_pose.hpp>

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0, 0, RCL_SYSTEM_TIME)

namespace stereolabs
{

  // ----> Global constants
  const double DEG2RAD = 0.017453293;
  const double RAD2DEG = 57.295777937;

  const sl::COORDINATE_SYSTEM ROS_COORDINATE_SYSTEM =
      sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  const sl::UNIT ROS_MEAS_UNITS = sl::UNIT::METER;

  const int QOS_QUEUE_SIZE = 10;
  // <---- Global constants

#ifdef _SL_JETSON_
  const bool IS_JETSON = true;
#else
  const bool IS_JETSON = false;
#endif

  const float NOT_VALID_TEMP = -273.15f;

  // ----> Typedefs to simplify declarations

  typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub;
  typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>
      pointcloudPub;

  typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuPub;

  typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      posePub;
  typedef std::shared_ptr<rclcpp::Publisher<zed_msgs::msg::PosTrackStatus>>
      poseStatusPub;

  typedef std::shared_ptr<
      rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>>
      poseCovPub;
  typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TransformStamped>>
      transfPub;
  typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub;
  typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pathPub;

  // typedef std::unique_ptr<point_cloud_transport::PointCloudTransport> ptTranspPtr;

  typedef std::unique_ptr<sensor_msgs::msg::Image> imageMsgPtr;
  typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
  typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
  typedef std::unique_ptr<sensor_msgs::msg::Imu> imuMsgPtr;

  typedef std::unique_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;

  typedef std::unique_ptr<zed_msgs::msg::PosTrackStatus> poseStatusMsgPtr;
  typedef std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>
      poseCovMsgPtr;
  typedef std::unique_ptr<geometry_msgs::msg::TransformStamped> transfMsgPtr;
  typedef std::unique_ptr<nav_msgs::msg::Odometry> odomMsgPtr;
  typedef std::unique_ptr<nav_msgs::msg::Path> pathMsgPtr;

  typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOdomSrvPtr;
  typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetPosTrkSrvPtr;
  typedef rclcpp::Service<zed_msgs::srv::SetPose>::SharedPtr setPoseSrvPtr;

  /*!
   * @brief Video/Depth topic resolution
   */
  typedef enum
  {
    NATIVE, //!< Same camera grab resolution
    CUSTOM  //!< Custom Rescale Factor
  } PubRes;

  std::string toString(const PubRes &res)
  {
    switch (res)
    {
    case NATIVE:
      return "NATIVE";
    case CUSTOM:
      return "CUSTOM";
    default:
      return "";
    }
  }

  typedef enum
  {
    PUB,     //!< Same resolution as Color and Depth Map. [Old behavior for compatibility]
    FULL,    //!< Full resolution. Not recommended because slow processing and high bandwidth requirements
    COMPACT, //!< Standard resolution. Optimizes processing and bandwidth
    REDUCED  //!< Half resolution. Low processing and bandwidth requirements
  } PcRes;
  std::string toString(const PcRes &res)
  {
    switch (res)
    {
    case PUB:
      return "PUB";
    case FULL:
      return "FULL";
    case COMPACT:
      return "COMPACT";
    case REDUCED:
      return "REDUCED";
    default:
      return "";
    }
  }

  const int NEURAL_W = 896;
  const int NEURAL_H = 512;
  // <---- Typedefs to simplify declarations

} // namespace stereolabs

#endif // SL_TYPES_HPP_
