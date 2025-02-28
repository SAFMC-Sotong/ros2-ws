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

#ifndef ZED_CAMERA_COMPONENT_HPP_
#define ZED_CAMERA_COMPONENT_HPP_

#include <atomic>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#include <unordered_set>

#include "sl_tools.hpp"
#include "sl_types.hpp"
#include "visibility_control.hpp"

namespace stereolabs
{

  class ZedCamera : public rclcpp::Node
  {
  public:
    ZED_COMPONENTS_PUBLIC
    explicit ZedCamera(const rclcpp::NodeOptions &options);

    virtual ~ZedCamera();

  protected:
    // ----> Initialization functions
    void init();
    void initParameters();
    void initServices();
    void initThreads();

    void getDebugParams();
    void getGeneralParams();
    void getVideoParams();
    void getDepthParams();
    void getPosTrackingParams();
    void getSensorsParams();
    void getAdvancedParams();

    void setTFCoordFrameNames();
    void initPublishers();
    void fillCamInfo(
        const std::shared_ptr<sl::Camera> zed,
        const std::shared_ptr<sensor_msgs::msg::CameraInfo> &leftCamInfoMsg,
        const std::shared_ptr<sensor_msgs::msg::CameraInfo> &rightCamInfoMsg,
        const std::string &leftFrameId, const std::string &rightFrameId,
        bool rawParam = false);

    bool startCamera();
    bool startPosTracking();
    // <---- Initialization functions

    // ----> Callbacks
    void callback_pubPaths();
    rcl_interfaces::msg::SetParametersResult callback_setParameters(
        std::vector<rclcpp::Parameter> parameters);
    void callback_updateDiagnostic(
        diagnostic_updater::DiagnosticStatusWrapper &stat);

    void callback_resetOdometry(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
        std::shared_ptr<std_srvs::srv::Trigger_Response> res);
    void callback_resetPosTracking(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
        std::shared_ptr<std_srvs::srv::Trigger_Response> res);
    void callback_setPose(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<zed_msgs::srv::SetPose_Request> req,
        std::shared_ptr<zed_msgs::srv::SetPose_Response> res);
    // <---- Callbacks

    // ----> Thread functions
    void threadFunc_zedGrab();
    void threadFunc_pointcloudElab();
    void threadFunc_pubSensorsData();
    // <---- Thread functions

    // ----> Publishing functions
    void publishImageWithInfo(
        sl::Mat &img,
        image_transport::CameraPublisher &pubImg,
        camInfoMsgPtr &camInfoMsg, std::string imgFrameId,
        rclcpp::Time t);
    void publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t);

    bool areVideoDepthSubscribed();
    void retrieveVideoDepth();
    void publishVideoDepth(rclcpp::Time &out_pub_ts);
    void publishPointCloud();
    void publishImuFrameAndTopic();

    void publishOdom(
        tf2::Transform &odom2baseTransf, sl::Pose &slPose,
        rclcpp::Time t);
    void publishPose();
    void publishPoseStatus();
    void publishTFs(rclcpp::Time t);
    void publishOdomTF(rclcpp::Time t);
    void publishPoseTF(rclcpp::Time t);
    bool publishSensorsData(rclcpp::Time force_ts = TIMEZERO_ROS);
    // <---- Publishing functions

    // ----> Utility functions
    bool isDepthRequired();
    bool isPosTrackingRequired();

    void applyVideoSettings();
    void applyDepthSettings();

    void processOdometry();
    void processPose();

    bool setPose(float xt, float yt, float zt, float rr, float pr, float yr);
    void initTransforms();
    bool getSens2BaseTransform();
    bool getSens2CameraTransform();
    bool getCamera2BaseTransform();

    void startPathPubTimer(double pathTimerRate);
    
    std::string getParam(std::string param_name, std::vector<std::vector<float> >& value);

    template <typename T>
    void getParam(
        std::string paramName, T defValue, T &outVal,
        std::string log_info = std::string(), bool dynamic = false);

  private:
    // ZED SDK
    std::shared_ptr<sl::Camera> mZed;
    sl::InitParameters mInitParams;
    sl::RuntimeParameters mRunParams;

    uint64_t mFrameCount = 0;

    // ----> Topics
    std::string mTopicRoot = "~/";
    std::string mOdomTopic;
    std::string mPoseTopic;
    std::string mPoseStatusTopic;
    std::string mPoseCovTopic;
    std::string mOdomPathTopic;
    std::string mPosePathTopic;

    // <---- Topics

    // ----> Parameter variables
    bool _debugCommon = false;
    bool _debugVideoDepth = false;
    bool _debugCamCtrl = false;
    bool _debugPointCloud = false;
    bool _debugPosTracking = false;
    bool _debugSensors = false;
    bool _debugAdvanced = false;

    int mCamSerialNumber = 0;

    sl::MODEL mCamUserModel = sl::MODEL::ZED_M; // Default camera model
    sl::MODEL mCamRealModel;                    // Camera model requested to SDK
    unsigned int mCamFwVersion;                 // Camera FW version
    unsigned int mSensFwVersion;                // Sensors FW version
    std::string mCameraName = "zed";            // Default camera name
    int mCamGrabFrameRate = 30;
    bool mAsyncImageRetrieval = false;
    int mImageValidityCheck = 1;
    int mVerbose = 1;
    int mGpuId = -1;
    std::string mOpencvCalibFile;
    sl::RESOLUTION mCamResol = sl::RESOLUTION::HD1080; // Default resolution: RESOLUTION_HD1080
    PubRes mPubResolution = PubRes::NATIVE;            // Use native grab resolution by default
    double mCustomDownscaleFactor = 1.0;               // Used to rescale data with user factor
    bool mOpenniDepthMode =
        false; // 16 bit UC data in mm else 32F in m,
               // for more info -> http://www.ros.org/reps/rep-0118.html
    double mCamMinDepth = 0.1;
    double mCamMaxDepth = 10.0;
    sl::DEPTH_MODE mDepthMode = sl::DEPTH_MODE::NEURAL;
    PcRes mPcResolution = PcRes::COMPACT;
    bool mDepthDisabled = false; // Indicates if depth calculation is not required (DEPTH_MODE::NONE)
    int mDepthStabilization = 1;

    int mCamTimeoutSec = 5;
    int mMaxReconnectTemp = 5;
    bool mCameraSelfCalib = true;
    bool mCameraFlip = false;

    bool mSensCameraSync = false;
    double mSensPubRate = 400.;

    bool mPosTrackingEnabled = false;
    bool mLightComputeEnabled = false;
    bool mPublishTF = false;
    bool mPublishMapTF = false;
    bool mPublishImuTF = false;
    bool mPoseSmoothing = false;
    bool mAreaMemory = true;
    std::string mAreaMemoryDbPath = "";
    sl::POSITIONAL_TRACKING_MODE mPosTrkMode =
        sl::POSITIONAL_TRACKING_MODE::GEN_2;
    bool mImuFusion = true;
    bool mFloorAlignment = false;
    bool mTwoDMode = false;
    double mFixedZValue = 0.0;
    std::vector<double> mInitialBasePose = std::vector<double>(6, 0.0);
    bool mResetOdomWhenLoopClosure = true;
    double mPathPubRate = 2.0;
    double mTfOffset = 0.05;
    double mPosTrackDepthMinRange = 0.0;
    bool mSetAsStatic = false;
    bool mSetGravityAsOrigin = false;
    int mPathMaxCount = -1;
    bool mPublishPoseCov = true;

    std::string mThreadSchedPolicy;
    int mThreadPrioGrab;
    int mThreadPrioSens;
    int mThreadPrioPointCloud;

    bool mStaticTransformsInitialized = false;
    // <---- Parameter variables

    // ----> Dynamic params
    OnSetParametersCallbackHandle::SharedPtr mParamChangeCallbackHandle;

    double mPubFrameRate = 15.0;
    int mCamBrightness = 4;
    int mCamContrast = 4;
    int mCamHue = 0;
    int mCamSaturation = 4;
    int mCamSharpness = 4;
    int mCamGamma = 8;
    bool mCamAutoExpGain = true;
    int mCamGain = 80;
    int mCamExposure = 80;
    bool mCamAutoWB = true;
    int mCamWBTemp = 42;
    int mDepthConf = 50;
    int mDepthTextConf = 100;
    double mPcPubRate = 15.0;
    bool mRemoveSatAreas = true;
    // <---- Dynamic params

    // ----> QoS
    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
    rclcpp::QoS mQos;
    rclcpp::PublisherOptions mPubOpt;
    rclcpp::SubscriptionOptions mSubOpt;
    // <---- QoS

    // ----> Frame IDs
    std::string mRgbFrameId;
    std::string mRgbOptFrameId;
  
    std::string mDepthFrameId;
    std::string mDepthOptFrameId;

    std::string mCloudFrameId;
    std::string mPointCloudFrameId;

    std::string mMapFrameId = "map";
    std::string mOdomFrameId = "odom";
    std::string mBaseFrameId = "";

    std::string mCameraFrameId;

    std::string mRightCamFrameId;
    std::string mRightCamOptFrameId;
    std::string mLeftCamFrameId;
    std::string mLeftCamOptFrameId;

    std::string mImuFrameId;
    // <---- Frame IDs

    // ----> Stereolabs Mat Info
    int mCamWidth;  // Camera frame width
    int mCamHeight; // Camera frame height
    sl::Resolution mMatResol;
    sl::Resolution mPcResol;
    // <---- Stereolabs Mat Info

    // Camera IMU transform
    sl::Transform mSlCamImuTransf;

    // ----> initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> mTfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    // <---- initialization Transform listener

    // ----> TF Transforms
    tf2::Transform
        mMap2OdomTransf;             // Coordinates of the odometry frame in map frame
    tf2::Transform mOdom2BaseTransf; // Coordinates of the base in odometry frame
    tf2::Transform mMap2BaseTransf;  // Coordinates of the base in map frame
    tf2::Transform
        mSensor2BaseTransf; // Coordinates of the base frame in sensor frame
    tf2::Transform
        mSensor2CameraTransf; // Coordinates of the camera frame in sensor frame
    tf2::Transform
        mCamera2BaseTransf; // Coordinates of the base frame in camera frame
    // <---- TF Transforms

    // ----> TF Transforms Flags
    bool mSensor2BaseTransfValid = false;
    bool mSensor2BaseTransfFirstErr = true;
    bool mSensor2CameraTransfValid = false;
    bool mSensor2CameraTransfFirstErr = true;
    bool mCamera2BaseTransfValid = false;
    bool mCamera2BaseFirstErr = true;
    // <---- TF Transforms Flags

    // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
    // Camera infos
    camInfoMsgPtr mRgbCamInfoMsg;
    camInfoMsgPtr mLeftCamInfoMsg;
    camInfoMsgPtr mRightCamInfoMsg;
    camInfoMsgPtr mDepthCamInfoMsg;
    // <---- Messages

    // ----> Publishers
    image_transport::CameraPublisher mPubRgb;
    image_transport::CameraPublisher mPubLeft;
    image_transport::CameraPublisher mPubRight;
    image_transport::CameraPublisher mPubDepth;

    pointcloudPub mPubCloud;
    pointcloudPub mPubFusedCloud;

    posePub mPubPose;
    poseStatusPub mPubPoseStatus;
    poseCovPub mPubPoseCov;
    odomPub mPubOdom;
    pathPub mPubOdomPath;
    pathPub mPubPosePath;
    transfPub mPubCamImuTransf;
    imuPub mPubImu;
    // <---- Publishers

    // <---- Publisher variables
    sl::Timestamp mSdkGrabTS = 0;
    size_t mRgbSubCount = 0;
    size_t mLeftSubCount = 0;
    size_t mRightSubCount = 0;
    size_t mDepthSubCount = 0;

    sl::Mat mMatLeft, mMatRight, mMatDepth;

    float mMinDepth = 0.0f;
    float mMaxDepth = 0.0f;
    // <---- Publisher variables

    // ----> Point cloud variables
    sl::Mat mMatCloud;
    // <---- Point cloud variables

    // ----> Threads and Timers
    sl::ERROR_CODE mGrabStatus;
    sl::ERROR_CODE mConnStatus;
    std::thread mGrabThread; // Main grab thread
    std::thread mPcThread;   // Point Cloud publish thread
    std::thread mSensThread; // Sensors data publish thread
    std::atomic<bool> mThreadStop;
    rclcpp::TimerBase::SharedPtr mInitTimer;
    rclcpp::TimerBase::SharedPtr mPathTimer;
    double mSensRateComp = 1.0;
    // <---- Threads and Timers

    // ----> Thread Sync
    std::mutex mDynParMutex;
    std::mutex mPcMutex;
    std::condition_variable mPcDataReadyCondVar;
    std::atomic_bool mPcDataReady;
    // <---- Thread Sync

    // ----> Status Flags
    bool mDebugMode = false; // Debug mode active?
    bool mPosTrackingStarted = false;
    bool mVdPublishing = false; // Indicates if video and depth data are
                                // subscribed and then published
    bool mPcPublishing =
        false;                       // Indicates if point cloud data are subscribed and then published
    bool mTriggerAutoExpGain = true; // Triggered on start
    bool mTriggerAutoWB = true;      // Triggered on start
    bool mPosTrackingReady = false;

    sl::PositionalTrackingStatus mPosTrackingStatus;

    bool mResetOdomFromSrv = false;
    bool mRgbSubscribed = false;
    // <---- Status Flags

    // ----> Positional Tracking
    sl::Pose mLastZedPose;
    sl::Transform mInitialPoseSl;
    std::vector<geometry_msgs::msg::PoseStamped> mOdomPath;
    std::vector<geometry_msgs::msg::PoseStamped> mPosePath;
    // <---- Positional Tracking

    // ----> Diagnostic
    sl_tools::StopWatch mUptimer;
    std::unique_ptr<sl_tools::WinAvg> mElabPeriodMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mGrabPeriodMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mVideoDepthPeriodMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mVideoDepthElabMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mPcPeriodMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mPcProcMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mImuPeriodMean_sec;
    std::unique_ptr<sl_tools::WinAvg> mPubOdomTF_sec;
    std::unique_ptr<sl_tools::WinAvg> mPubPoseTF_sec;
    std::unique_ptr<sl_tools::WinAvg> mPubImuTF_sec;
    bool mImuPublishing = false;

    diagnostic_updater::Updater mDiagUpdater; // Diagnostic Updater

    sl_tools::StopWatch mImuTfFreqTimer;
    sl_tools::StopWatch mGrabFreqTimer;
    sl_tools::StopWatch mImuFreqTimer;
    sl_tools::StopWatch mOdomFreqTimer;
    sl_tools::StopWatch mPoseFreqTimer;
    sl_tools::StopWatch mPcPubFreqTimer;
    sl_tools::StopWatch mVdPubFreqTimer;
    sl_tools::StopWatch mSensPubFreqTimer;
    sl_tools::StopWatch mPcFreqTimer;

    int mSysOverloadCount = 0;
    // <---- Diagnostic

    // ----> Timestamps
    sl::Timestamp mLastTs_grab = 0; // Used to calculate stable publish frequency
    rclcpp::Time mFrameTimestamp;
    rclcpp::Time mLastTs_imu;
    rclcpp::Time mLastTs_odom;
    rclcpp::Time mLastTs_pose;
    rclcpp::Time mLastTs_pc;
    rclcpp::Time mPrevTs_pc;
    rclcpp::Time mLastClock;
    // <---- Timestamps

    // ----> Services
    resetOdomSrvPtr mResetOdomSrv;
    resetPosTrkSrvPtr mResetPosTrkSrv;
    setPoseSrvPtr mSetPoseSrv;
    // <---- Services

    // ----> Services names
    const std::string mSrvResetOdomName = "reset_odometry";
    const std::string mSrvResetPoseName = "reset_pos_tracking";
    const std::string mSrvSetPoseName = "set_pose";
    // <---- Services names
  };

  // ----> Template Function definitions
  template <typename T>
  void ZedCamera::getParam(
      std::string paramName, T defValue, T &outVal,
      std::string log_info, bool dynamic)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

    if (!get_parameter(paramName, outVal))
    {
      RCLCPP_WARN_STREAM(
          get_logger(),
          "The parameter '"
              << paramName
              << "' is not available or is not valid, using the default value: "
              << defValue);
    }

    if (!log_info.empty())
    {
      RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
    }
  }

} // namespace stereolabs

#endif // ZED_CAMERA_COMPONENT_HPP_
