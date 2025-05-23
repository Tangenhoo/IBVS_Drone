#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.h>
#include "custom_msgs/RectMsg.h"
#include <image_transport/image_transport.h>

#include <visp_bridge/image.h>       
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

class ServoPixhawk
{
  public:
    ServoPixhawk(ros::NodeHandle& nh);
    ~ServoPixhawk(){};

    void loadParameters(ros::NodeHandle & nh);
    void setVelocity(const vpColVector &ve);
    void checkDetectionTimeout();
    void yoloCallback(const custom_msgs::RectMsg& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    static bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2);
    void BuildTask();
    void BuildTask_param(ros::NodeHandle &nh);
    void run();
    void debugPrint(const vpServo& task, 
      const vpFeatureMomentGravityCenterNormalized* s_mgn, 
      const vpFeatureMomentGravityCenterNormalized* s_mgn_d,
      const vpFeatureMomentAreaNormalized* s_man, 
      const vpFeatureMomentAreaNormalized* s_man_d);
    int acq_fps;
  private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber state_sub;
    ros::Subscriber yolo_sub;
    image_transport::Subscriber image_sub;
    ros::Time current_time;
    ros::Time last_detection_time;
    mavros_msgs::State current_state;
    custom_msgs::RectMsg bbox;

    bool has_detection = false;
    bool image_received = false;
    bool drone_isRunning = false;
    bool vec_ip_has_been_sorted = false;

    const double detection_timeout = 0.5;       
    int orig_displayX = 90;
    int orig_displayY = 0;
    double area = 0;
    // Desired plane
    double A = 0.0;
    double B = 0.0;
    double C = 1.0;
    double Z_d;

    double tagSize_x, tagSize_y, distance_to_tag;
    bool enable_distance_to_tag, opt_verbose, opt_debug;
    bool send_velocity, condition;
    int opt_display_fps;
    double cam_px, cam_py, cam_u0, cam_v0;

    vpImage<vpRGBa> I;
    vpCameraParameters cam;
    vpServo task; // Visual servoing task
    std::unique_ptr<vpMomentObject> m_obj, m_obj_d;
    vpMomentDatabase mdb, mdb_d;
    vpMomentBasic mb_d;                                 // Here only to get the desired area m00
    vpMomentGravityCenter mg, mg_d;
    vpMomentCentered mc, mc_d;
    std::unique_ptr<vpMomentAreaNormalized> man, man_d; // Declare normalized area updated below with m00
    vpMomentGravityCenterNormalized mgn, mgn_d;         // Declare normalized gravity center
    vpMatrix eJe; // J
    vpVelocityTwistMatrix cVe;
    std::vector<vpPoint> vec_P, vec_P_d;
    std::unique_ptr<vpFeatureMomentGravityCenterNormalized> s_mgn,s_mgn_d;  // 当前和期望的重心归一化特征
    std::unique_ptr<vpFeatureMomentAreaNormalized> s_man, s_man_d;          // 当前和期望的面积归一化特征
    std::unique_ptr<vpFeatureVanishingPoint> s_vp, s_vp_d;
    std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;
    std::string image_topic, state_topic, yolo_topic, velocity_topic;

  };