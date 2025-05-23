/****************************************************************************
All in one visual servoing node.
*****************************************************************************/
#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "custom_msgs/RectMsg.h"
#include <visp_bridge/image.h>    
#include <image_transport/image_transport.h>

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
// #include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define CONTROL_UAV
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
bool drone_isRunning = false;
bool has_detection = false;
bool image_received = false;

const double detection_timeout = 0.5; 
std::chrono::steady_clock::time_point last_detection_time;

double tagSize_x, tagSize_y, distance_to_tag;
bool enable_distance_to_tag, opt_verbose, opt_debug;
int opt_display_fps, acq_fps;
bool send_velocity, condition;
double cam_px, cam_py, cam_u0, cam_v0;

vpImage<vpRGBa> I(480, 640);
ros::Publisher vel_pub;
ros::Publisher transvel_pub;
custom_msgs::RectMsg bbox;

void load_parameters(ros::NodeHandle & nh)
{
  std::cout << "===== load params =====" << std::endl;
  // Load parameters from the parameter server
  nh.param("/servo_pixhawk_node/tag_size_x", tagSize_x, 0.3);
  nh.param("/servo_pixhawk_node/tag_size_y", tagSize_y, 0.08);
  nh.param("/servo_pixhawk_node/distance_to_tag", distance_to_tag, 0.8);
  nh.param("/servo_pixhawk_node/has_distance_to_tag", enable_distance_to_tag, true);
  nh.param("/servo_pixhawk_node/display_fps", opt_display_fps, 10);
  nh.param("/servo_pixhawk_node/verbose", opt_verbose, false);
  nh.param("/servo_pixhawk_node/debug", opt_debug, false);
  nh.param("/servo_pixhawk_node/acquisition_fps", acq_fps, 40);
  nh.param("/servo_pixhawk_node/enable_velocity_control", send_velocity, true);
  nh.param("/servo_pixhawk_node/enable_visual", condition, false); 
  nh.param("/servo_pixhawk_node/camera_params/px", cam_px, 554.254691191187);
  nh.param("/servo_pixhawk_node/camera_params/py", cam_py, 554.254691191187);
  nh.param("/servo_pixhawk_node/camera_params/u0", cam_u0, 320.5);
  nh.param("/servo_pixhawk_node/camera_params/v0", cam_v0, 240.5);

  if(opt_verbose){
    ROS_INFO("Loaded parameters:");
    ROS_INFO("  tag_size_x: %f", tagSize_x);
    ROS_INFO("  tag_size_y: %f", tagSize_y);
    ROS_INFO("  distance_to_tag: %f", distance_to_tag);
    ROS_INFO("  has_distance_to_tag: %d", enable_distance_to_tag);
    ROS_INFO("  display_fps: %d", opt_display_fps);
    ROS_INFO("  verbose: %d", opt_verbose);
    ROS_INFO("  debug: %d", opt_debug);
    ROS_INFO("  acquisition_fps: %d", acq_fps);
    ROS_INFO("  enable_velocity_control: %d", send_velocity);
    ROS_INFO("  enable_visual: %d", condition);
    ROS_INFO("  camera_params: px=%f, py=%f, u0=%f, v0=%f", cam_px, cam_py, cam_u0, cam_v0);
  }
}

// 在主循环中检查超时
void checkDetectionTimeout() {
  // 改为ros时钟
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_detection_time).count();
  // std::cout << "[[now]]" << now << std::endl;
  std::cout << "[[elapsed]]" << elapsed <<std::endl;
  if (elapsed > detection_timeout && has_detection) {
    has_detection = false;
    std::cout << "[Timeout] No detection for " << elapsed << " seconds. Setting has_detection to false." << std::endl;
  }
}

void debugPrint(const vpServo& task, const vpFeatureMomentGravityCenterNormalized& s_mgn, const vpFeatureMomentGravityCenterNormalized& s_mgn_d,
  const vpFeatureMomentAreaNormalized& s_man, const vpFeatureMomentAreaNormalized& s_man_d)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Task error: " << task.getError().t() << std::endl;
    std::cout << "Current gravity center: " << std::endl << s_mgn.get_s() << std::endl;
    std::cout << "Desired gravity center: " << std::endl << s_mgn_d.get_s() << std::endl;
    std::cout << "Current normalizae area: " << std::endl << s_man.get_s() << std::endl;
    std::cout << "Desired normalizae area: " << std::endl << s_man_d.get_s() << std::endl;
}

void setVelocity(const vpColVector &ve)
{
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = ve[0];
    twist.twist.linear.y = ve[1];
    twist.twist.linear.z = ve[2];
    twist.twist.angular.z = ve[3];
    vel_pub.publish(twist);
}

bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
  return (p1.second.get_v() < p2.second.get_v());
};

// 回调函数处理YOLO检测结果
void yoloCallback(const custom_msgs::RectMsg msg) {
  last_detection_time = std::chrono::steady_clock::now();
  if (msg.xmin >= msg.xmax || msg.ymin >= msg.ymax ) {
    std::cout << "[[yolo-error111]]" << std::endl;
    has_detection = false;
    return;
  }
  // if(bbox.xmin == msg.xmin && bbox.xmax == msg.xmax && bbox.ymin == msg.ymin && bbox.ymax == msg.ymax) {
  //   std::cout << "[[yolo-error222]]" << std::endl;
  //   has_detection = false;
  //   return;
  // }
  
    bbox.ymin = msg.ymin;
    bbox.xmin = msg.xmin;
    bbox.ymax = msg.ymax;
    bbox.xmax = msg.xmax;
    if(image_received){
      // 在图像上绘制边界框
      vpDisplay::displayRectangle(I, vpImagePoint(bbox.ymin, bbox.xmin), vpImagePoint(bbox.ymax, bbox.xmax), vpColor::red, 0.2, 0.1);
      vpDisplay::flush(I);
    }
    has_detection = true;
    std::cout << "[yoloCallback] Detection: " << has_detection << std::endl;
  
}

// void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//   cv_bridge::CvImagePtr cv_ptr;
//   image_received = true;
//   if(condition){
//     try{
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//       vpImageConvert::convert(cv_ptr->image, I); // 转换为ViSP图像格式
//       image_received = true;
//     } catch (cv_bridge::Exception& e) {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
//   }
// }
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // 转换为ViSP图像格式
  cv_bridge::CvImagePtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      vpImageConvert::convert(cv_ptr->image, I); // 转换为ViSP图像格式
      image_received = true;
      // cv::Mat rgba;
      // cv::cvtColor(cv_ptr->image, rgba, cv::COLOR_BGR2RGBA);
      // vpImageConvert::convert(rgba, I);
      // cv::imshow("Debug", cv_ptr->image);
      // cv::waitKey(1);
      // vpDisplay::display(I);
      // vpDisplay::flush(I);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}
mavros_msgs::State current_state;
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  if(msg->connected && image_received) {
    drone_isRunning = true;
  }
  else {
    drone_isRunning = false;
  }
  std::cout << "Drone state: " << (drone_isRunning ? "Run" : "Disconnected") << std::endl;
  current_state = *msg;
}

int main(int argc, char **argv)
{
  last_detection_time = std::chrono::steady_clock::now();

  ros::init(argc, argv, "drone_ibvs_node");
  ros::NodeHandle nh;
  load_parameters(nh);
  image_transport::ImageTransport it(nh);

  image_transport::Subscriber image_sub = it.subscribe("/iris/realsense/depth_camera/color/image_raw", 1, imageCallback);
  ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, stateCallback);
  ros::Subscriber yolo_sub = nh.subscribe("/uav_detect_from_yolo", 1, yoloCallback);
  vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  ros::spinOnce();
  
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpDisplayX * display;
#else
  vpDisplay *display = nullptr;
#endif

  try {
    
    ros::Rate rate(acq_fps); 

    if(opt_verbose){
      std::cout << std::endl
        << "WARNING:" << std::endl
        << " - This program does no sensing or avoiding of obstacles, " << std::endl
        << "   the drone WILL collide with any objects in the way! Make sure the " << std::endl
        << "   drone has approximately 3 meters of free space on all sides." << std::endl
        << " - The drone uses a forward-facing camera for Apriltag detection," << std::endl
        << "   make sure the drone flies  above a non-uniform flooring," << std::endl
        << "   or its movement will be inacurate and dangerous !" << std::endl
        << std::endl;
    }

    // Connect to the drone
    if (ros::ok()) { 
      vpCameraParameters cam;
      cam.initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0); 
      if (opt_verbose) {
        cam.printParameters();
      }

      int orig_displayX = 90;
      int orig_displayY = 0;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      // display = new vpDisplayX(I, orig_displayX, orig_displayY, "DRONE VIEW"); 
#else
      // vpDisplay *display = vpDisplayFactory::allocateDisplay(I, orig_displayX, orig_displayY, "DRONE VIEW");
#endif

      // vpDisplay::display(I);
      // vpDisplay::flush(I);

      // double time_since_last_display = vpTime::measureTimeMs();
      ros::Time current_time = ros::Time::now();
      double time_since_last_display = current_time.toSec() * 1000.0; // 转为毫秒（可选）

      // vpPlot plotter(1, 640, 480, orig_displayY + static_cast<int>(I.getHeight()), orig_displayX,
      //                "Visual servoing tasks");
      unsigned int iter = 0;
      
      vpServo task; // Visual servoing task

      vpAdaptiveGain lambda = vpAdaptiveGain(2, 1.5 ,20); 
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe);      // cVe:相机坐标系到无人机机体坐标系的 旋转矩阵，eJe:无人机机体坐标系到执行器（电机/螺旋桨）的 雅可比矩阵
      task.setInteractionMatrixType(vpServo::CURRENT);
      task.setLambda(lambda);
      std::cout << "=========set up task=========" << std::endl;

      //! [DJI-F450 cMe]
      /*
       * In the following section, (c1) is an intermediate frame attached to the camera
       * that has axis aligned with the FLU body frame. The real camera frame is denoted (c).
       * The FLU body-frame of the drone denoted (e) is the one in which we need to convert
       * every velocity command computed by visual servoing.
       *
       * We can easily estimate the homogeneous matrix between (c1) and (c) where
       * in our case we have -10 degrees around X camera axis.
       * Then for the transformation between (e) and (c1) frames we can consider only translations.
       *
       * Using those matrices, we can in the end obtain the homogeneous matrix between (c) and (e) frames.
       * This homogeneous matrix is then used to compute the velocity twist matrix cVe.
       */
      vpRxyzVector c1_rxyz_c(vpMath::rad(-15.0), vpMath::rad(0), 0);
      vpRotationMatrix c1Rc(c1_rxyz_c);                       // Rotation between (c1) and (c)
      vpHomogeneousMatrix c1Mc(vpTranslationVector(), c1Rc);  // Homogeneous matrix between (c1) and (c)

      vpRotationMatrix c1Re { 0, -1, 0, 0, 0, -1, 1, 0, 0 };  // Rotation between (c1) and (e)
      vpTranslationVector c1te(0, 0, -0.10);                  // Translation between (c1) and (e)
      vpHomogeneousMatrix c1Me(c1te, c1Re);                   // Homogeneous matrix between (c1) and (e)

      vpHomogeneousMatrix cMe = c1Mc.inverse() * c1Me;        // Homogeneous matrix between (c) and (e)

      vpVelocityTwistMatrix cVe(cMe);
      //! [DJI-F450 cMe]
      task.set_cVe(cVe);

      vpMatrix eJe(6, 4, 0); // 雅可比矩阵

      eJe[0][0] = 1;
      eJe[1][1] = 1;
      eJe[2][2] = 1;
      eJe[5][3] = 1;

      // Desired distance to the target 默认距离1米
      double Z_d = (enable_distance_to_tag ? distance_to_tag : 1.);
      std::cout << "=========Z_d: " << Z_d << "========== "<< std::endl;

      // Define the desired polygon corresponding the the AprilTag CLOCKWISE
      double X[4] = { tagSize_x / 2., tagSize_x / 2., -tagSize_x / 2., -tagSize_x / 2. };
      double Y[4] = { tagSize_y / 2., -tagSize_y / 2., -tagSize_y / 2., tagSize_y / 2. };
      std::cout << "X: " << X[0] << ", " << X[1] << ", " << X[2] << ", " << X[3] << std::endl;
      std::cout << "Y: " << Y[0] << ", " << Y[1] << ", " << Y[2] << ", " << Y[3] << std::endl;
      std::vector<vpPoint> vec_P, vec_P_d;

      for (int i = 0; i < 4; i++) {
        vpPoint P_d(X[i], Y[i], 0);
        vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
        std::cout << "cdMo: " << cdMo << std::endl;
        P_d.track(cdMo); //
        std::cout << "P_d[" << i << "]: " << P_d.get_x() << ", " << P_d.get_y() << std::endl;

        vec_P_d.push_back(P_d);
      }
      vpMomentObject m_obj(3), m_obj_d(3);
      vpMomentDatabase mdb, mdb_d;
      vpMomentBasic mb_d;                                // Here only to get the desired area m00
      vpMomentGravityCenter mg, mg_d;
      vpMomentCentered mc, mc_d;
      vpMomentAreaNormalized man(0, Z_d), man_d(0, Z_d); // Declare normalized area updated below with m00
      vpMomentGravityCenterNormalized mgn, mgn_d;        // Declare normalized gravity center

      // Desired moments
      m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
      m_obj_d.fromVector(vec_P_d);                    // Initialize the object with the points coordinates

      mb_d.linkTo(mdb_d);       // Add basic moments to database
      mg_d.linkTo(mdb_d);       // Add gravity center to database
      mc_d.linkTo(mdb_d);       // Add centered moments to database
      man_d.linkTo(mdb_d);      // Add area normalized to database
      mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
      mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just an_d
      mg_d.compute();           // Compute gravity center moment
      mc_d.compute();           // Compute centered moments AFTER gravity center

      double area = 0;
      if (m_obj_d.getType() == vpMomentObject::DISCRETE)
        area = mb_d.get(2, 0) + mb_d.get(0, 2);
      else
        area = mb_d.get(0, 0);
      // Update moment with the desired area
      man_d.setDesiredArea(area);

      man_d.compute(); // Compute area normalized moment AFTER centered moments
      mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized                       // moment

      // Desired plane
      double A = 0.0;
      double B = 0.0;
      double C = 1.0 / Z_d;

      // Construct area normalized features
      vpFeatureMomentGravityCenterNormalized s_mgn(mdb, A, B, C), s_mgn_d(mdb_d, A, B, C);
      vpFeatureMomentAreaNormalized s_man(mdb, A, B, C), s_man_d(mdb_d, A, B, C);
      vpFeatureVanishingPoint s_vp, s_vp_d;

      // Add the features
      task.addFeature(s_mgn, s_mgn_d);
      task.addFeature(s_man, s_man_d);
      task.addFeature(s_vp, s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());


      // plotter.initGraph(0, 4);
      // plotter.setLegend(0, 0, "Xn");          // Distance from center on X axis feature
      // plotter.setLegend(0, 1, "Yn");          // Distance from center on Y axis feature
      // plotter.setLegend(0, 2, "an");          // Tag area feature
      // plotter.setLegend(0, 3, "atan(1/rho)"); // Vanishing point feature

      // Update desired gravity center normalized feature
      s_mgn_d.update(A, B, C);
      s_mgn_d.compute_interaction();
      // Update desired area normalized feature
      s_man_d.update(A, B, C);
      s_man_d.compute_interaction();

      // Update desired vanishing point feature for the horizontal line
      s_vp_d.setAtanOneOverRho(0);
      s_vp_d.setAlpha(0);

      // bool runLoop = true;
      bool vec_ip_has_been_sorted = false;
      std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;

      std::cout << "=========start loop=========" << std::endl;

      while (ros::ok()) {
      // while (ros::ok() && runLoop) {
        ros::spinOnce();
        double startTime = current_time.toSec() * 1000.0;

        if (condition) {
          vpDisplay::display(I);
          double time_since_last_display = current_time.toSec() * 1000.0;
          double t = ros::Time::now().toSec() * 1000.0 - startTime;
          std::stringstream ss;
          ss << "Detection time: " << t << " ms";
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        }

        checkDetectionTimeout();
        std::cout << "[[has_detection]]: " << has_detection  << std::endl;

        if (has_detection == true ) {
          // Update current points used to compute the moments
          // 这个地方的vpImagePoint需要确认
          std::vector<vpImagePoint> vec_ip; //顺时针的顺序是基于图像平面（即 X-Y 平面，右-下）
          vec_ip.push_back(vpImagePoint(bbox.xmax,bbox.ymax)); // 右下
          vec_ip.push_back(vpImagePoint(bbox.xmin,bbox.ymax)); // 左下
          vec_ip.push_back(vpImagePoint(bbox.xmin,bbox.ymin)); // 左上
          vec_ip.push_back(vpImagePoint(bbox.xmax,bbox.ymin)); // 右上
          if(opt_debug) {
            std::cout << "[[bbox]]: " << bbox.ymax<< bbox.xmax << bbox.ymin << bbox.xmin << std::endl;
            for (size_t i = 0; i < vec_ip.size(); i++) { 
              std::cout << "vec_ip[" << i << "]: " << vec_ip[i] << std::endl;
            }
          }
          vec_P.clear();
          for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam, vec_ip[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            if(opt_debug) {
              std::cout << "vec_ip[" << i << "]: " << vec_ip[i] << std::endl;
              std::cout << "P[" << i << "]: " << P.get_x() << ", " << P.get_y() << std::endl;
            }
            vec_P.push_back(P);
          }

          // Current moments
          m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
          m_obj.fromVector(vec_P);                      // Initialize the object with the points coordinates

          mg.linkTo(mdb);           // Add gravity center to database
          mc.linkTo(mdb);           // Add centered moments to database
          man.linkTo(mdb);          // Add area normalized to database
          mgn.linkTo(mdb);          // Add gravity center normalized to database
          mdb.updateAll(m_obj);     // All of the moments must be updated, not just an_d
          mg.compute();             // Compute gravity center moment
          mc.compute();             // Compute centered moments AFTER gravity center
          man.setDesiredArea(area); // Desired area was init at 0 (unknow at construction),
                                    // need to be updated here
          man.compute();            // Compute area normalized moment AFTER centered moment
          mgn.compute();            // Compute gravity center normalized moment AFTER area normalized
                                    // moment

          s_mgn.update(A, B, C);
          s_mgn.compute_interaction();
          s_man.update(A, B, C);
          s_man.compute_interaction();

          /* Sort points from their height in the image, and keep original indexes.
          This is done once, in order to be independent from the orientation of the tag
          when detecting vanishing points. */
          if (!vec_ip_has_been_sorted) {
            for (size_t i = 0; i < vec_ip.size(); i++) {

              // Add the points and their corresponding index
              std::pair<size_t, vpImagePoint> index_pair = std::pair<size_t, vpImagePoint>(i, vec_ip[i]);
              vec_ip_sorted.push_back(index_pair);
            }

            // Sort the points and indexes from the v value of the points
            std::sort(vec_ip_sorted.begin(), vec_ip_sorted.end(), compareImagePoint);

            vec_ip_has_been_sorted = true;
          }
          if(opt_debug) {
            std::cout << "[[vec_ip_sorted]]: " << vec_ip[vec_ip_sorted[0].first] << "| " << vec_ip[vec_ip_sorted[1].first] << "| "
                      << vec_ip[vec_ip_sorted[2].first] << "| " << vec_ip[vec_ip_sorted[3].first] << std::endl;
          }
          // vpFeatureBuilder::create(s_vp, cam, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first],
          //                          vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first],
          //                          vpFeatureVanishingPoint::selectAtanOneOverRho());
          vpFeatureBuilder::create(s_vp, cam, vec_ip[vec_ip_sorted[1].first],vec_ip[vec_ip_sorted[3].first],
                                   vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[2].first], 
                                   vpFeatureVanishingPoint::selectAtanOneOverRho());
          task.set_cVe(cVe);
          task.set_eJe(eJe);

          // Compute the control law. Velocities are computed in the mobile robot reference frame
          vpColVector ve = task.computeControlLaw();
          if (!send_velocity) {
            ve = 0;
          }
          std::cout << "[[send_velocity]]:" << send_velocity << std::endl;
          if (opt_verbose) {
            std::cout << "[[ve]]: " << ve.t() << std::endl;
          }
          // Sending the control law to the drone
          // if(send_velocity){
          //   setVelocity(ve);
          //   if (opt_verbose) {
          //     std::cout << "[[ve]]: " << ve.t() << std::endl;
          //   }
          // }
#ifdef CONTROL_UAV
  setVelocity(ve);
  if (opt_verbose) {
    std::cout << "[[ve]]: " << ve.t() << std::endl;
  }
#endif

        if (condition) {
            for (size_t i = 0; i < 4; i++) {
              vpDisplay::displayCross(I, vec_ip[i], 15, vpColor::red, 1);
              std::stringstream ss;
              ss << i;
              vpDisplay::displayText(I, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
            }

            // Display 4 visual features
            vpDisplay::displayPolygon(I, vec_ip, vpColor::green, 3);  // Current polygon used to compure an moment
            vpImagePoint cog((bbox.ymin + bbox.ymax)/2, (bbox.xmin + bbox.xmax)/2);
            vpDisplay::displayCross(I, cog, 15, vpColor::green, 3);   // Current polygon used to compute a moment
            vpDisplay::displayLine(I, 0, static_cast<int>(cam.get_u0()), static_cast<int>(I.getHeight()) - 1,
                                   static_cast<int>(cam.get_u0()), vpColor::red, 3);     // Vertical line as desired x position
            vpDisplay::displayLine(I, static_cast<int>(cam.get_v0()), 0, static_cast<int>(cam.get_v0()),
                                   static_cast<int>(I.getWidth()) - 1, vpColor::red, 3); // Horizontal line as desired y position

            // Display lines corresponding to the vanishing point for the horizontal lines
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first], vpColor::red, 1,
                                   false);
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first], vpColor::red, 1,
                                   false);
          }
        }
        else { // 没有检测到tag，停止运动
          std::stringstream sserr;
          sserr << "Failed to detect an Object, stop the robot";
          if (condition) {
            vpDisplay::displayText(I, 120, 20, sserr.str(), vpColor::red);
            vpDisplay::flush(I);
          }
          else {
            std::cout << sserr.str() << std::endl;
          }

          // if(send_velocity){
#ifdef CONTROL_UAV
            geometry_msgs::TwistStamped twist;
            twist.header.stamp = ros::Time::now();
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            twist.twist.angular.z = 0;
            vel_pub.publish(twist);
            std::cout << "Published velocity command: "<< std::endl << twist << std::endl;
          // }
#endif

        }
        if (condition) {
          {
            std::stringstream ss;
            ss << "Left click to " << (send_velocity ? "stop the robot" : "servo the robot")
              << ", right click to quit.";
            vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
          }
          vpDisplay::flush(I);

          // plotter.plot(0, iter, task.getError());
        }

        // vpMouseButton::vpMouseButtonType button; // 鼠标点击图片，左建停止/发布控制指令，右键着陆
        // if (vpDisplay::getClick(I, button, false)) {
        //   switch (button) {
        //   case vpMouseButton::button1: //鼠标左键
        //   send_velocity = !send_velocity;
        //     break;

        //   case vpMouseButton::button3: // 鼠标右键
        //     // drone.land(); // ·着陆·
        //     runLoop = false;
        //     break;

        //   default:
        //     break;
        //   }
        // }

        if (condition) {
          double totalTime = ros::Time::now().toSec() * 1000.0 - startTime;
          std::stringstream sstime;
          sstime << "Total time: " << totalTime << " ms";
          vpDisplay::displayText(I, 80, 20, sstime.str(), vpColor::red);
          vpDisplay::flush(I);
        }

        if(opt_debug){
          debugPrint(task, s_mgn, s_mgn_d,s_man,s_man_d);
        }
        
        iter++;
        ros::spinOnce();  
        rate.sleep(); 
      }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
      if (display != nullptr) {
        delete display;
      }
#endif

      return EXIT_SUCCESS;
    }

    else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
      if (display != nullptr) {
        delete display;
      }
#endif
      return EXIT_FAILURE;
    }
  }
  catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
  }
#endif
    return EXIT_FAILURE;
  }
}
