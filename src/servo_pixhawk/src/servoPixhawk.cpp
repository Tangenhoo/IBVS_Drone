#include "servo_pixhawk/servoPixhawk.h"

ServoPixhawk::ServoPixhawk(ros::NodeHandle& nh)
{
  I.resize(480, 640);
  last_detection_time = ros::Time::now();
  loadParameters(nh);
  BuildTask();
  image_transport::ImageTransport it(nh);

  image_sub = it.subscribe(image_topic, 1, &ServoPixhawk::imageCallback, this);
  state_sub = nh.subscribe(state_topic, 10, &ServoPixhawk::stateCallback, this);
  yolo_sub = nh.subscribe(yolo_topic, 1, &ServoPixhawk::yoloCallback, this);
  vel_pub = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 10);

  ros::spinOnce();
}

// load parameters from the parameter server
void ServoPixhawk::loadParameters(ros::NodeHandle & nh){
  std::cout << "===== load params =====" << std::endl;
  // object actcal size, x for length, y for weidth
  nh.param("/ibvs_task_param/tag_size_x", tagSize_x, 0.03);
  nh.param("/ibvs_task_param/tag_size_y", tagSize_y, 0.03);
  // desired distance to object
  nh.param("/ibvs_task_param/distance_to_tag", Z_d, 1.0);
  nh.param("/ibvs_task_param/has_distance_to_tag", enable_distance_to_tag, true);
  // display and debug parameters
  nh.param("/ibvs_task_param/display_fps", opt_display_fps, 10); // display fps
  nh.param("/ibvs_task_param/verbose", opt_verbose, false); // print verbose info
  nh.param("/ibvs_task_param/debug", opt_debug, false); // print debug info
  nh.param("/ibvs_task_param/enable_visual", condition, false);  // enable visual
  // acquisition fps
  nh.param("/ibvs_task_param/acquisition_fps", acq_fps, 40);
  // enable velocity control
  nh.param("/ibvs_task_param/enable_velocity_control", send_velocity, true);
  // camera parameters
  nh.param("/ibvs_task_param/camera_params/px", cam_px, 554.254691191187);
  nh.param("/ibvs_task_param/camera_params/py", cam_py, 554.254691191187);
  nh.param("/ibvs_task_param/camera_params/u0", cam_u0, 320.5);
  nh.param("/ibvs_task_param/camera_params/v0", cam_v0, 240.5);
  // topics
  nh.param<std::string>("/ibvs_task_param/image_topic", image_topic, "/iris/realsense/depth_camera/color/image_raw");
  nh.param<std::string>("/ibvs_task_param/state_topic", state_topic, "/mavros/state");
  nh.param<std::string>("/ibvs_task_param/yolo_topic", yolo_topic, "/uav_detect_from_yolo");
  nh.param<std::string>("/ibvs_task_param/velocity_topic", velocity_topic, "/mavros/setpoint_velocity/cmd_vel");

  if(opt_verbose){
    ROS_INFO("Loaded parameters:");
    ROS_INFO("  tag_size_x: %f", tagSize_x);
    ROS_INFO("  tag_size_y: %f", tagSize_y);
    ROS_INFO("  distance_to_tag: %f", distance_to_tag);
    ROS_INFO("  has_distance_to_tag: %d", enable_distance_to_tag);
    ROS_INFO("  display_fps: %d", opt_display_fps);
    ROS_INFO("  verbose: %d", opt_verbose);
    ROS_INFO("  debug: %d", opt_debug);
    ROS_INFO("  enable_visual: %d", condition);
    ROS_INFO("  acquisition_fps: %d", acq_fps);
    ROS_INFO("  enable_velocity_control: %d", send_velocity);
    ROS_INFO("  camera_params: px=%f, py=%f, u0=%f, v0=%f", cam_px, cam_py, cam_u0, cam_v0);
  }
}

// build the ibvs task
void ServoPixhawk::BuildTask_param(ros::NodeHandle &nh){
  std::cout << "=========set up task=========" << std::endl;

  // 加载自适应增益参数
  double adaptive_gain_initial, adaptive_gain_min, adaptive_gain_max;
  nh.param("/ibvs_task_param/adaptive_gain/initial", adaptive_gain_initial, 2.0);
  nh.param("/ibvs_task_param/adaptive_gain/min", adaptive_gain_min, 1.5);
  nh.param("/ibvs_task_param/adaptive_gain/max", adaptive_gain_max, 20.0);
  vpAdaptiveGain lambda = vpAdaptiveGain(adaptive_gain_initial, adaptive_gain_min, adaptive_gain_max);
  task.setLambda(lambda);
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe); 
  task.setInteractionMatrixType(vpServo::CURRENT);
  // 加载旋转角度（Rxyz）
  double rotation_x, rotation_y, rotation_z;
  nh.param("/ibvs_task_param/rotation_rxyz/x", rotation_x, -15.0);
  nh.param("/ibvs_task_param/rotation_rxyz/y", rotation_y, 0.0);
  nh.param("/ibvs_task_param/rotation_rxyz/z", rotation_z, 0.0);
  vpRxyzVector c1_rxyz_c(vpMath::rad(rotation_x), vpMath::rad(rotation_y), vpMath::rad(rotation_z));
  vpRotationMatrix c1Rc(c1_rxyz_c);

  // 加载旋转矩阵
  double r11, r12, r13, r21, r22, r23, r31, r32, r33;
  nh.param("/ibvs_task_param/rotation_matrix/r11", r11, 0.0);
  nh.param("/ibvs_task_param/rotation_matrix/r12", r12, -1.0);
  nh.param("/ibvs_task_param/rotation_matrix/r13", r13, 0.0);
  nh.param("/ibvs_task_param/rotation_matrix/r21", r21, 0.0);
  nh.param("/ibvs_task_param/rotation_matrix/r22", r22, 0.0);
  nh.param("/ibvs_task_param/rotation_matrix/r23", r23, -1.0);
  nh.param("/ibvs_task_param/rotation_matrix/r31", r31, 1.0);
  nh.param("/ibvs_task_param/rotation_matrix/r32", r32, 0.0);
  nh.param("/ibvs_task_param/rotation_matrix/r33", r33, 0.0);
  vpRotationMatrix c1Re{r11, r12, r13, r21, r22, r23, r31, r32, r33};

  // 加载平移向量
  double translation_x, translation_y, translation_z;
  nh.param("/ibvs_task_param/translation_vector/x", translation_x, 0.0);
  nh.param("/ibvs_task_param/translation_vector/y", translation_y, 0.0);
  nh.param("/ibvs_task_param/translation_vector/z", translation_z, -0.10);
  vpTranslationVector c1te(translation_x, translation_y, translation_z);

  // 计算齐次矩阵
  vpHomogeneousMatrix c1Mc(vpTranslationVector(), c1Rc);  // Homogeneous matrix between (c1) and (c)
  vpHomogeneousMatrix c1Me(c1te, c1Re);                   // Homogeneous matrix between (c1) and (e)
  vpHomogeneousMatrix cMe = c1Mc.inverse() * c1Me;        // Homogeneous matrix between (c) and (e)
  cVe.buildFrom(cMe);
  task.set_cVe(cVe);

  if (opt_verbose) {
    ROS_INFO("Adaptive Gain: initial=%f, min=%f, max=%f", adaptive_gain_initial, adaptive_gain_min, adaptive_gain_max);
    ROS_INFO("Rotation Rxyz: x=%f, y=%f, z=%f", rotation_x, rotation_y, rotation_z);
    ROS_INFO("Rotation Matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
            r11, r12, r13, r21, r22, r23, r31, r32, r33);
    ROS_INFO("Translation Vector: x=%f, y=%f, z=%f", translation_x, translation_y, translation_z);
  }
}
// build the ibvs task
void ServoPixhawk::BuildTask(){
  cam.initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0); 
  if (opt_verbose) {
    cam.printParameters();
  }
  unsigned int iter = 0;
  current_time = ros::Time::now();
  double time_since_last_display = current_time.toSec() * 1000.0; // 转为毫秒（可选）

  BuildTask_param(nh);

  eJe.resize(6, 4, 0);
  eJe[0][0] = 1;
  eJe[1][1] = 1;
  eJe[2][2] = 1;
  eJe[5][3] = 1;

  // Desired distance to the target, default distance 1 m
  std::cout << "=========Z_d: " << Z_d << "========== "<< std::endl;

  // Define the desired polygon corresponding the the AprilTag CLOCKWISE
  double X[4] = { tagSize_x / 2., tagSize_x / 2., -tagSize_x / 2., -tagSize_x / 2. };
  double Y[4] = { tagSize_y / 2., -tagSize_y / 2., -tagSize_y / 2., tagSize_y / 2. };
  if(opt_verbose){
    std::cout << "X: " << X[0] << ", " << X[1] << ", " << X[2] << ", " << X[3] << std::endl;
    std::cout << "Y: " << Y[0] << ", " << Y[1] << ", " << Y[2] << ", " << Y[3] << std::endl;
  }

  for (int i = 0; i < 4; i++) {
    vpPoint P_d(X[i], Y[i], 0);
    vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
    P_d.track(cdMo); 
    vec_P_d.push_back(P_d);
    if(opt_verbose){
      std::cout << "cdMo: " << cdMo << std::endl;
      std::cout << "P_d[" << i << "]: " << P_d.get_x() << ", " << P_d.get_y() << std::endl;
    }
  }
  man = std::unique_ptr<vpMomentAreaNormalized>(new vpMomentAreaNormalized(0,Z_d)); //
  man_d = std::unique_ptr<vpMomentAreaNormalized>(new vpMomentAreaNormalized(0,Z_d)); //
  // Desired moments
  m_obj = std::make_unique<vpMomentObject>(3);      // 4 points
  m_obj_d = std::make_unique<vpMomentObject>(3);    // 4 points
  m_obj_d->setType(vpMomentObject::DENSE_POLYGON);  // Consider the AprilTag as a polygon
  m_obj_d->fromVector(vec_P_d);                     // Initialize the object with the points coordinates

  mb_d.linkTo(mdb_d);       // Add basic moments to database
  mg_d.linkTo(mdb_d);       // Add gravity center to database
  mc_d.linkTo(mdb_d);       // Add centered moments to database
  man_d->linkTo(mdb_d);     // Add area normalized to database
  mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
  mdb_d.updateAll(*m_obj_d);// All of the moments must be updated, not just an_d
  mg_d.compute();           // Compute gravity center moment
  mc_d.compute();           // Compute centered moments AFTER gravity center

  if (m_obj_d->getType() == vpMomentObject::DISCRETE)
    area = mb_d.get(2, 0) + mb_d.get(0, 2);
  else
    area = mb_d.get(0, 0);
  // Update moment with the desired area
  man_d->setDesiredArea(area);

  man_d->compute(); // Compute area normalized moment AFTER centered moments
  mgn_d.compute();  // Compute gravity center normalized moment AFTER area normalized moment

  // Desired plane
  C = 1.0 / Z_d;

  // Construct area normalized features
  s_mgn = std::make_unique<vpFeatureMomentGravityCenterNormalized>(mdb, A, B, C);
  s_mgn_d = std::make_unique<vpFeatureMomentGravityCenterNormalized>(mdb_d, A, B, C);
  s_man = std::make_unique<vpFeatureMomentAreaNormalized>(mdb, A, B, C);
  s_man_d = std::make_unique<vpFeatureMomentAreaNormalized>(mdb_d, A, B, C);
  s_vp_d = std::make_unique<vpFeatureVanishingPoint>();
  s_vp = std::make_unique<vpFeatureVanishingPoint>();
  // Add the features
  task.addFeature(*s_mgn, *s_mgn_d);
  task.addFeature(*s_man, *s_man_d);
  task.addFeature(*s_vp, *s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());


  // plotter.initGraph(0, 4);
  // plotter.setLegend(0, 0, "Xn");          // Distance from center on X axis feature
  // plotter.setLegend(0, 1, "Yn");          // Distance from center on Y axis feature
  // plotter.setLegend(0, 2, "an");          // Tag area feature
  // plotter.setLegend(0, 3, "atan(1/rho)"); // Vanishing point feature

  // Update desired gravity center normalized feature

  s_mgn_d->update(A, B, C);
  s_mgn_d->compute_interaction();
  // Update desired area normalized feature
  s_man_d->update(A, B, C);
  s_man_d->compute_interaction();

  // Update desired vanishing point feature for the horizontal line
  s_vp_d->setAtanOneOverRho(0);
  s_vp_d->setAlpha(0);
  std::cout << "========= task set up finish =========" << std::endl;

}

// iteration of ibvs control law
void ServoPixhawk::run(){
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
    m_obj->setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
    m_obj->fromVector(vec_P);                      // Initialize the object with the points coordinates

    mg.linkTo(mdb);           // Add gravity center to database
    mc.linkTo(mdb);           // Add centered moments to database
    man->linkTo(mdb);         // Add area normalized to database
    mgn.linkTo(mdb);          // Add gravity center normalized to database
    mdb.updateAll(*m_obj);    // All of the moments must be updated, not just an_d
    mg.compute();             // Compute gravity center moment
    mc.compute();             // Compute centered moments AFTER gravity center
    man->setDesiredArea(area);// Desired area was init at 0 (unknow at construction),need to be updated here
    man->compute();           // Compute area normalized moment AFTER centered moment
    mgn.compute();            // Compute gravity center normalized moment AFTER area normalized moment

    s_mgn->update(A, B, C);
    s_mgn->compute_interaction();
    s_man->update(A, B, C);
    s_man->compute_interaction();

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
    vpFeatureBuilder::create(*s_vp, cam, vec_ip[vec_ip_sorted[1].first],vec_ip[vec_ip_sorted[3].first],
                             vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[2].first], 
                             vpFeatureVanishingPoint::selectAtanOneOverRho());
    task.set_cVe(cVe);
    task.set_eJe(eJe);

    // Compute the control law. Velocities are computed in the mobile robot reference frame
    vpColVector ve = task.computeControlLaw();
    if (!send_velocity)
      ve = 0;
        
    std::cout << "[[send_velocity]]:" << send_velocity << std::endl;
    if (opt_verbose) {
      std::cout << "[[ve]]: " << ve.t() << std::endl;
    }

    if(send_velocity) {
      setVelocity(ve);
      if (opt_verbose) {
        std::cout << "[[ve]]: " << ve.t() << std::endl;
      }
    }

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

      if(send_velocity){

        geometry_msgs::TwistStamped twist;
        twist.header.stamp = ros::Time::now();
        twist.twist.linear.x = 0;
        twist.twist.linear.y = 0;
        twist.twist.linear.z = 0;
        twist.twist.angular.z = 0;
        vel_pub.publish(twist);
        std::cout << "Published velocity command: "<< std::endl << twist << std::endl;
      }
    }
  if (condition) {
    std::stringstream ss;
    ss << "Left click to " << (send_velocity ? "stop the robot" : "servo the robot")
      << ", right click to quit.";
    vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
    
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
    debugPrint(task, s_mgn.get(), s_mgn_d.get(), s_man.get(), s_man_d.get());
  }
}


// check delay
void ServoPixhawk::checkDetectionTimeout() {
    ros::Time now = ros::Time::now();
    double elapsed = (now - last_detection_time).toSec();
    std::cout << "[[elapsed]]" << elapsed <<std::endl;
    if (elapsed > detection_timeout && has_detection) {
      has_detection = false;
      std::cout << "[Timeout] No detection for " << elapsed << " seconds. Setting has_detection to false." << std::endl;
    }
}

// publish velocity
void ServoPixhawk::setVelocity(const vpColVector &ve) {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = ve[0];
    twist.twist.linear.y = ve[1];
    twist.twist.linear.z = ve[2];
    twist.twist.angular.z = ve[3];
    vel_pub.publish(twist);
}

// recieve object boungingbox 4 pixel points
void ServoPixhawk::yoloCallback(const custom_msgs::RectMsg& msg) {
  bbox = msg;
  last_detection_time = ros::Time::now();
  if (msg.xmin >= msg.xmax || msg.ymin >= msg.ymax ) {
    std::cout << "[[yolo-error111]]" << std::endl;
    has_detection = false;
    return;
  }
  if(image_received){
      vpDisplay::displayRectangle(I, vpImagePoint(bbox.ymin, bbox.xmin), vpImagePoint(bbox.ymax, bbox.xmax), vpColor::red, 0.2, 0.1);
      vpDisplay::flush(I);
  }
  has_detection = true;
  std::cout << "====yoloCallback Detection====: " << has_detection << std::endl; 
}

// make sure uav is connected
void ServoPixhawk::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  if(msg->connected && image_received) {
    drone_isRunning = true;
  }
  else {
    drone_isRunning = false;
  }
  std::cout << "Drone state: " << (drone_isRunning ? "Run" : "Disconnected") << std::endl;
  current_state = *msg;
}

// receive image from camera
void ServoPixhawk::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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
  } 
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

// make sure the points are sorted by y coordinate
bool ServoPixhawk::compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
  return (p1.second.get_v() < p2.second.get_v());
};

// print debug info
void ServoPixhawk::debugPrint(const vpServo& task, 
  const vpFeatureMomentGravityCenterNormalized* s_mgn, 
  const vpFeatureMomentGravityCenterNormalized* s_mgn_d,
  const vpFeatureMomentAreaNormalized* s_man, 
  const vpFeatureMomentAreaNormalized* s_man_d)
{
  std::cout << "------------------------------------" << std::endl;
  std::cout << "Task error: " << task.getError().t() << std::endl;
  // 打印当前和期望的重心归一化特征
  if (s_mgn) {
    std::cout << "Current gravity center: " << std::endl << s_mgn->get_s() << std::endl;
    } else {
    std::cout << "Current gravity center: [NULL]" << std::endl;
  }
  if (s_mgn_d) {
    std::cout << "Desired gravity center: " << std::endl << s_mgn_d->get_s() << std::endl;
    } else {
    std::cout << "Desired gravity center: [NULL]" << std::endl;
  }

  // 打印当前和期望的面积归一化特征
  if (s_man) {
    std::cout << "Current normalized area: " << std::endl << s_man->get_s() << std::endl;
    } else {
    std::cout << "Current normalized area: [NULL]" << std::endl;
  }
  if (s_man_d) {
    std::cout << "Desired normalized area: " << std::endl << s_man_d->get_s() << std::endl;
    } else {
    std::cout << "Desired normalized area: [NULL]" << std::endl;
  }
  std::cout << "------------------------------------" << std::endl;
}