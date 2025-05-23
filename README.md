# IBVS_Drone
The project is based on Ubuntu 20.04 system, ROS noetic, and implements tracking the recognize target form yolo.\
[video of simulation in gazebo](https://www.bilibili.com/video/BV1UCLqzkENt/)\
**Required data & params**
1. Four image coordinate points from yolo, data type is customized message type.
>Note that no depth information of the target is needed.

2. Camera parameters.
3. c1Re: Rotation matrix from the local coordinate system to the camera coordinate system.
4. c1te: Translation between (c1) and (e).
5. c1_rxyz_c: Rotation angle between the real camera coordinate system and the virtual camera coordinate system.(c1) is an intermediate frame attached to the camera that has axis aligned with the FLU body frame. The real camera frame is denoted (c).
6. tag_size: Length and width of the actual target object, in meters.
7. distance_to_tag: Expected distance between UAV and target object, in meters.
>View more at config.yaml

**Topic**\
`/camera/color/image_raw`\
`/mavros/state`\
`/object_detection/yolo`\
`/mavros/setpoint_velocity/cmd_vel`

key reference [Image-based visual-servoing on a drone equipped with a Pixhawk](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-pixhawk-vs.html)
