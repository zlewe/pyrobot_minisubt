# pyrobot_minisubt
### Repo for hcc final project.
> This repo does not contain necessary codes to start tx2 and locobot!

**Prerequisite:**
- Darknet_ROS dependencies:
    - boost
    - OpenCV
    - gcc-6 / gcc-7
    - CUDA <= 10.0 (may or may not work without CUDA)

**Download weights for YoloV4:**
Path: pyrobot_minisubt/darknet_ros/darknet_ros/yolo_network_config/weights/

gdrive: [yolo-obj_best.weights](https://drive.google.com/file/d/1VoZ6D7qP23LdjM_y_O0KyHcHyu6xtbz4/view?usp=sharing)

**Build the project:**
catkin_make -DCMAKE_BUILD_TYPE=Release 

*If there is issue with gcc version，build with specifying gcc 6:*
> -DCMAKE_C_COMPILER=/usr/bin/gcc-6

**Step 0: Start tx2 and locobot then launch joy teleoperation node**
> *Not needed if using rosbags*
- tx2 $ source start_tx2.sh & locobot $ hcc_base
- laptop $ roslaunch teleop_twist_joy joy.launch

**Step 1: Start Localization**
- tx2 $ roslaunch localization localization.launch
- tx2 $ rosrun object_localization d435_object_dis.py

**Step 2: Launch Object Detection**
- laptop $ rosrun image_transport republish compressed in:=/teleop/image raw out:=/teleop/image
- laptop $ roslaunch darknet_ros yolo_v4.launch

> if using rosbags, change the image topic to /camera/color/image, and does not need to republish /teleop/image 

**Step 3: Launch service client for calling /trigger_save**
- laptop $ rosrun object_localization joy_trigger.py
> use button B to save entry to dict. (dict is then dumped into json file when the d435_object_dis.py node exits.)

## Explanation on each nodes
1. **Localization**

a. Map_tf node: hcc_final_map.launch
- Publish tf for every map apriltags to origin.

Subscribe | Publish
--------- | ----------
--       | /tf_static


b. Apriltag detect node: detect_apriltag.launch
- Detect apriltags from camera stream and publish their tf with respect to the camera frame

Subscribe | Publish
--------- | ----------
/camera/color/image_raw | /tf
/camera/color/camera_info | /tag_detections
-- | /tag_detections_image

c. Localization node: apriltag_lab.cpp
- Calculate and update tf from origin to map (local frame of locobot)
- Also publish MarkerArray for visualization

Subscribe | Publish
--------- | ----------
/tf | /tf
/tf_static | /robot_pose_from_tag


2. **Object Localization**

a. d435_object_dis.py
- Calculate location of object based on bounding_box， depth_image and tf.
- Also, save the informations when service is called, then dumps all the informatioms into json when node is killed.

Subscribe | Publish
--------- | ----------
/tf | -
/camera/color/camera_info | -
/camera/color/aligned_depth_to_color/image_raw | -
/darknet_ros/detection_image | -
/darknet_ros/bounding_boxes | -

**Service server:** /trigger_save

3. **Object Detection**
- Detect object using Yolov4

Subscribe | Publish
--------- | ----------
/teleop/image | /darknet_ros/object_detector
-- | /darknet_ros/detection_image
-- | /darknet_ros/bounding_boxes
