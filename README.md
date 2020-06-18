# pyrobot_minisubt
Repo for hcc final project.＜/br＞
start_tx2 & locobot hcc_base＜/br＞
laptop $ roslaunch teleop_twist_joy joy.launch＜/br＞
tx2 $ roslaunch localization localization.launch＜/br＞
tx2 $ rosrun object_localization d435_object_dis.py＜/br＞
laptop $ rosrun object_localization joy_trigger.py＜/br＞
laptop $ rosrun image_transport republish compressed in:=/teleop/image raw out:=/teleop/image＜/br＞
laptop $ roslaunch darknet_ros yolo_v4.launch＜/br＞
