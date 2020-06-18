# pyrobot_minisubt   
Repo for hcc final project.   
start_tx2 & locobot hcc_base   

laptop $ roslaunch teleop_twist_joy joy.launch    

tx2 $ roslaunch localization localization.launch   
tx2 $ rosrun object_localization d435_object_dis.py    

laptop $ rosrun object_localization joy_trigger.py    
laptop $ rosrun image_transport republish compressed in:=/teleop/image raw out:=/teleop/image    
laptop $ roslaunch darknet_ros yolo_v4.launch    
