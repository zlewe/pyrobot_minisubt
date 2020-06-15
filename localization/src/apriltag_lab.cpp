#include <stdlib.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

visualization_msgs::Marker marker;
visualization_msgs::MarkerArray marker_array;
tf::StampedTransform echo_transform;
tf::TransformListener *tf_listener;
tf::StampedTransform min_distance_trans;
tf::Transform localization_trans;
tf::Transform origintoodom_trans;
tf::TransformBroadcaster *br;
std::pair<int, double> min_distance (-1,100.0);

double get_distance(tf::Vector3 v){
    return sqrt(v.getX()*v.getX() + v.getY()*v.getY() + v.getZ()*v.getZ());
}

void listener(){
    min_distance.first = -1;
    min_distance.second = 100.0;
    // use tf_listener to get the transformation from base_link to tag 0
    for (int i = 0; i<16;i++){
        string parent_id = "base_link";
        string child_id = "tag_" + std::to_string(i);
        // string child_id = "tag_0";
        tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.1));
        try {

            tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), echo_transform);
            //std::cout << "At time " << std::setprecision(16) << echo_transform.stamp_.toSec() << std::endl;
            //cout << "Frame id:" << echo_transform.frame_id_ << ", Child id:" << echo_transform.child_frame_id_ << endl;
            double yaw, pitch, roll;
            echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Quaternion q = echo_transform.getRotation();
            tf::Vector3 v = echo_transform.getOrigin();
            //std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            //std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
            //        << q.getZ() << ", " << q.getW() << "]" << std::endl;
            
            /*
                please calucluate the distance from tag_0 to other tags
            */
            double distance = get_distance(v);
            //std::cout << "Distance: " << distance << std::endl;

            /*
                find the closet distance from the tag to base_link(remember to modify the parent_id)
            */
            if (distance < min_distance.second){
                min_distance.first = i;
                min_distance.second = distance;
                min_distance_trans = echo_transform;
            }

        }
        catch (tf::TransformException& ex)
        {
            std::cout << "Exception thrown:" << ex.what() << std::endl;
        }
	}

    /*
        find the robot position from map_tag_0
    */
    string parent_id = "origin";
    string child_id = "map_tag_" + std::to_string(min_distance.first);
    tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.001));
    try {

        tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), echo_transform);
        localization_trans = echo_transform*min_distance_trans.inverse();
        /*
            find transformation matrix from echo_transform and min_distance_trans
        */    
        //cout << "=================================\n";
        //cout << "*get the robot position in map*\n";
        //cout << "rotation:\n";
        tf::Quaternion q = localization_trans.getRotation();
        //cout << "[" << q.getX() << ", " << q.getY() << ", "<< q.getZ() << ", " << q.getW() << "]" << endl; 
        tf::Vector3 v = localization_trans.getOrigin();
        //cout << "translation:\n";
        //cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
        //cout << "=================================\n";
        marker.pose.orientation.x = q.getX();
        marker.pose.orientation.y = q.getY();
        marker.pose.orientation.z = q.getZ();
        marker.pose.orientation.w = q.getW();
        marker.pose.position.x = v.getX();
        marker.pose.position.y = v.getY();
        marker.pose.position.z = v.getZ();
        marker_array.markers.push_back(marker);
        marker.id += 1;
    }
    catch (tf::TransformException& ex)
    {
        std::cout << "Exception thrown:" << ex.what() << std::endl;
    }

    //get odom_to_baselink
    parent_id = "map";
    child_id = "base_link";
    tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.001));
    try {
        tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), echo_transform);
        origintoodom_trans = localization_trans*echo_transform.inverse();
        br->sendTransform(tf::StampedTransform(origintoodom_trans, ros::Time::now(), "origin", "map"));
        ROS_LOGINFO("Sending tf");
        /*
            find transformation matrix from echo_transform and min_distance_trans
        */    
        //cout << "=================================\n";
        //cout << "*get the robot position in map*\n";
        //cout << "rotation:\n";
        //tf::Quaternion q = localization_trans.getRotation();
        //cout << "[" << q.getX() << ", " << q.getY() << ", "<< q.getZ() << ", " << q.getW() << "]" << endl; 
        //tf::Vector3 v = localization_trans.getOrigin();
        //cout << "translation:\n";
        //cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
        //cout << "=================================\n";
    }
    catch (tf::TransformException& ex)
    {
        std::cout << "Exception thrown:" << ex.what() << std::endl;
    }

    return ;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "apriltag_tf");
    ros::NodeHandle nh;
    ros::Subscriber tf_sub;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "robot_pose_from_tag", 0);
    // tf_sub = nh.subscribe("tf_static",16,&tf_cb);
    tf_listener = new tf::TransformListener();
    br = new tf::TransformBroadcaster();

    marker.header.frame_id = "origin";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    /*marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;*/
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    while (ros::ok())
    {
        ros::spinOnce();
        listener();
        vis_pub.publish(marker_array);
    }
    
    return 0;
}
