#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_transformListener");

  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/map_pose", 10);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped stampedPose;
  
  ros::Rate rate(100.0);
  while (nh.ok()){
    try{
      transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    stampedPose.header = transformStamped.header;
    stampedPose.pose.position.x = transformStamped.transform.translation.x;
    stampedPose.pose.position.y = transformStamped.transform.translation.y;
    stampedPose.pose.orientation = transformStamped.transform.rotation;

    pose_pub.publish(stampedPose);

    // ROS_INFO_STREAM("[Trans pose]:" << stampedPose.pose.position.x << "," << stampedPose.pose.position.y << "," << tf2::getYaw(stampedPose.pose.orientation));
    
    rate.sleep();
  }
  return 0;
};