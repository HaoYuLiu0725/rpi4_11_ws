#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

geometry_msgs::Twist input_twist;
geometry_msgs::Twist twist;
nav_msgs::Odometry output_odom;
ros::Publisher pose_pub;
ros::Publisher odom_pub;

void updateTwist()
{
  twist.linear.x = input_twist.linear.x;
  twist.linear.y = input_twist.linear.y;
  twist.angular.z = input_twist.angular.z;

  output_odom.twist.twist = twist;
}

void updatePose(const ros::TimerEvent& e)
{
  double dt = (e.current_expected - e.last_expected).toSec();

  double dx = twist.linear.x * dt;
  double dy = twist.linear.y * dt;
  double dw = twist.angular.z * dt;

  double yaw = tf2::getYaw(output_odom.pose.pose.orientation);

  output_odom.pose.pose.position.x += (dx * cos(yaw) - dy * sin(yaw));
  output_odom.pose.pose.position.y += (dx * sin(yaw) + dy * cos(yaw));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw + dw);
  output_odom.pose.pose.orientation = tf2::toMsg(q);
}

void publish()
{
  /* odom */
  ros::Time now = ros::Time::now();
  output_odom.header.stamp = now;
  // output_odom.header.frame_id = p_fixed_frame_id_;
  // output_odom.child_frame_id = p_target_frame_id_;
  // if (p_publish_odom_)
    odom_pub.publish(output_odom);

  /* pose */
  static geometry_msgs::PoseWithCovarianceStamped pose;

  pose.header.stamp = now;
  pose.header.frame_id = "map";
  pose.pose = output_odom.pose;
  // if (p_publish_pose_)
    pose_pub.publish(pose);

  /* tf */
  static geometry_msgs::TransformStamped transform;
  // transform.header.frame_id = p_fixed_frame_id_;
  transform.header.stamp = now;
  // transform.child_frame_id = p_target_frame_id_;

  transform.transform.translation.x = output_odom.pose.pose.position.x;
  transform.transform.translation.y = output_odom.pose.pose.position.y;
  transform.transform.rotation.w = output_odom.pose.pose.orientation.w;
  transform.transform.rotation.x = output_odom.pose.pose.orientation.x;
  transform.transform.rotation.y = output_odom.pose.pose.orientation.y;
  transform.transform.rotation.z = output_odom.pose.pose.orientation.z;

  // if (p_publish_tf_)
  //   tf2_broadcaster_.sendTransform(transform);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& ptr)
{
  input_twist = *ptr;
  // last_time = ros::Time::now();
}

void timerCallback(const ros::TimerEvent& e)
{
  updateTwist();
  updatePose(e);
  publish();
}

int main(int argc, char **argv)
{
  //node initialization
  ros::init(argc, argv, "odomtery");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe("/base_speed", 10, &twistCallback);
  pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom_pose", 10);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), &timerCallback);

  /* init state param */
  double p_init_pose_x = 0.5;
  double p_init_pose_y = -0.15;
  double p_init_pose_yaw = M_PI / 2;
  output_odom.pose.pose.position.x = p_init_pose_x;
  output_odom.pose.pose.position.y = p_init_pose_y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p_init_pose_yaw);
  output_odom.pose.pose.orientation = tf2::toMsg(q);

  ros::spin();
}