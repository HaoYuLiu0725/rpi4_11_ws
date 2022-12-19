#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

std::string pkg_path = ros::package::getPath("rviz_display");
std::string pic_path = pkg_path + "/map/project_map_2000.png";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_image");
    ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(pic_path);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("map_image", 10);
    ros::Rate loop_rate(5);

    while (nh.ok())
    {
        pub.publish(ros_image);
        loop_rate.sleep();
    }
}