#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    cv::Mat mask = cv::Mat(480, 752, CV_8UC1, cv::Scalar(255));
    // cv::Mat mask = cv::Mat(768, 1024, CV_8UC1, cv::Scalar(255));
    // cv::Point p1,p2;
    // cv::Point p3,p4;
    // cv::Point p5,p6,p7;
    // p1.x = 300;
    // p1.y = 0;
    // p2.x = 724;
    // p2.y = 0;
    // p3.x = 200;
    // p3.y = 0;
    // p4.x = 824;
    // p4.y = 0;
    // p5.x = 400;
    // p5.y = 0;
    // p6.x = 624;
    // p6.y = 0;
    // p7.x = 512;
    // p7.y = 0;
    // cv::circle(mask, p1, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p2, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p3, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p4, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p5, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p6, 100, cv::Scalar(0),-1,8);
    // cv::circle(mask, p7, 100, cv::Scalar(0),-1,8);
    cv::Point p1, p2;
    p1.x = 0;
    // p1.y = 380;
    p1.y = 319;
    p2.x = 751;
    p2.y = 479;
    cv::rectangle(mask, p1, p2, cv::Scalar(0), -1, 0);
    cv::Point p3, p4;
    p3.x = 100;
    // p1.y = 380;
    p3.y = 0;
    p4.x = 551;
    p4.y = 479;
    cv::rectangle(mask, p1, p2, cv::Scalar(0), -1, 0);
    imwrite("test.jpg",mask);
}