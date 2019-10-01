#ifndef CLOUD_H
#define CLOUD_H

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class Cloud
{
public:
    void readPCloud(std::string filename, ros::NodeHandle &n);
    sensor_msgs::PointCloud2 pCloud2Msg;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Timer particleTimer;
    ros::NodeHandle _n;

private:

    void timerCallback(const ros::TimerEvent&)
    {
        publishPointCloud();
    }

    void publishPointCloud()
    {
        sensor_msgs::PointCloud2 particleCloudMsg;
        particleCloudMsg.header.stamp = ros::Time::now();
        particleCloudMsg.header.frame_id = "cloud";
        cloud_pub.publish(particleCloudMsg);
    }
};

#endif// CLOUD_H
