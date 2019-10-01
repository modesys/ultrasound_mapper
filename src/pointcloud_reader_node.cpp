#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "../include/cloud.h"
#include <rosbag/bag.h>
#include <memory.h>
using namespace std;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "pcl_tutorial_cloud");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1000);
    ros::Rate loop_rate(1);
    rosbag::Bag bag;
    bag.open("cloud.bag", rosbag::bagmode::Write);
    std::string fstring = "/home/emanuele/Desktop/cloud_25.pcd";
    Cloud p;
    int count = 0;
    while(ros::ok())
    {
        sensor_msgs::PointCloud2 pcloud2;
        pub.publish(pcloud2);
        bag.write("cloud/data", p.pCloud2Msg.header.stamp , p.pCloud2Msg);
        ros::spinOnce();
        loop_rate.sleep();
        count ++;
    }
    return 0;
}
