#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <Eigen/Eigen>
#include <grid_map_msgs/GridMap.h>
#include <string>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace grid_map;

// load the grid map from file
// this will be the bag.file previously recorded
grid_map_msgs::GridMap loadGridMapFromFile(std::string gridMapBagName)
{
    grid_map_msgs::GridMap output;
    //reading the bag where the grid map is recorded
    rosbag::Bag gridBag;
    gridBag.open(gridMapBagName, rosbag::bagmode::Read);

    std::vector<std::string> gridTopic;
    gridTopic.push_back(std::string("grid_map"));
    rosbag::View view(gridBag, rosbag::TopicQuery(gridTopic));

    foreach(rosbag::MessageInstance const m, view)
    {
        grid_map_msgs::GridMap::ConstPtr s = m.instantiate<grid_map_msgs::GridMap>();
        if (s != nullptr)
            output = *s;
    }

    gridBag.close();
    return output;

    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map_loader");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    grid_map_msgs::GridMap msg = loadGridMapFromFile("/home/emanuele/Desktop/grid_map_example.bag");
    ros::Rate rate(.1);
    while (nh.ok()) {
        ros::Time time = ros::Time::now();
        pub.publish(msg);
        rate.sleep();
        (void) time;
    }
    return 0;
}
