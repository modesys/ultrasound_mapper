#include "cloud.h"

void Cloud::readPCloud(std::string filename, ros::NodeHandle &n)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load point cloud file
    {
        PCL_ERROR("Could not read the file");
        return;
    }
    std::cout<<"Loaded"<<cloud->width * cloud->height
             <<"data points from /home/emanuele/Desktop/cloud_25.pcd "
             <<std::endl;
    for(size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;

    particleTimer = n.createTimer(ros::Duration(1), &Cloud::timerCallback, this);
}
