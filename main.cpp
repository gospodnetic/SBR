/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-16 10:17:41
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (
        new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("test.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file.\n");
        return -1;
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from the .pcd file with the following fields: "
        << std::endl;

    for(size_t i = 0; i < cloud->points.size(); i++)
        std::cout << "    " << cloud->points[i].x
            << " "    << cloud->points[i].y
            << " "    << cloud->points[i].z
            << " "    << cloud->points[i].rgb
            << std::endl;

    return 0;
}   