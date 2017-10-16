/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-16 10:45:11
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>

int main()
{
    // Read in the cloud.
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

    // Print the cloud.
    for(size_t i = 0; i < cloud->points.size(); i++)
        std::cout << "    " << cloud->points[i].x
            << " "    << cloud->points[i].y
            << " "    << cloud->points[i].z
            << " "    << cloud->points[i].rgb
            << std::endl;

    //  
    // Visualize the cloud.
    //

    // Initialize the viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Run the main loop.
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    return 0;
}   