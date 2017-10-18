/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-18 13:47:20
*/
#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cinema_image.h"
#include "pcl_utils.h"

int main()
{
    test_lodepng("/home/petra/Desktop/SampleBasedReconstruction/screenshot-1508257412.png");

    // Read in the cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = 
        pcl_utils::open_RGBpcd("test.pcd");
	
    // Generate a point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud =
        pcl_utils::generate_sphere_cloud(5000);

    //
    // Reconstruct the surface.
    //
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals =
        pcl_utils::estimate_normals(sphere_cloud);

    pcl::PolygonMesh mesh = pcl_utils::greedy_surface_reconstruct(
        cloud_with_normals,
        0.2,
        5.5,
        100,
        M_PI / 18,
        2 * M_PI / 3,
        false);

    //  
    // Visualize the cloud.
    //

    // Initialize the viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        rgb(sphere_cloud);
    viewer->addPolygonMesh(mesh, "sample mesh");

    // viewer->addPointCloud<pcl::PointXYZRGB>(sphere_cloud, rgb, "sample cloud");
    // viewer->setPointCloudRenderingProperties(
    //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //     3,
    //     "sample cloud");
    
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "sample mesh");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Run the main loop.
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(1);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    return 0;
}   