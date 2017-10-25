/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-25 12:23:10
*/

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cinema_db.h"
#include "pcl_utils.h"

int main()
{   
    // Static paths to the image db structure in folders.
    const std::string db_path = "/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image";
    const std::string db_label = "colorSphere1";

    cinema::CinemaDB cinema_db(
        db_path,
        db_label,
        2.305517831184482,
        4.6363642410628785,
        3);

    // Concatenate all the depth values into a single cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_depth_cloud = cinema_db.point_cloud_rgb(3);
    
    // // Read the image.
    // cinema::CinemaImage cinema_image1("/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz",
    //     0,
    //     0);
    // cinema::CinemaImage cinema_image2("/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz",
    //     90,
    //     0);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_depth_cloud(
    //     new pcl::PointCloud<pcl::PointXYZRGB>);

    // *image_depth_cloud += *(cinema_image1.point_cloud_rgb());
    // *image_depth_cloud += *(cinema_image2.point_cloud_rgb());

    //  
    // Visualize the cloud.
    //

    // Initialize the viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        rgb(image_depth_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(image_depth_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "sample cloud");


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