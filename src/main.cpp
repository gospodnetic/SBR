/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-23 13:53:44
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
    // Takes about a second to read the depth image.
    // TODO: pass a pointer Instead of using swap file.
    std::vector<std::vector<float>> depth_image = cinema::read_depth_image();
    //
    // Get max element of a 2D vector.
    //
    std::vector<float> column_maxs;
    for(std::vector<std::vector<float>>::const_iterator row=depth_image.begin(); row!=depth_image.end(); row++)
    {
        column_maxs.push_back(*std::max_element(row->begin(), row->end()));
    }
    const float max_depth = *std::max_element(std::begin(column_maxs), std::end(column_maxs));

    // Camera near far out of info.json.
    const double camera_near = 2.305517831184482;
    const double camera_far = 4.6363642410628785;
    const double near_far_step = (camera_far - camera_near) / max_depth;
    
    // Map depth values to camera near/far space.
    // Currently not being mapped because of the scale in pcl cloud.
    // TODO: Remove far plane from the point cloud
    // TODO: Rotate the depth image according to the theta and phi.
    // for(std::vector<std::vector<float>>::iterator row=depth_image.begin(); row!=depth_image.end(); row++)
    // {
    //     for(std::vector<float>::iterator col=row->begin(); col!=row->end(); col++)
    //         *col = *col / max_depth;
    // }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_depth_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    image_depth_cloud->points.resize(depth_image.size() * depth_image[0].size());

    // Generate the point cloud out of the depth values.
    size_t idx = 0;
    for(std::vector<std::vector<float>>::iterator row=depth_image.begin(); row!=depth_image.end(); row++)
    {
        for(std::vector<float>::iterator col=row->begin(); col!=row->end(); col++)
        {
            image_depth_cloud->points[idx].z = col - row->begin();
            image_depth_cloud->points[idx].y = row - depth_image.begin();
            image_depth_cloud->points[idx].x = *col; // Switched with z just so that I don't have to rotate it every time when vieweing
            image_depth_cloud->points[idx].r = *col;
            image_depth_cloud->points[idx].g = 50;
            image_depth_cloud->points[idx].b = 50;
            idx++;
        }
            
    }

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