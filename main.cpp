/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
<<<<<<< Updated upstream
* @Last Modified time: 2017-10-16 17:13:49
=======
* @Last Modified time: 2017-10-16 16:39:11
>>>>>>> Stashed changes
*/
#include <iostream>
#include <random>
#include <cmath>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
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
        << " data points from the .pcd file."
        << std::endl;

    // Generate a sphere point cloud.
    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);

    // Generate N numbers
    const int N = 1000;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    // cloud.width    = 5;
    // cloud.height   = 1;
    sphere_cloud->is_dense = false;
    sphere_cloud->points.resize(N);

    for(size_t i = 0; i < N; i++)
    {
        const double theta = 2 * M_PI * uniform01(generator);
        const double phi = M_PI * uniform01(generator);
        sphere_cloud->points[i].x = sin(phi) * cos(theta);
        sphere_cloud->points[i].y = sin(phi) * sin(theta);
        sphere_cloud->points[i].z = cos(phi);
        sphere_cloud->points[i].r = int(255 * uniform01(generator));
        sphere_cloud->points[i].g = int(255 * uniform01(generator));
        sphere_cloud->points[i].b = int(255 * uniform01(generator));
    }

    //
    // Reconstruct the surface.
    // For now we need to use PointXYZ type instead of the RGB type
    //

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate the point data with the normal fields.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Update search tree.
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(
        new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points.
    gp3.setSearchRadius(0.1);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);;
    gp3.setMaximumSurfaceAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);

    // Reconstruct.
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Export the mesh as .vtk file ()see with pcl_viewer.
    pcl::io::saveVTKFile("reconstructed.vtk", triangles);
    
    //  
    // Visualize the cloud.
    //

    // Initialize the viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        rgb(sphere_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(sphere_cloud, rgb, "sample cloud");
    viewer->addPolygonMesh(triangles, "sample mesh");

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "sample cloud");
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