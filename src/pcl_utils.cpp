/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-18 10:36:09
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-18 13:45:13
*/

#include "pcl_utils.h"

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

namespace pcl_utils
{
    /*! \brief Wrapped adjusted example PCL code.
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr open_RGBpcd(
        const std::string filename)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read pcd file.\n");
            return cloud;
        }
        std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from the .pcd file."
            << std::endl;

        return cloud;
    }

    /*! \brief Generate a point cloud on the surface of a sphere at coordinate
                soace origin.
        \param N number of points in the cloud
        \param r sphere radius
        \param uniform how the points are generated. TRUE - uniformly
                distributed over the surface of the sphere. FALSE - uniformly
                distributed over the (phi, theta) plane, but more condensed 
                around the sphere poles.
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_sphere_cloud(
        const size_t    N,
        const size_t    r,
        const bool      uniform)
    {
        // Generate a sphere point cloud.
        const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::mt19937 generator(seed);
        std::uniform_real_distribution<double> uniform01(0.0, 1.0);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        // Fill in the cloud data
        sphere_cloud->is_dense = false;
        sphere_cloud->points.resize(N);

        // TODO: Avoid code duplication.
        if(uniform)
            for(size_t i = 0; i < N; i++)
            {
                const double theta = 2 * M_PI * uniform01(generator);
                const double phi = acos(1 - 2 * uniform01(generator));
                sphere_cloud->points[i].x = r * sin(phi) * cos(theta);
                sphere_cloud->points[i].y = r * sin(phi) * sin(theta);
                sphere_cloud->points[i].z = r * cos(phi);
                sphere_cloud->points[i].r = int(255 * uniform01(generator));
                sphere_cloud->points[i].g = int(255 * uniform01(generator));
                sphere_cloud->points[i].b = int(255 * uniform01(generator));
            }
        else
            for(size_t i = 0; i < N; i++)
            {
                const double theta = 2 * M_PI * uniform01(generator);
                const double phi = M_PI * uniform01(generator);
                sphere_cloud->points[i].x = r * sin(phi) * cos(theta);
                sphere_cloud->points[i].y = r * sin(phi) * sin(theta);
                sphere_cloud->points[i].z = r * cos(phi);
                sphere_cloud->points[i].r = int(255 * uniform01(generator));
                sphere_cloud->points[i].g = int(255 * uniform01(generator));
                sphere_cloud->points[i].b = int(255 * uniform01(generator));
            }

        return sphere_cloud;
    }

    /*! \brief Wrapped adjusted example PCL code.
    */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimate_normals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
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

        return cloud_with_normals;
    }

    /*! \brief Wrapped adjusted example PCL code.
    */
    pcl::PolygonMesh greedy_surface_reconstruct(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr  cloud,
        const double    search_radius,
        const double    mu,
        const size_t    max_nearest_neighbor,
        const double    max_surf_angle,
        const double    max_angle,
        const bool      normal_consistency)
    {
        // Greedy surface reconstruction.
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree->setInputCloud(cloud);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points.
        gp3.setSearchRadius(search_radius);

        // Set typical values for the parameters
        gp3.setMu(mu);
        gp3.setMaximumNearestNeighbors(max_nearest_neighbor);
        gp3.setMaximumSurfaceAngle(max_surf_angle);
        gp3.setMaximumAngle(max_angle);
        gp3.setNormalConsistency(normal_consistency);

        // Reconstruct.
        gp3.setInputCloud(cloud);
        gp3.setSearchMethod(tree);
        gp3.reconstruct(triangles);

        // The reconstructed mesh can be exported as .vtk file and alse viewed
        // with the pcl_viewer.
        // pcl::io::saveVTKFile("reconstructed.vtk", triangles);

        return triangles;
    }
} // !namespace pcl_utils