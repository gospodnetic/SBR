/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-18 10:36:09
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-18 10:36:09
*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pcl_utils
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr open_RGBpcd(
        const std::string filename);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_sphere_cloud(
        const size_t    N = 1000,
        const size_t    r = 1,
        const bool      uniform = true);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimate_normals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    pcl::PolygonMesh greedy_surface_reconstruct(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
        const double    search_radius,
        const double    mu,
        const size_t    max_nearest_neighbor,
        const double    max_surf_angle,
        const double    max_angle,
        const bool      normal_consistency);
}