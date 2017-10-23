/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:32:07
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-17 16:32:07
*/

#ifndef CINEMA_IMAGE_P
#define CINEMA_IMAGE_P

#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace cinema
{
    class CinemaImage
    {
      public:
        // Currently only .npz file.
        // TODO: make more sensible interface for a CinemaImage. Does it make
        // sense to read in and store every image?? Huge memory consumption!
        // It is probably the best to fill in the point cloud as the images are
        // read and just add a phi/theta ref to each point.
        CinemaImage(
            const std::string   filename,
            const int           phi,
            const int           theta);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud() const;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb() const;

      private:
        std::vector<std::vector<float>> read_depth_image(
            const std::string                   filename) const;

        std::vector<std::vector<float>>   m_depth_image;
        int                               m_phi;
        int                               m_theta;
        // TODO: add simulation values.
    };

    void test_lodepng(std::string filename);
}
#endif