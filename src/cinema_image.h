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
            const std::string           filename) const;

        std::vector<std::vector<float>> m_depth_image;
        double                          m_phi;
        double                          m_theta;
        double                          m_phi_rad;
        double                          m_theta_rad;
        double                          m_camera_near;
        double                          m_camera_far;
        double                          m_near_far_step;

        double                          m_far_plane; // Should be 255 
                                            // but it has happened to be 256.
                                            // Therefore, we have a var storing
                                            // the max depth value as the
                                            // representation of the far plane.
        // TODO: add simulation values.
    };

    std::vector<CinemaImage> load_cinema_db(
        const std::string db_path,
        const std::string db_label);

    void test_lodepng(std::string filename);
}
#endif