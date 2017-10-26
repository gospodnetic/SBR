
/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-25 09:27:45
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-25 09:27:45
*/

//
// CinemaDB
// Container for multiple CinemaImage objects.
//

#ifndef CINEMA_DB_H
#define CINEMA_DB_H

#include <string>
#include <vector>

#include "cinema_image.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace cinema
{
    class CinemaDB
    {
      public:
        CinemaDB(
            const std::string   db_path,
            const std::string   db_label,
            const int           n_phi_angles = -1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb(
            const int number_of_images = -1) const;

      private:
        void load_cinema_db(
        const std::string   db_path,
        const std::string   db_label,
        const int           number_of_images = -1);

        std::vector<CinemaImage>        m_depth_images; // .npz files from cinema database
        double                          m_camera_near;  // Camera near plane
        double                          m_camera_far;   // Camera far plane
        double                          m_camera_angle; // Field of view
        Eigen::Matrix4d                 m_projection_matrix;
    };
    void test_lodepng(std::string filename);
}
#endif