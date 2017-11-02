
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

#include "Eigen/Dense"

namespace cinema
{
    class CinemaDB
    {
      public:
        CinemaDB(
            const std::string   db_path,
            const std::string   db_label,
            const int           n_phi_angles = -1,
            const int           phi_json_idx = -1,
            const int           theta_json_idx = -1);

        CinemaDB(
            const std::string           depth_filename,
            const size_t                   height,
            const size_t                   width,
            const std::vector<int>      phi,
            const std::vector<int>      theta);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb(
            const int number_of_images = -1) const;

      private:
        void load_cinema_db(
            const std::string   db_path,
            const std::string   db_label,
            const int           number_of_images = -1,
            const int           phi_json_idx = -1,
            const int           theta_json_idx = -1);

        std::vector<CinemaImage>        m_depth_images; // .z files from cinema database
        CameraMetadata                  m_camera_metadata; // Metadata read from info.json file
    };

    void test_lodepng(std::string filename);
}
#endif