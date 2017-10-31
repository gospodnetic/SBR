/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-25 09:27:45
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-30 13:38:28
*/

#include "cinema_db.h"

#include <cmath>

#include "json/json.h"

namespace cinema
{
    CinemaDB::CinemaDB(
        const std::string   db_path,
        const std::string   db_label,
        const int           n_phi_angles) // Number of phi angles.
    {
        // Load depth images and read info.json values.
        load_cinema_db(db_path, db_label, n_phi_angles);

        // Calculate perspective projection matrix.
        const double S = 1 / tan((m_camera_angle) * (M_PI / 180));
        
        // // Projection matrix from https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/building-basic-perspective-projection-matrix
        // m_projection_matrix <<
        // const double a = -m_camera_far / (m_camera_far - m_camera_near);
        // const double b = -m_camera_far * m_camera_near / (m_camera_far - m_camera_near);
        //     S,  0,  0,  0,
        //     0,  S,  0,  0,
        //     0,  0,  a, -1,
        //     0,  0,  b,  0;
        // Projection matrix from http://www.terathon.com/gdc07_lengyel.pdf
        // but with aspect ratio 1 (m_projection_matrix[1][1] = S)
        const double b = -(m_camera_far + m_camera_near) / (m_camera_far - m_camera_near);
        const double c = -2 * m_camera_far * m_camera_near / (m_camera_far - m_camera_near);
        m_projection_matrix <<
            S,  0,  0,  0,
            0,  S,  0,  0,
            0,  0,  b,  c,
            0,  0, -1,  0;
    }

    void CinemaDB::load_cinema_db(
        const std::string   db_path,
        const std::string   db_label,
        const int           n_phi_angles)
    {
        // Load database info.json.
        const std::string filename_json = db_path + "/info.json";
        std::ifstream info_file(filename_json.c_str(), std::ifstream::binary);
        Json::Value info_json;
        info_file >> info_json;

        // Rad camera values.
        // NOTE: [0] index before indexing is a hack due to bad .json generator.
        //       Camera near far values are generated as array within an array
        //       for some reason.
        m_camera_near = info_json["metadata"]["camera_nearfar"][0][0].asDouble();
        m_camera_far = info_json["metadata"]["camera_nearfar"][0][1].asDouble();
        m_camera_angle = info_json["metadata"]["camera_angle"][0].asDouble();

        // Read phi values
        std::vector<int> phi_values;
        Json::Value phi = info_json["parameter_list"]["phi"]["values"];
        for(Json::ValueIterator it = phi.begin();  it != phi.end(); it++)
            phi_values.push_back(it->asInt());

        // Read theta values
        std::vector<int> theta_values;
        Json::Value theta = info_json["parameter_list"]["theta"]["values"];
        for(Json::ValueIterator it = theta.begin(); it != theta.end(); it++)
            theta_values.push_back(it->asInt());

        std::vector<CinemaImage> cinema_db;
        int count = 0;
        if(n_phi_angles < 0)
        {
            std::cout << "Reading all the images..." << std::endl;
            // Read the depth values for all positions
            for(std::vector<int>::const_iterator it_phi = phi_values.begin(); it_phi != phi_values.end(); it_phi++)
            {
                const size_t idx_phi = it_phi - phi_values.begin();
                std::string dir_phi = db_path + "/phi=" + std::to_string(idx_phi);
                for(std::vector<int>::const_iterator it_theta = theta_values.begin(); it_theta != theta_values.end(); it_theta++)
                {
                    const size_t idx_theta = it_theta - theta_values.begin();
                    std::string dir_theta = dir_phi + "/theta=" + std::to_string(idx_theta);
                    std::string npz_filename = dir_theta + "/vis=0/" + db_label + "=0.npz";
                    cinema_db.push_back(CinemaImage(npz_filename, *it_phi, *it_theta));
                    std::cout << "Read " << count++ << std::endl;                
                    std::cout << "Theta " << *it_theta << std::endl;                
                    std::cout << "Phi " << *it_phi << std::endl;
                }
            }
        }
        else
        {
            // Read the depth values for n phi angles.
            for(std::vector<int>::const_iterator it_phi = phi_values.begin(); (it_phi - phi_values.begin()) < n_phi_angles && (it_phi != phi_values.end()); it_phi++)
            {
                const size_t idx_phi = it_phi - phi_values.begin();
                std::string dir_phi = db_path + "/phi=" + std::to_string(idx_phi);
                for(std::vector<int>::const_iterator it_theta = theta_values.begin(); it_theta != theta_values.end(); it_theta++)
                {
                    const size_t idx_theta = it_theta - theta_values.begin();
                    std::string dir_theta = dir_phi + "/theta=" + std::to_string(idx_theta);
                    std::string npz_filename = dir_theta + "/vis=0/" + db_label + "=0.npz";
                    cinema_db.push_back(CinemaImage(npz_filename, *it_phi, *it_theta));                
                }
            }
        }

        m_depth_images = cinema_db;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CinemaDB::point_cloud_rgb(
        const int number_of_images) const
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_depth_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        
        if(number_of_images < 0)
        {
            // Load all the depth images into the point cloud.
            for(std::vector<CinemaImage>::const_iterator it = m_depth_images.begin(); it != m_depth_images.end(); it++)
                *image_depth_cloud += *(it->point_cloud_rgb(m_projection_matrix));
        }
        else
        {
            // Load given number of images from the beginning.
            int count = 0;
            for(
                std::vector<CinemaImage>::const_iterator it = m_depth_images.begin();
                ((it - m_depth_images.begin()) < number_of_images) && it != m_depth_images.end();
                it++)
            {
                *image_depth_cloud += *(it->point_cloud_rgb(m_projection_matrix));
                std::cout << count++ << std::endl;                
            }
        }

        return image_depth_cloud;
    }
    
} // !namespace cinema