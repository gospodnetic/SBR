/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-25 09:27:45
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-11-02 14:09:56
*/

#include "cinema_db.h"

#include <cmath>

#include "json/json.h"

namespace cinema
{
    CinemaDB::CinemaDB(
        const std::string   db_path,
        const std::string   db_label,
        const int           n_images,
        const int           phi_json_idx,
        const int           theta_json_idx) // Number of images to be read.
    {
        // Load depth images and read info.json values.
        load_cinema_db(
            db_path,
            db_label,
            n_images,
            phi_json_idx,
            theta_json_idx);
    }

    /*! \brief Constructor used for testing the PCL visualization using manually
    *       generated depth image.
    */
    CinemaDB::CinemaDB(
        const std::string           depth_filename,
        const size_t                height,
        const size_t                width,
        const std::vector<int>      phi,
        const std::vector<int>      theta)
    {
        CameraMetadata fake_camera_metadata; 
        fake_camera_metadata.image_height = height;
        fake_camera_metadata.image_width = width;

        typedef std::vector<int>::const_iterator veci_cit;
        for(veci_cit phi_it = phi.begin(); phi_it != phi.end(); phi_it++)
        {
            for(veci_cit theta_it = theta.begin(); theta_it != theta.end(); theta_it++)
            {
                m_depth_images.push_back(CinemaImage(
                    depth_filename,
                    *phi_it,
                    *theta_it,
                    fake_camera_metadata,
                    true));
            }
        }
    }

    void CinemaDB::load_cinema_db(
        const std::string   db_path,
        const std::string   db_label,
        const int           n_images,
        const int           phi_json_idx,
        const int           theta_json_idx)
    {
        // Load database info.json.
        const std::string filename_json = db_path + "/info.json";
        std::ifstream info_file(filename_json.c_str(), std::ifstream::binary);
        Json::Value info_json;
        info_file >> info_json;

        // Rad camera values.
        // NOTE: [0] index before indexing camera values is a hack due to only
        //       one timestep / camera config.
        // TODO: Use iterator so that it can be saved into vectors for each 
        //       timestep / camera config.
        m_camera_metadata.camera_near = info_json["metadata"]["camera_nearfar"][0][0].asDouble();
        m_camera_metadata.camera_far = info_json["metadata"]["camera_nearfar"][0][1].asDouble();
        m_camera_metadata.camera_angle = info_json["metadata"]["camera_angle"][0].asDouble();

        m_camera_metadata.camera_up << 
            info_json["metadata"]["camera_up"][0][0].asDouble(),
            info_json["metadata"]["camera_up"][0][1].asDouble(),
            info_json["metadata"]["camera_up"][0][2].asDouble();
            
        m_camera_metadata.camera_eye << 
            info_json["metadata"]["camera_eye"][0][0].asDouble(),
            info_json["metadata"]["camera_eye"][0][1].asDouble(),
            info_json["metadata"]["camera_eye"][0][2].asDouble();

        m_camera_metadata.camera_at << 
            info_json["metadata"]["camera_at"][0][0].asDouble(),
            info_json["metadata"]["camera_at"][0][1].asDouble(),
            info_json["metadata"]["camera_at"][0][2].asDouble();

        m_camera_metadata.image_height = info_json["metadata"]["image_size"][0].asDouble();
        m_camera_metadata.image_width = info_json["metadata"]["image_size"][1].asDouble();

        // Restrict camera near far interval.
        // const double max_far = 10.0f;
        // const double clipping_plane_scaling_factor = max_far / m_camera_metadata.camera_far;
        // m_camera_metadata.camera_far = m_camera_metadata.camera_far * clipping_plane_scaling_factor;
        // m_camera_metadata.camera_near = m_camera_metadata.camera_near * clipping_plane_scaling_factor;

        // Calculate perspective projection matrix.
        const double S = 1 / tan((m_camera_metadata.camera_angle) * (M_PI / 180));
        
        // Projection matrix from http://www.terathon.com/gdc07_lengyel.pdf
        // but with aspect ratio 1 (projection_matrix[1][1] = S)
        const double b = -(m_camera_metadata.camera_far + m_camera_metadata.camera_near) / (m_camera_metadata.camera_far - m_camera_metadata.camera_near);
        const double c = -2 * m_camera_metadata.camera_far * m_camera_metadata.camera_near / (m_camera_metadata.camera_far - m_camera_metadata.camera_near);
        m_camera_metadata.projection_matrix <<
            S,  0,  0,  0,
            0,  S,  0,  0,
            0,  0,  b,  c,
            0,  0, -1,  0;

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
        if(phi_json_idx >= 0 || theta_json_idx >=0)
        {
            if(phi_json_idx >= 0 && theta_json_idx >=0)
            {
                std::string dir_phi = db_path + "/phi=" + std::to_string(phi_json_idx);
                std::string dir_theta = dir_phi + "/theta=" + std::to_string(theta_json_idx);
                std::string z_filename = dir_theta + "/vis=0/" + db_label + "=0.Z";
        
                cinema_db.push_back(CinemaImage(
                    z_filename,
                    phi_values[phi_json_idx],
                    theta_values[theta_json_idx],
                    m_camera_metadata));

                std::cout << "Read theta " << theta_json_idx << " phi " << phi_json_idx << std::endl;
            }
        }
        else if(n_images < 0)
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
                    std::string z_filename = dir_theta + "/vis=0/" + db_label + "=3.Z";
                    
                    cinema_db.push_back(CinemaImage(
                        z_filename,
                        *it_phi,
                        *it_theta,
                        m_camera_metadata));

                    std::cout << "Read theta" << *it_theta << " phi " << *it_phi << std::endl;                
                }
            }
        }
        else
        {
            // Read the depth values for n images.
            std::cout << "Reading " << n_images << " images..." << std::endl;
            size_t image_count = 0;
            for(std::vector<int>::const_iterator it_phi = phi_values.begin(); it_phi != phi_values.end(); it_phi++)
            {
                const size_t idx_phi = it_phi - phi_values.begin();
                std::string dir_phi = db_path + "/phi=" + std::to_string(idx_phi);
                for(std::vector<int>::const_iterator it_theta = theta_values.begin(); it_theta != theta_values.end(); it_theta++)
                {
                    const size_t idx_theta = it_theta - theta_values.begin();
                    std::string dir_theta = dir_phi + "/theta=" + std::to_string(idx_theta);
                    std::string z_filename = dir_theta + "/vis=0/" + db_label + "=0.Z";
                    
                    cinema_db.push_back(CinemaImage(
                        z_filename,
                        *it_phi,
                        *it_theta,
                        m_camera_metadata));                
                    
                    std::cout << "Read theta" << *it_theta << " phi " << *it_phi << std::endl;                
                    if(++image_count >= n_images)
                       break;
                }

                if(image_count >= n_images)
                    break;
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
                *image_depth_cloud += *(it->point_cloud_rgb());
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
                *image_depth_cloud += *(it->point_cloud_rgb());
                std::cout << count++ << std::endl;                
            }
        }

        return image_depth_cloud;
    }
    
} // !namespace cinema