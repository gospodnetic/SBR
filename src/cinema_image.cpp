/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-11-01 15:04:55
*/
// Composite raster of .im and .png files from Cinema database into a single
// CinemaImage class.

#include "cinema_image.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "../submodules/lodepng/lodepng.h"

#include "yaml-cpp/yaml.h"

namespace cinema
{
    CinemaImage::CinemaImage(
        const std::string       filename,
        const int               phi,
        const int               theta,
        const CameraMetadata    camera_metadata)
    : m_phi(phi)
    , m_theta(theta)
    , m_camera_metadata(camera_metadata)
    {
        // Takes about a second to read the depth image.
        // TODO: pass a pointer or o it directly in cpp, instead of using a 
        //       swap file. It imposes a big performance hit!        
        m_depth_image = read_depth_image(filename);

        m_phi_rad = m_phi * M_PI / 180;
        m_theta_rad = m_theta * M_PI / 180;

        // Get max element of a 2D vector.
        std::vector<float> column_maxs;
        for(std::vector<std::vector<float>>::const_iterator row=m_depth_image.begin(); row!=m_depth_image.end(); row++)
        {
            column_maxs.push_back(*std::max_element(row->begin(), row->end()));
        }
        const float max_depth = *std::max_element(std::begin(column_maxs), std::end(column_maxs));
        m_far_plane = max_depth;

        // Camera unit declared based on the near_far distance.
        m_near_far_step = (m_camera_metadata.camera_far - m_camera_metadata.camera_near) / max_depth;
    }

    /*! \brief Return point cloud created out of the cinema image.
    *   
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr CinemaImage::point_cloud() const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        point_cloud->points.resize(m_depth_image.size() * m_depth_image[0].size());

        // Generate the point cloud out of the depth values.
        size_t idx = 0;
        for(std::vector<std::vector<float>>::const_iterator row=m_depth_image.begin(); row!=m_depth_image.end(); row++)
        {
            for(std::vector<float>::const_iterator col=row->begin(); col!=row->end(); col++)
            {
                point_cloud->points[idx].z = col - row->begin();
                point_cloud->points[idx].y = row - m_depth_image.begin();
                point_cloud->points[idx].x = *col; // Switched with z just so that I don't have to rotate it every time when vieweing
                idx++;
            }
        }

        return point_cloud;
    }

    /*! \brief Return colored point cloud created out of the cinema image.
    *   
    *   TODO: Add simulation values - currenly it returns only depth with
    *   arbitrary color.
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CinemaImage::point_cloud_rgb() const
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud->points.resize(m_depth_image.size() * m_depth_image[0].size());
        
        // Rotation transformation with phi rotating around y-axis and theta
        // rotating around x-axis.
        Eigen::Matrix4d rot_theta;
        rot_theta <<  cos(m_theta_rad), 0, sin(m_theta_rad), 0,
                      0               , 1               , 0, 0,
                     -sin(m_theta_rad), 0, cos(m_theta_rad), 0,
                      0               , 0               , 0, 1;

        Eigen::Matrix4d rot_phi;
        rot_phi <<  1,              0,              0, 0,
                    0, cos(m_phi_rad), sin(m_phi_rad), 0,
                    0,-sin(m_phi_rad), cos(m_phi_rad), 0,
                    0,              0,              0, 1;

        Eigen::Matrix4d rot_matrix = rot_phi * rot_theta;

        // Generate the point cloud out of the depth values.
        const int width_half = m_camera_metadata.image_width / 2;
        const int height_half = m_camera_metadata.image_height / 2;
        const float depth_shift = m_camera_metadata.camera_near + m_near_far_step * m_far_plane;
        size_t idx = 0;
        for(std::vector<std::vector<float>>::const_iterator row = m_depth_image.begin(); row != m_depth_image.end(); row++)
        {
            for(std::vector<float>::const_iterator col = row->begin(); col != row->end(); col++)
            {
                // Do not use points which represent the far plane.
                // TODO: This is float to double comparison - FIX POTENTIAL ERROR!
                // TODO: What Can I resize the point cloud initially to fit the 
                //       number of points which are actually used? Is there a
                //       way to resize it again afterwards?
                if(*col == m_far_plane)
                    continue;

                const float depth = (*col) * m_near_far_step;

                // x y z vector.
                // Scaling pixel values to camera space units.
                // Translating depth for the value of the far plane to invert
                // positions so that point cloud origin corresponds with object
                // origin.
                
                Eigen::Vector4d pos(
                    (col - row->begin() - width_half) * m_near_far_step,
                    (row - m_depth_image.begin() - height_half) * m_near_far_step,
                    depth - m_camera_metadata.camera_far,
                    1);
                
                pos = rot_matrix * m_camera_metadata.projection_matrix * pos;
                

                point_cloud->points[idx].x = pos[0];
                point_cloud->points[idx].y = pos[1];
                point_cloud->points[idx].z = pos[2];
                point_cloud->points[idx].r = *col;
                point_cloud->points[idx].g = 50;
                point_cloud->points[idx].b = 50;
                idx++;
            }
        }

        return point_cloud;
    }

    /*! \brief Read the .Z file containing depth information.
    */
    std::vector<std::vector<float>> CinemaImage::read_depth_image(
        const std::string                   filename) const
    {
        //
        // Read .Z
        //

        struct stat buffer;
        std::vector<std::vector<float>> depth_array;   
        const bool use_cached_yaml = true;
        if(!use_cached_yaml || stat ((filename + ".yaml").c_str(), &buffer) != 0)
        {
            // If YAML file dose not exist, create it using python.
            // Call python. The call path assumes we are in build folder at the time
            // execution of the SBR program.
            std::cout << "No YAML file..." << std::endl;
            const std::string system_call = 
                "python ../python_utils/z2yaml.py -i " + filename + 
                " -w " + std::to_string(m_camera_metadata.image_width) +
                " -h " + std::to_string(m_camera_metadata.image_height);
            std::cout << system_call << std::endl;
            std::system(system_call.c_str());
        } 
            
        depth_array = YAML::LoadFile(filename + ".yaml").as<std::vector<std::vector<float>>>();

        return depth_array;
    }

    void test_lodepng(std::string filename)
    {
        std::vector<unsigned char> image; //the raw pixels
        unsigned width, height;

        //decode
        unsigned error = lodepng::decode(image, width, height, filename.c_str());

        std::cout << "zzz: PNG image size: " << width << " x " << height << std::endl;
        //if there's an error, display it
        if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    }
} // !namespace cinema

