/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-31 10:48:04
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

#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

namespace cinema
{
    CinemaImage::CinemaImage(
        const std::string   filename,
        const int           phi,
        const int           theta)
    : m_phi(phi)
    , m_theta(theta)
    {
        // Takes about a second to read the depth image.
        // TODO: pass a pointer Instead of using swap file.
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

        // TODO: figure out if the depth values should be mapped into camera
        // space or not.

        // Camera near far out of info.json.
        // TODO: Unnecessary duplication - move it somewhere more appropriate.
        // m_camera_near = 2.305517831184482;
        // m_camera_far = 4.6363642410628785;
        m_camera_near = 0.007670275366679059;
        m_camera_far = 7.670275366679059;
        m_near_far_step = (m_camera_far - m_camera_near) / max_depth;
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CinemaImage::point_cloud_rgb(
        Eigen::Matrix4d projection_matrix) const
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

        std::cout << "m_far_plane: " << m_far_plane << std::endl;
        // Generate the point cloud out of the depth values.
        const int width_half = 2200 / 2;
        const int height_half = 1223 / 2;
        const float depth_shift = m_camera_near + m_near_far_step * m_far_plane;
        size_t idx = 0;
        for(std::vector<std::vector<float>>::const_iterator row=m_depth_image.begin(); row!=m_depth_image.end(); row++)
        {
            for(std::vector<float>::const_iterator col=row->begin(); col!=row->end(); col++)
            {
                // Do not use points which represent the far plane.
                // TODO: This is float to double comparison - FIX POTENTIAL ERROR!
                // TODO: What Can I resize the point cloud initially to fit the 
                //       number of points which are actually used? Is there a
                //       way to resize it again afterwards?
                if(*col == m_far_plane)
                    continue;

                const float depth = m_near_far_step * (*col);
                std::cout << "depth: " << depth << std::endl;

                // x y z vector.
                // Scaling pixel values to camera space units.
                // Translating depth for the value of the far plane to invert
                // positions so that point cloud origin corresponds with object
                // origin.
                Eigen::Vector4d pos(
                    (col - row->begin() - width_half) * m_near_far_step,
                    (row - m_depth_image.begin() - height_half) * m_near_far_step,
                    depth,
                    1);
                
                pos = rot_matrix * projection_matrix * pos;

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

    /*! \brief Read the .npz file containing depth information.
    */
    std::vector<std::vector<float>> CinemaImage::read_depth_image(
        const std::string                   filename) const
    {
        //
        // Read .npz file
        //

        struct stat buffer;
        std::vector<std::vector<float>> depth_array;   

        if(stat ((filename + ".yaml").c_str(), &buffer) != 0)
        {
            // If YAML file dose not exist, create it using python.
            // Call python. The call path assumes we are in build folder at the time
            // execution of the SBR program.
            std::cout << "No YAML file..." << std::endl;
            const std::string system_call = "python ../python_utils/npz2yaml.py -i " + filename;
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

