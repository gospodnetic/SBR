/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-11-02 17:10:12
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

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4, glm::ivec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

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
    , m_test_PCL_depth(false)
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

    // 
    // TODO: Switch this with an empty constructor and test_depth function. The
    //      boolean here is confusing.
    // 
    /*! \brief Constructor used for testing the PCL visualization using manually,
    *       generated depth image.
    */
    CinemaImage::CinemaImage(
            const std::string       filename,
            const int               phi,
            const int               theta,
            const CameraMetadata    camera_metadata,
            const bool              test_image)
    : m_phi(phi)
    , m_theta(theta)
    , m_camera_metadata(camera_metadata)
    , m_test_PCL_depth(true)
    {
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

        m_near_far_step = 0;
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
        // Use adapted function for PCL testing.
        if(m_test_PCL_depth)
            return point_cloud_rgb_test();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud->points.resize(m_depth_image.size() * m_depth_image[0].size());
        
        // Generate the point cloud out of the depth values.
        const double width_half = m_camera_metadata.image_width / 2.0f;
        const double height_half = m_camera_metadata.image_height / 2.0f;

        // Rotation transformation with phi rotating around y-axis and theta
        // rotating around x-axis.
        // Modelview matrix is in our case only rotation matrix.

        glm::mat4 rot_phi(
            cos(m_phi_rad), 0, sin(m_phi_rad), 0,
                         0, 1,              0, 0,
           -sin(m_phi_rad), 0, cos(m_phi_rad), 0,
                         0, 0,              0, 1);

        glm::mat4 rot_theta(
              1,                0,                0, 0,
              0, cos(m_theta_rad), sin(m_theta_rad), 0,
              0,-sin(m_theta_rad), cos(m_theta_rad), 0,
              0,                0,                0, 1);

        glm::mat4 projection_matrix(
            m_camera_metadata.projection_matrix(0, 0), m_camera_metadata.projection_matrix(0, 1), m_camera_metadata.projection_matrix(0, 2), m_camera_metadata.projection_matrix(0, 3),
            m_camera_metadata.projection_matrix(1, 0), m_camera_metadata.projection_matrix(1, 1), m_camera_metadata.projection_matrix(1, 2), m_camera_metadata.projection_matrix(1, 3),
            m_camera_metadata.projection_matrix(2, 0), m_camera_metadata.projection_matrix(2, 1), m_camera_metadata.projection_matrix(2, 2), m_camera_metadata.projection_matrix(2, 3),
            m_camera_metadata.projection_matrix(3, 0), m_camera_metadata.projection_matrix(3, 1), m_camera_metadata.projection_matrix(3, 2), m_camera_metadata.projection_matrix(3, 3));
        glm::mat4 rotation_matrix = rot_phi * rot_theta;

        // glm::mat4 projection_matrix = glm::ortho(
        //     -width_half * m_near_far_step,          // Left plane
        //     (width_half - 1) * m_near_far_step,     // Right plane
        //     -height_half * m_near_far_step,         // Bottom plane
        //     (height_half - 1) * m_near_far_step,     // Top plane
        //     m_camera_metadata.camera_near,          // Near plane
        //     m_camera_metadata.camera_far);          // Far plane
        glm::vec4 viewport(
            0,
            0,
            m_camera_metadata.image_width * m_near_far_step,
            m_camera_metadata.image_height * m_near_far_step);

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

                double depth = (*col) * m_near_far_step;

                // x y z vector.
                // Scaling pixel values to camera space units.
                // Translating depth for the value of the far plane to invert
                // positions so that point cloud origin corresponds with object
                // origin.

                // It seems that depth is not linear!!
                // OpenGL non linear depth 
                // https://stackoverflow.com/questions/8990735/how-to-use-opengl-orthographic-projection-with-the-depth-buffer

                // double zNear = m_camera_metadata.camera_near;
                // double zFar = m_camera_metadata.camera_far;
                // double z_n = 

                // depth = depth * (m_camera_metadata.camera_far + m_camera_metadata.camera_near - z_n * (m_camera_metadata.camera_far - m_camera_metadata.camera_near));
                glm::vec4 pos(
                    (col - row->begin() - width_half) * m_near_far_step,
                    (row - m_depth_image.begin() - height_half) * m_near_far_step,
                    (depth) - m_camera_metadata.camera_far,
                    1);

                // pos = glm::unProject(
                //     pos,
                //     glm::mat4(1.0f),
                //     glm::mat4(1.0f),
                //     viewport);.

                pos = rotation_matrix * pos;

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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CinemaImage::point_cloud_rgb_test() const
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud->points.resize(m_depth_image.size() * m_depth_image[0].size());
        
        // Generate the point cloud out of the depth values.
        const double width_half = (m_camera_metadata.image_width - 1) / 2.0f;
        const double height_half = (m_camera_metadata.image_height - 1) / 2.0f;

        // Rotation transformation with phi rotating around y-axis and theta
        // rotating around x-axis.
        glm::mat4 rot_phi(
            cos(m_phi_rad), 0, sin(m_phi_rad), 0,
                         0, 1,              0, 0,
           -sin(m_phi_rad), 0, cos(m_phi_rad), 0,
                         0, 0,              0, 1);

        glm::mat4 rot_theta(
              1,                0,                0, 0,
              0, cos(m_theta_rad), sin(m_theta_rad), 0,
              0,-sin(m_theta_rad), cos(m_theta_rad), 0,
              0,                0,                0, 1);
    
        glm::mat4 rotation_matrix = rot_phi * rot_theta;

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

                // x y z vector.
                glm::vec4 pos(
                    col - row->begin(),
                    row - m_depth_image.begin(),
                    *col,
                    1);

                glm::vec4 translate(
                    width_half,
                    height_half,
                    128, // Half depth
                    0);
                pos = pos - translate;

                pos = rotation_matrix * pos;

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

