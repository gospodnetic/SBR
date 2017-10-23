/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-23 14:47:02
*/
// Composite raster of .im and .png files from Cinema database into a single
// CinemaImage class.

#include "cinema_image.h"

#include <cstdlib>
#include <iostream>
#include <string>

#include "../submodules/lodepng/lodepng.h"

#include "yaml-cpp/yaml.h"

namespace cinema
{
    CinemaImage::CinemaImage(
        const std::string   filename,
        const int           phi,
        const int           theta)
    :m_phi(phi)
    ,m_theta(theta)
    {
        // Takes about a second to read the depth image.
        // TODO: pass a pointer Instead of using swap file.
        m_depth_image = read_depth_image(filename);

        // TODO: figure out if the depth values should be mapped into camera
        // space or not.

        //
        // Get max element of a 2D vector.
        //
        // std::vector<float> column_maxs;
        // for(std::vector<std::vector<float>>::const_iterator row=depth_image.begin(); row!=depth_image.end(); row++)
        // {
        //     column_maxs.push_back(*std::max_element(row->begin(), row->end()));
        // }
        // const float max_depth = *std::max_element(std::begin(column_maxs), std::end(column_maxs));

        // // Camera near far out of info.json.
        // const double camera_near = 2.305517831184482;
        // const double camera_far = 4.6363642410628785;
        // const double near_far_step = (camera_far - camera_near) / max_depth;
        
        // Map depth values to camera near/far space.
        // Currently not being mapped because of the scale in pcl cloud.
        // TODO: Remove far plane from the point cloud
        // TODO: Rotate the depth image according to the theta and phi.
        // for(std::vector<std::vector<float>>::iterator row=depth_image.begin(); row!=depth_image.end(); row++)
        // {
        //     for(std::vector<float>::iterator col=row->begin(); col!=row->end(); col++)
        //         *col = *col / max_depth;
        // }
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

        // Generate the point cloud out of the depth values.
        size_t idx = 0;
        for(std::vector<std::vector<float>>::const_iterator row=m_depth_image.begin(); row!=m_depth_image.end(); row++)
        {
            for(std::vector<float>::const_iterator col=row->begin(); col!=row->end(); col++)
            {
                point_cloud->points[idx].z = col - row->begin();
                point_cloud->points[idx].y = row - m_depth_image.begin();
                point_cloud->points[idx].x = *col; // Switched with z just so that I don't have to rotate it every time when vieweing
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
        // Call python. The call path assumes we are in build folder at the time
        // execution of the SBR program.
        const std::string system_call = "python ../python_utils/npz2yaml.py -i " + filename;
        std::system(system_call.c_str());

        std::vector<std::vector<float>> depth_array = YAML::LoadFile(filename + ".yaml").as<std::vector<std::vector<float>>>();

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

