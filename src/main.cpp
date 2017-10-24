/*
* @Author: Petra Gospodnetic
* @Date:   2017-09-28 12:56:17
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-24 15:03:27
*/

#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cinema_image.h"
#include "json/json.h"
#include "pcl_utils.h"

int main()
{   
    // Static paths to the image db structure in folders.
    const std::string db_path = "/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image";
    const std::string db_label = "colorSphere1";

    // Load database info.json.
    const std::string filename_json = db_path + "/info.json";
    std::ifstream info_file(filename_json.c_str(), ifstream::binary);
    Json::Value info_json;
    info_file >> info_json;

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

    // Read the depth values for all positions
    std::vector<cinema::CinemaImage> CinemaDB;
    for(std::vector<int>::const_iterator it_phi = phi_values.begin(); it_phi != phi_values.end(); it_phi++)
    {
        const size_t idx_phi = it_phi - phi_values.begin();
        std::string dir_phi = db_path + "/phi=" + std::to_string(idx_phi);
        for(std::vector<int>::const_iterator it_theta = theta_values.begin(); it_theta != theta_values.end(); it_theta++)
        {
            const size_t idx_theta = it_theta - theta_values.begin();
            std::string dir_theta = dir_phi + "/theta=" + std::to_string(idx_theta);
            std::string npz_filename = dir_theta + "/vis=0/" + db_label + "=0.npz";
            CinemaDB.push_back(cinema::CinemaImage(npz_filename, *it_phi, *it_theta));
        }
    }

    std::cout << "Number of read images: " << CinemaDB.size() << std::endl;
    // // Read the image.
    // cinema::CinemaImage cinema_image("/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz",
    //     90,
    //     90);

    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_depth_cloud = 
    //     cinema_image.point_cloud_rgb();

    // //  
    // // Visualize the cloud.
    // //

    // // Initialize the viewer
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    //     new pcl::visualization::PCLVisualizer("3D viewer"));
    // viewer->setBackgroundColor(0, 0, 0);

    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
    //     rgb(image_depth_cloud);
    // viewer->addPointCloud<pcl::PointXYZRGB>(image_depth_cloud, rgb, "sample cloud");
    // viewer->setPointCloudRenderingProperties(
    //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //     3,
    //     "sample cloud");


    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // // Run the main loop.
    // while(!viewer->wasStopped())
    // {
    //     viewer->spinOnce(1);
    //     boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    // }
    return 0;
}   