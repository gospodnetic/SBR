/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-23 11:28:14
*/
// Composite raster of .im and .png files from Cinema database into a single
// CinemaImage class.

#include "cinema_image.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "../submodules/lodepng/lodepng.h"

#include "yaml-cpp/yaml.h"

namespace cinema
{
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

    void test_read_cinema_image()
    {
        //
        // Read .npz file
        //
        const std::string filename = "/home/petra/Desktop/SampleBasedReconstruction/data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz";
        // Call python. The call path assumes we are in build folder at the time
        // execution of the SBR program.
        const std::string system_call = "python ../python_utils/npz2yaml.py -i " + filename;
        std::system(system_call.c_str());

        std::vector<std::vector<float>> npz_array = YAML::LoadFile(filename + ".yaml").as<std::vector<std::vector<float>>>();
        // // Otput the data.
        for (std::vector<std::vector<float>>::iterator row=npz_array.begin(); row!=npz_array.end(); row++) {
            for (std::vector<float>::iterator col=row->begin(); col!=row->end(); col++)
                std::cout << *col << " ";
        }
    }
} // !namespace cinema

