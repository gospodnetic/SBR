/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-18 17:28:39
*/
// Composite raster of .im and .png files from Cinema database into a single
// CinemaImage class.

#include "cinema_image.h"

#include <iostream>
#include <vector>

#include "../submodules/lodepng/lodepng.h"

#include"../submodules/cnpy/cnpy.h"
#include<complex>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>

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
         //load the entire npz file
        cnpy::npz_t my_npz = cnpy::npz_load("../data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz");

        // npz_t is a map of strings
        typedef cnpy::npz_t::const_iterator MapIterator;
        for (MapIterator iter = my_npz.begin(); iter != my_npz.end(); iter++)
        {
            std::cout << "Key: " << iter->first << std::endl;
        }
    }
} // !namespace cinema

