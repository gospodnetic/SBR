/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:19:55
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-18 16:27:04
*/
// Composite raster of .im and .png files from Cinema database into a single
// CinemaImage class.

#include "cinema_image.h"

#include <iostream>
#include <vector>

#include <Python.h>
#include </usr/local/lib/python2.7/dist-packages/numpy/core/include/numpy/arrayobject.h>
#include <pyxmod.h>

#include "..submodules/lodepng/lodepng.h"

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

    }
} // !namespace cinema