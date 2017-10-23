/*
* @Author: Petra Gospodnetic
* @Date:   2017-10-17 16:32:07
* @Last Modified by:   Petra Gospodnetic
* @Last Modified time: 2017-10-17 16:32:07
*/

#ifndef CINEMA_IMAGE_P
#define CINEMA_IMAGE_P

#include <string>
#include <vector>

namespace cinema
{
    void test_lodepng(std::string filename);

    std::vector<std::vector<float>> read_depth_image();
}
#endif