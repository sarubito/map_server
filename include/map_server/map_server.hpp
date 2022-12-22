#ifndef MAP_SERVER_HPP_
#define MAP_SERVER_HPP_

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

enum class MapMode
{
    Trinary,
    Scale,
    Raw
};

struct Parameters
{
    std::string image_file_name;
    MapMode mode;
    double resolution;
    std::vector<double> origin{0.0, 0.0, 0.0};
    bool negate;
    double occupied_thresh;
    double free_thresh;
    u_int32_t width;
    u_int32_t height;
    std::vector<int8_t> data;
};

enum class nav_util : int8_t
{
    OCC_GRID_UNKNOWN = -1,
    OCC_GRID_FREE = 0,
    OCC_GRID_OCCUPIED = 100
};

#endif