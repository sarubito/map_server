#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "map_server_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <chrono>
#include <vector>
#include <memory>
#include <string>

YAML::Node doc;

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
};

const char * map_mode_to_string(MapMode map_mode)
{
    switch (map_mode)
    {
    case MapMode::Trinary:
        return "trinary";
    case MapMode::Scale:
        return "scale";
    case MapMode::Raw:
        return "raw";
    default:
        throw std::invalid_argument("map_mode");
    }
}


void loadMapFromFile(const std::shared_ptr<map_server_msgs::srv::LoadMap::Request> request,
                     std::shared_ptr<map_server_msgs::srv::LoadMap::Response> response)
{

    //process by request begin
    Parameters parameters;
    nav_msgs::msg::OccupancyGrid msg;
    doc = YAML::LoadFile(request->map_url);
    // auto image_file_name = yaml_get_value<std::string>(doc, "image");
    auto image_file_name = doc["image"].as<std::string>();
    if(image_file_name.empty())
    {
        throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
    }
    if (image_file_name[0] != '/') 
    {
        // dirname takes a mutable char *, so we copy into a vector
        std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
        fname_copy.push_back('\0');
        image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
    }

    //load parameters
    // parameters.image_file_name = image_file_name;
    // parameters.resolution = yaml_get_value<double>(doc, "resolution");
    // parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
    // parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
    // parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

    // auto map_mode_node = doc["mode"];

    // load_parameters.negate = yaml_get_value<int>(doc, "negate");

    //test begin
    parameters.image_file_name = image_file_name;
    parameters.resolution = doc["resolution"].as<double>();
    parameters.origin = doc["origin"].as<std::vector<double>>();
    parameters.free_thresh = doc["free_thresh"].as<std::vector>();
    parameters.occupied_thresh = doc["occupied_thresh"].as<double>();

    auto map_mode_node = doc["mode"].as<std::string>();
    parameters.mode = map_mode_to_string(map_mode_node);

    try{
        load_parameters.negate = doc["negate"].as<int>();
    } catch (YAML::Exception &){
        load_parameters.negate = doc["negate"].as<bool>();
    }
    //test end

    //process by request end

}