#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "map_server_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_server/map_server.hpp"

YAML::Node doc;
rclcpp::Node::SharedPtr node = nullptr;

inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    return tf2::toMsg(q);
}

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


Parameters loadMapFromFile(const std::string map_url)
{

    //process by request begin
    Parameters parameters;

    // nav_msgs::msg::OccupancyGrid msg;
    doc = YAML::LoadFile(map_url);
    // auto image_file_name = yaml_get_value<std::string>(doc, "image");
    auto image_file_name = doc["image"].as<std::string>();

    //test begin
    parameters.image_file_name = image_file_name;
    parameters.resolution = doc["resolution"].as<double>();
    parameters.origin = doc["origin"].as<std::vector<double>>();
    parameters.free_thresh = doc["free_thresh"].as<double>();
    parameters.occupied_thresh = doc["occupied_thresh"].as<double>();

    auto map_mode_node = doc["mode"].as<std::string>();
    // parameters.mode = map_mode_to_string(map_mode_node);

    try{
        parameters.negate = doc["negate"].as<int>();
    } catch (YAML::Exception &){
        parameters.negate = doc["negate"].as<bool>();
    }
    //test end
    //png load by OpenCV
    cv::Mat img;
    img = cv::imread(parameters.image_file_name);
    cv::MatIterator_<unsigned char> it = img.begin<unsigned char>(); 
    cv::MatIterator_<unsigned char> it_end = img.end<unsigned char>();

    //opencv
    parameters.width = img.cols;
    parameters.height = img.rows;

    for(; it!=it_end; it++)
    {
        double occ = (0 ? (static_cast<float>(*it) / 255.0) : ((255.0 - static_cast<float>(*it)) / 255.0));
        if(parameters.occupied_thresh < occ)
        {
            *it = static_cast<int8_t>(nav_util::OCC_GRID_OCCUPIED);
        }
        else if(parameters.occupied_thresh > occ)
        {
            *it = static_cast<int8_t>(nav_util::OCC_GRID_FREE);
        }   
        else
        {
            *it = static_cast<int8_t>(nav_util::OCC_GRID_UNKNOWN);
        }
        parameters.data.push_back(static_cast<int8_t>(*it));
    }

    return parameters; 
}

void publishmap(const std::shared_ptr<map_server_msgs::srv::LoadMap::Request> request,
                std::shared_ptr<map_server_msgs::srv::LoadMap::Response> response)
{
    Parameters parameters = loadMapFromFile(request->map_url);

    response->map.header.frame_id = "map";
    response->map.header.stamp = node->get_clock()->now();
    response->map.info.map_load_time = node->get_clock()->now();
    response->map.info.resolution = parameters.resolution;
    response->map.info.width = parameters.width;
    response->map.info.height = parameters.height;
    response->map.info.origin.position.x = parameters.origin[0];
    response->map.info.origin.position.y = parameters.origin[1];
    response->map.info.origin.position.z = 0.0;
    response->map.info.origin.orientation = orientationAroundZAxis(parameters.origin[2]);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("map_server");
    rclcpp::Service<map_server_msgs::srv::LoadMap>::SharedPtr service =
        node->create_service<map_server_msgs::srv::LoadMap>("map_loader", &publishmap);

    rclcpp::spin(node);
    rclcpp::shutdown();

}
