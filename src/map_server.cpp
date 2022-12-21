#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "map_server_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <chrono>
#include <vector>
#include <memory>
#include <string>

YAML::Node doc;
rclcpp::Node::SharedPtr node = nullptr;

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
    uint32 width;
    uint32 height;
    std::vector<int8> data;
};

enum class nav_util
{
    static constexpr int8_t OCC_GRID_UNKNOWN = -1
    static constexpr int8_t OCC_GRID_FREE = 0,
    static constexpr int8_t OCC_GRID_OCCUPIED = 100

};

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


Parameters loadMapFromFile(const string map_url)
{

    //process by request begin
    Parameters parameters;

    // nav_msgs::msg::OccupancyGrid msg;
    doc = YAML::LoadFile(map_url);
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
    //png load by Magick
    Magick::Image img(parameters.image_file_name);
    //png load by OpenCV
    // cv::Mat img;
    // img.read(parameters.image_file_name);


    // msg.info.width = img.size().width();
    // msg.info.height = img.size().height();
    // msg.info.resolution = parameters.resolution;
    // msg.info.origin.position.x = parameters.origin[0];
    // msg.info.origin.position.y = parameters.origin[1];
    // msg.info.origin.position.z = 0.0;
    // msg.info.origin.orientation = orientationAroundZAxis(parameters.origin)

    parameters.width = img.size().width();
    parameters.height = img.size().height();

    //opencv
    //parameters.width = img.cols;
    //parameters.height = img.rows;


    // msg.data.resize(msg.info.width * msg.info.height);

    for(size_t y=0; y<parameters.height; y++)
    {
        for(size_t x=0; X<parameters.width; x++)
        {
            auto pixel = img.pixelColor(x,y);//load 1pixel gotoni yomikomu
            std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(), pixel.blueQuantum()};
            if(parameters.mode == MapMode::Trinary && img.matte())
            {
                channels.push_back(MaxRGB - pixel.alphaQuantum());
            }
            double sum = 0;
            for (auto c : channels) {
                sum += c;
            }
            double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());
            double occ = (load_parameters.negate ? shade : 1.0 - shade);

            int8_t map_cell;
            switch(parameters.mode)
            {
                case MapMode::Trinary:
                    if(parameters.occupied_thresh < occ)
                    {
                        map_cell = nav2_util::OCC_GRID_OCCUPIED;
                    }
                    else if(parameters.occupied_thresh > occ)
                    {
                        map_cell = nav2_util::OCC_GRID_FREE;
                    }
                    else
                    {
                        map_cell = nav2_util::OCC_GRID_UNKNOWN;
                    }
                    break;
                case MapMode::Scale:
                    if(pixel.alphaQuantum() != OpaqueOpacity)
                    {
                        map_cell = nav2_util::OCC_GRID_UNKNOWN;

                    }
                    else if(parameters.occupied_thresh < occ)
                    {
                        map_cell = nav2_util::OCC_GRID_OCCUPIED;
                    }
                    else if(parameters.free_thresh > occ)
                    {
                        map_cell = nav2_util::OCC_GRID_FREE;
                    }
                    else
                    {
                        map_cell = std::rint((occ-parameters.free_thresh) / (parameters.occupied_thresh - parameters.free_thresh) * 100.0);    
                    }
                    break;
                case MapMode::Raw:
                    double occ_percent = std::round(shade * 255);
                    if(nav2_util::OCC_GRID_FREE <= occ_percent && occ_percent <= nav2_util::OCC_GRID_OCCUPIED)
                    {
                        map_cell = static_cast<int8_t>(occ_percent);
                    }
                    else
                    {
                        map_cell = nav2_util::OCC_GRID_UNKNOWN;
                    }
                    break;
                default:
                    throw std::runtime_error("Invalid map mode");
            }
            parameters.data.push_back(map_cell)
            // msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
        }
    }
}

void publishmap(const std::shared_ptr<map_server_msgs::srv::LoadMap>::Request request,
                std::shared_ptr<map_server_msgs::srv::LoadMap>::Response response)
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
