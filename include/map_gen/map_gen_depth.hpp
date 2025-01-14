#ifndef MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_
#define MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_

#include <rclcpp/rclcpp.hpp>
#include "map_gen/srv/generate_map.hpp"
#include <sensor_msgs/msg/image.hpp>

namespace map_gen {
    class MapGenDepth {
        public:
            explicit MapGenDepth(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                                          std::vector<rclcpp::Parameter> parameters = {});
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Service<map_gen::srv::GenerateMap>::SharedPtr map_gen_service_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_cam_sub_;
            std::string camera_ns_;
            
            /*
            * Main function. Generates a map from depth image.
            */
            void generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, std::shared_ptr<map_gen::srv::GenerateMap::Response> response);
            void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    };
} //map_gen

#endif //MAP_GEN_INCLUDE_MAP_GEN_DEPTH_H_