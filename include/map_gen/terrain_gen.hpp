#ifndef MAP_GEN_INCLUDE_TERRAIN_GEN_H_
#define MAP_GEN_INCLUDE_TERRAIN_GEN_H_

#include <rclcpp/rclcpp.hpp>
#include "map_gen/srv/generate_terrain.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <FastNoiseLite.h>

namespace map_gen {
  class TerrainGen {
    public:
      explicit TerrainGen(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                                    std::vector<rclcpp::Parameter> parameters = {});
    private:

      std::vector<std::array<uint8_t, 3>> color_map_ = {
        {137, 174, 231},   // Light Blue
        {128, 0,   128},   // Purple
        {255, 255, 0  },   // Yellow
        {128, 0,   0  },   // Dark Red
        {255, 0,   0  },   // Red
        {0,   255, 0  },   // Green
        {0,   0,   255},   // Blue
        {255, 165, 0  },   // Orange
        {0,   255, 255},   // Cyan
        {255, 0,   255},   // Magenta
        {0, 128, 0  },     // Dark Green
        {0, 0, 128},       // Dark Blue
        {0, 128, 128},     // Teal
        {210, 105, 30},    // Chocolate
        {255, 255, 255},   // White
        {75, 0, 130}       // Indigo
      };

      FastNoiseLite noise_;
        
      rclcpp::Node::SharedPtr node_;
      rclcpp::Service<map_gen::srv::GenerateTerrain>::SharedPtr terrain_gen_service_;
        
      /*
      * Main function. Generates a terrain of specified properties and writes it to a file.
      */
      void generate_terrain(const std::shared_ptr<map_gen::srv::GenerateTerrain::Request> request, std::shared_ptr<map_gen::srv::GenerateTerrain::Response> response);
  };
} //map_gen

#endif //MAP_GEN_INCLUDE_TERRAIN_GEN_H_