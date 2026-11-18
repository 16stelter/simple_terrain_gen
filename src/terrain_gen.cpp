#include "map_gen/terrain_gen.hpp"

namespace map_gen {
TerrainGen::TerrainGen(rclcpp::Node::SharedPtr node,
                       const std::string &ns,
                       std::vector<rclcpp::Parameter> parameters)
: node_(node)
{
  terrain_gen_service_ = node_->create_service<map_gen::srv::GenerateTerrain>(
    "/map_gen/generate_terrain",
    std::bind(&TerrainGen::generate_terrain, this, std::placeholders::_1, std::placeholders::_2)
  );
};

void TerrainGen::generate_terrain(const std::shared_ptr<map_gen::srv::GenerateTerrain::Request> request, std::shared_ptr<map_gen::srv::GenerateTerrain::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Got request");
  if (request->file_path == "")
  {
    RCLCPP_ERROR(node_->get_logger(), "No file path specified. Cannot generate terrain.");
    response->success = false;
    return;
  }

  if (request->noise_type == "perlin")
  {
    noise_.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
  }
  else if (request->noise_type == "simplex")
  {
    noise_.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
  }
  else if (request->noise_type == "cellular")
  {
    noise_.SetNoiseType(FastNoiseLite::NoiseType_Cellular);
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid noise type specified. Cannot generate terrain.");
    response->success = false;
    return;
  }
  noise_.SetSeed(request->seed);
  noise_.SetFrequency(request->frequency);
  noise_.SetFractalOctaves(request->octaves);
  noise_.SetFractalLacunarity(request->lacunarity);
  noise_.SetFractalGain(request->persistence);

  // Generate noise
  std::vector<std::vector<float>> noise_map(request->size, std::vector<float>(request->size, 0));
  for (int y = 0; y < request->size; y++)
  {
    for (int x = 0; x < request->size; x++)
    {
      float noise_value = noise_.GetNoise((float)x, (float)y);
      noise_map[y][x] = noise_value;
    }
  }

  // Normalize
  std::vector<std::vector<int>> terrain_map(request->size, std::vector<int>(request->size, 0));
  if (request->stretch)
  {
    float min_value = std::numeric_limits<float>::max();
    float max_value = std::numeric_limits<float>::lowest();
    for (int y = 0; y < request->size; y++)
    {
      for (int x = 0; x < request->size; x++)
      {
        if (noise_map[y][x] < min_value)
        {
          min_value = noise_map[y][x];
        }
        if (noise_map[y][x] > max_value)
        {
          max_value = noise_map[y][x];
        }
      }
    }

    for (int y = 0; y < request->size; y++)
    {
      for (int x = 0; x < request->size; x++)
      {
        terrain_map[y][x] = std::min(static_cast<int>(((noise_map[y][x] - min_value) / (max_value - min_value)) * request->terrain_types_count), request->terrain_types_count - 1);
      }
    }
  }
  else
  {
    for (int y = 0; y < request->size; y++)
    {
      for (int x = 0; x < request->size; x++)
      {
        terrain_map[y][x] = std::min(static_cast<int>(((noise_map[y][x] + 1.0f) / 2.0f) * request->terrain_types_count), request->terrain_types_count - 1);
      }
    }
  }

  // Generate yaml file and image
  cv::Mat terrain_image(request->size, request->size, CV_8UC3);
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "terrain" << YAML::Value << YAML::BeginSeq;
  for (int y = 0; y < request->size; y++)
  {
    out << YAML::BeginSeq;
    for (int x = 0; x < request->size; x++)
    {
      int terrain_type = terrain_map[y][x];
      out << terrain_type;
      std::array<uint8_t, 3> color;
      if (terrain_type < 0 || static_cast<std::size_t>(terrain_type) >= color_map_.size())
      {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Terrain type %d is out of bounds. Using default color.", terrain_type);
        color = {0, 0, 0}; // Default to white
      }
      else
      {
        color = color_map_[terrain_type];
      }
      terrain_image.at<cv::Vec3b>(y, x) = cv::Vec3b(color[2], color[1], color[0]);
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  std::filesystem::path file(request->file_path);
  std::ofstream fout(file.replace_extension(".yaml"));
  fout << out.c_str();
  fout.close();
  cv::imwrite(file.replace_extension(".png"), terrain_image);
  response->success = true;
  RCLCPP_INFO(node_->get_logger(), "Terrain generated successfully.");
}


} //map_gen

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("terrain_gen");

  // Create cpp node
  [[maybe_unused]] map_gen::TerrainGen terrain_gen(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}