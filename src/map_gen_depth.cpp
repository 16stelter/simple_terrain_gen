#include "map_gen/map_gen_depth.hpp"

namespace map_gen {

MapGenDepth::MapGenDepth(rclcpp::Node::SharedPtr node, 
                                       const std::string &ns, 
                                       std::vector<rclcpp::Parameter> parameters)
: node_(node)
{
  node_->declare_parameter<std::string>("camera_ns", "camera");
  node_->get_parameter("camera_ns", camera_ns_);

  map_gen_service_ = node_->create_service<map_gen::srv::GenerateMap>(
    "/map_gen_depth/generate_map", 
    std::bind(&MapGenDepth::generate_map, this, std::placeholders::_1, std::placeholders::_2)
  );

  depth_cam_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    camera_ns_ + "/depth_camera/depth/image_raw", 
    10, 
    std::bind(&MapGenDepth::depth_image_cb, this, std::placeholders::_1)
  );
};

void MapGenDepth::generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, 
                          std::shared_ptr<map_gen::srv::GenerateMap::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Generating map from depth image");
};

void MapGenDepth::depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Received depth image");
};

} //map_gen

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("map_gen_depth");

  // Create cpp node
  [[maybe_unused]] map_gen::MapGenDepth map_gen(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}