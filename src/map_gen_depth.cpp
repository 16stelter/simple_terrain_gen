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
    camera_ns_ + "/camera/aligned_depth_to_color/image_raw", 
    10, 
    std::bind(&MapGenDepth::depth_image_cb, this, std::placeholders::_1)
  );
  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_ns_ + "/camera/aligned_depth_to_color/camera_info", 
    10, 
    std::bind(&MapGenDepth::camera_info_cb, this, std::placeholders::_1)
  );
};

void MapGenDepth::generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, 
                          std::shared_ptr<map_gen::srv::GenerateMap::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Generating map from depth image");

  // Camera intrinsic parameters
  const float fx = camera_intrinsics_[0]; // Focal length x
  const float fy = camera_intrinsics_[4]; // Focal length y
  const float cx = camera_intrinsics_[2]; // Principal point x
  const float cy = camera_intrinsics_[5]; // Principal point y

  //convert ros image to cv image
  std::vector<std::vector<float>> depth_data(depth_image_.height, std::vector<float>(depth_image_.width));
  for (int y = 0; y < int(depth_image_.height); ++y)
  {
    for (int x = 0; x < int(depth_image_.width); ++x)
    {
      int idx = (y * depth_image_.width + x) * 2;
      uint16_t depth = depth_image_.data[idx] | (depth_image_.data[idx + 1] << 8); //uint8 to uint16
      depth_data[y][x] = static_cast<float>(depth) / 1000.0; //mm to m
    }
  }

  // Determine step size based on request->size
  int step = (request->size > 0) ? request->size : 1;

  //generate map
  Mesh mesh;
  std::vector<Mesh::Vertex_index> vertices;

  for (int y = 0; y < int(depth_image_.height); y += step)
  {
    for (int x = 0; x < int(depth_image_.width); x += step)
    {
      //transform for each pixels xy coordinates based on z value and camera intrinsics
      float z = depth_data[y][x];
      if(z < 1.0) {
        z = 1.0;
      }
      float X = (x - cx) * z / fx;
      float Y = (y - cy) * z / fy;

      Point p(X, Y, z);
      vertices.push_back(mesh.add_vertex(p));
    }
  }

  int reduced_width = (depth_image_.width + step - 1) / step;
  for (int y = 0; y < int(depth_image_.height / step) - 1; ++y)
  {
    for (int x = 0; x < int(depth_image_.width / step) - 1; ++x)
    {
      //add faces
      Mesh::Vertex_index v0 = vertices[y * reduced_width + x];
      Mesh::Vertex_index v1 = vertices[y * reduced_width + (x + 1)];
      Mesh::Vertex_index v2 = vertices[(y + 1) * reduced_width + x];
      Mesh::Vertex_index v3 = vertices[(y + 1) * reduced_width + (x + 1)];

      mesh.add_face(v0, v2, v1);
      mesh.add_face(v1, v2, v3);
    }
  }
  
  //mesh is inverted and needs to be rotated
  Transformation rotate_x_180(
    1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, -1, 0,
    1
  );
  for (auto v : mesh.vertices()) {
        Point p = mesh.point(v);
        mesh.point(v) = rotate_x_180.transform(p);
    }

  if (request->file_path == "")
  {
    RCLCPP_WARN(node_->get_logger(), "I don't know where you want the file to be saved as file_path is not defined. Showing rendering instead.");
    CGAL::draw(mesh);
    response->success = true;
  }
  else
  {
    std::ofstream output(request->file_path);
    CGAL::IO::write_STL(output, mesh);
    output.close();
    response->success = true;
  }
  RCLCPP_INFO(node_->get_logger(), "Finished generating map.");
};

void MapGenDepth::depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  depth_image_ = *msg;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Depth image received: %d x %d", depth_image_.width, depth_image_.height);
};

void MapGenDepth::camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_intrinsics_ = msg->k;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Camera intrinsics received: fx=%f, fy=%f, cx=%f, cy=%f", camera_intrinsics_[0], camera_intrinsics_[4], camera_intrinsics_[2], camera_intrinsics_[5]);
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