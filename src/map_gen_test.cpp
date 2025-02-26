#include "map_gen/map_gen.hpp"

namespace PMP = CGAL::Polygon_mesh_processing;
namespace map_gen {

MapGen::MapGen(rclcpp::Node::SharedPtr node, 
                                       const std::string &ns, 
                                       std::vector<rclcpp::Parameter> parameters) 
: node_(node)
{
  map_gen_service_ = node_->create_service<map_gen::srv::GenerateMap>(
    "/map_gen/generate_map", 
    std::bind(&MapGen::generate_map, this, std::placeholders::_1, std::placeholders::_2)
  );
};

void MapGen::generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, std::shared_ptr<map_gen::srv::GenerateMap::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Got request");
  Mesh mesh;

  RCLCPP_INFO(node_->get_logger(), "Generating new map");
  if (request->seed == 0)
  {
    std::srand(static_cast<unsigned int>(std::time(0)));
  }
  else
  {
    std::srand(static_cast<unsigned int>(request->seed));
  }

  std::vector<Mesh::Vertex_index> vertices;

  for (int i = 0; i <= request->size; i=i+request->smoothing_iterations) {
    for (int j = 0; j <= request->size; j=j+request->smoothing_iterations) {
      double z = 0;
      if(i < request->size/2 && j < request->size/2)
      {
        z = 0.0;
      }
      else if (i < request->size/2)
      {
        z = 2.0;
      }
      else if (j < request->size/2)
      {
        z = 2.0;
      }
      else
      {
        z = 4.0;
      }
      Point p(i, j, z);
      vertices.push_back(mesh.add_vertex(p));
    }
  }

  for (int i = 0; i < request->size/request->smoothing_iterations; ++i) {
    for (int j = 0; j < request->size/request->smoothing_iterations; ++j) {
      Mesh::Vertex_index v0 = vertices[i * (request->size/request->smoothing_iterations + 1) + j];
      Mesh::Vertex_index v1 = vertices[i * (request->size/request->smoothing_iterations + 1) + (j + 1)];
      Mesh::Vertex_index v2 = vertices[(i + 1) * (request->size/request->smoothing_iterations + 1) + j];
      Mesh::Vertex_index v3 = vertices[(i + 1) * (request->size/request->smoothing_iterations + 1) + (j + 1)];

      mesh.add_face(v0, v2, v1);
      mesh.add_face(v1, v2, v3);
    }
  }

  Mesh steep_mesh;
  std::map<Point, Mesh::Vertex_index> vertex_map;

  for (const auto& f : mesh.faces()) {
    Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, mesh);
    double angle = std::acos(normal.z()) * 180 / M_PI;
    if (angle > 15) {
      auto h = mesh.halfedge(f);
      std::vector<Mesh::Vertex_index> vs;

      for (int i = 0; i < 3; ++i) {
        Point p = mesh.point(mesh.target(h));
        auto it = vertex_map.find(p);
        if (it == vertex_map.end()) {
          Mesh::Vertex_index idx = steep_mesh.add_vertex(p);
          vertex_map[p] = idx;
        }
        vs.push_back(vertex_map[p]);
        h = mesh.next(h); 
      }
      steep_mesh.add_face(vs);
    }
  }

  if (request->file_path == "")
  {
    RCLCPP_WARN(node_->get_logger(), "I don't know where you want the file to be saved as file_path is not defined. Showing rendering instead.");
    CGAL::draw(mesh);
    response->success = true;
  }
  else
  {
    std::filesystem::path file(request->file_path);
    std::ofstream output_steep(file.replace_extension(".steep.stl"));
    std::ofstream output(request->file_path);

    CGAL::IO::write_STL(output, mesh);
    CGAL::IO::write_STL(output_steep, steep_mesh);
    output.close();
    output_steep.close();
    response->success = true;
  }
  RCLCPP_INFO(node_->get_logger(), "Finished generating map.");
}



} //map_gen

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("map_gen");

  // Create cpp node
  [[maybe_unused]] map_gen::MapGen map_gen(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}