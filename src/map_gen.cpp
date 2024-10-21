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
  if (request->seed == 0)
  {
    std::srand(static_cast<unsigned int>(std::time(0)));
  }
  else
  {
    std::srand(static_cast<unsigned int>(request->seed));
  }

  Mesh mesh;
  double max_slope_rad = request->max_slope * M_PI / 180.0;
  double max_height_diff = std::tan(max_slope_rad);

  std::vector<Mesh::Vertex_index> vertices;

  for (int i = 0; i <= request->size; ++i) {
    for (int j = 0; j <= request->size; ++j) {
      double z = 0;
      if(i==0 && j==0)
      {
        z = static_cast <float> (rand()) / (static_cast <double> (RAND_MAX/100));
      }
      else if (i==0)
      {
        auto it = mesh.vertices().begin();
        std::advance(it, j - 1);
        z = mesh.point(*it).z() + max_height_diff * (static_cast<double>(rand()) / (RAND_MAX / 2) - 1);
      }
      else if (j==0)
      {
        auto it = mesh.vertices().begin();
        std::advance(it, i * request->size + j - request->size);
        z = mesh.point(*it).z() + max_height_diff * (static_cast<double>(rand()) / (RAND_MAX / 2) - 1);
      }
      else
      {
        auto it = mesh.vertices().begin();
        std::advance(it, j - 1);
        float zj = mesh.point(*it).z();
        it = mesh.vertices().begin();
        std::advance(it, i * request->size + j - request->size);
        float zi = mesh.point(*it).z();
        double max_z = std::min(zi + max_height_diff, zj + max_height_diff);
        double min_z = std::max(zi - max_height_diff, zj - max_height_diff);
        z = min_z + (max_z - min_z) * static_cast<double>(rand()) / RAND_MAX;
      }
      Point p(i, j, z);
      vertices.push_back(mesh.add_vertex(p));
    }
  }

  for (int i = 0; i < request->size; ++i) {
    for (int j = 0; j < request->size; ++j) {
      Mesh::Vertex_index v0 = vertices[i * (request->size + 1) + j];
      Mesh::Vertex_index v1 = vertices[i * (request->size + 1) + (j + 1)];
      Mesh::Vertex_index v2 = vertices[(i + 1) * (request->size + 1) + j];
      Mesh::Vertex_index v3 = vertices[(i + 1) * (request->size + 1) + (j + 1)];

      mesh.add_face(v0, v1, v2);
      mesh.add_face(v1, v3, v2);
    }
  }

  //Laplacian smoothing
  for (int i = 0; i < request->smoothing_iterations; ++i) {
    std::vector<double> new_z;

    for (auto v : mesh.vertices()) {
      std::vector<double> nz;
      for (auto vertex : mesh.vertices_around_face(mesh.halfedge(v))) {
          nz.push_back(mesh.point(vertex).z());
      }
      
      double sum = mesh.point(v).z();
      sum += std::accumulate(nz.begin(), nz.end(), 0.0);
      double avg_z = sum / (nz.size() + 1);
      new_z.push_back(mesh.point(v).z() + request->smoothing_alpha * (avg_z - mesh.point(v).z()));
    }

    int idx = 0;
    for (auto v : mesh.vertices()) {
      auto point = mesh.point(v);
      mesh.point(v) = Point(point.x(), point.y(), new_z[idx++]);
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
    std::ofstream output(request->file_path);
    CGAL::IO::write_STL(output, mesh);
    output.close();
    response->success = true;
  }
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