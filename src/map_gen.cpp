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
  if(request->in_file_path != "")
  { 
    RCLCPP_INFO(node_->get_logger(), "From file");
    if(request->scale != 0.0)
    {
      RCLCPP_WARN(node_->get_logger(), "The scale parameter is currently only implemented for random maps. Parameter is ignored.");
    }
    CGAL::IO::read_STL(request->in_file_path, mesh);
    RCLCPP_INFO(node_->get_logger(), "Faces: %d", mesh.number_of_faces());
    if(request -> in_file_reduction_factor > 0.0) // 0 is the default in a ros msg. If this is 0 we assume the user did not want mesh reduction.
    {
      // find lowest z value
      double min_z = std::numeric_limits<double>::max();
      for (auto v : vertices(mesh)) {
          double z = mesh.point(v).z();
          if (z < min_z && z != 0.0) {
              min_z = z;
          }
      }

      // remove bottom vertices
      Mesh::Halfedge_index  hr; 
      for (auto v : vertices(mesh)) {
        if(abs(mesh.point(v).z() - min_z) < 1e-2) // small tolerance so all bottom vertices are removed
        {
          Mesh::Halfedge_index  h = halfedge(v, mesh);
          hr = CGAL::Euler::remove_center_vertex(h, mesh);
        }
      }

      CGAL::Euler::remove_face(hr, mesh); // previous step leaves one large face from merging the others. Remove that face here.

      CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<Mesh> stop(request->in_file_reduction_factor);
      int num_collapsed = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop);
      RCLCPP_INFO(node_->get_logger(), "Reduced Faces: %d", mesh.number_of_faces());
    }
  }
  else
  {
    if(request->scale == 0.0)
    {
      request->scale = 1.0;
    }
    RCLCPP_INFO(node_->get_logger(), "Generating new map");
    if (request->seed == 0)
    {
      RCLCPP_WARN(node_->get_logger(), "No seed. Using timestamp for seeding.");
      std::srand(static_cast<unsigned int>(std::time(0)));
    }
    else
    {
      std::srand(static_cast<unsigned int>(request->seed));
    }

    double max_slope_rad = request->max_slope * M_PI / 180.0;
    double max_height_diff = std::tan(max_slope_rad) * request->scale;

    std::vector<Mesh::Vertex_index> vertices;

    for (int i = 0; i <= request->size; ++i) {
      for (int j = 0; j <= request->size; ++j) {
        double z = 0;
        if (request->mode == 1 && (i <= 1 || j <= 1 || i >= request->size - 1 || j >= request->size - 1))
        {
          z = 0.0;
        }
        else if(i==0 && j==0)
        {
          z = static_cast <float> (rand()) / (static_cast <double> (RAND_MAX));
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
        Point p(i * request->scale, j * request->scale, z);
        vertices.push_back(mesh.add_vertex(p));
      }
    }

    for (int i = 0; i < request->size; ++i) {
      for (int j = 0; j < request->size; ++j) {
        Mesh::Vertex_index v0 = vertices[i * (request->size + 1) + j];
        Mesh::Vertex_index v1 = vertices[i * (request->size + 1) + (j + 1)];
        Mesh::Vertex_index v2 = vertices[(i + 1) * (request->size + 1) + j];
        Mesh::Vertex_index v3 = vertices[(i + 1) * (request->size + 1) + (j + 1)];

        mesh.add_face(v0, v2, v1);
        mesh.add_face(v1, v2, v3);
      }
    }

    //Laplacian smoothing
    for (int i = 0; i < request->smoothing_iterations; ++i) {
      std::vector<double> new_z;

      for (auto v : mesh.vertices()) {
        std::vector<double> nz;
        for(auto h : halfedges_around_target(v, mesh)) {
            auto neighbor = source(h, mesh);
            nz.push_back(mesh.point(neighbor).z());
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

    const double barrier_height = 0.05; // 5 cm

    std::vector<Mesh::Vertex_index> barrier_vertices(vertices.size(), Mesh::Vertex_index());

    for (int i = 0; i <= request->size; ++i)
    {
      for (int j = 0; j <= request->size; ++j)
      {
        if (i == 0 || j == 0 || i == request->size || j == request->size)
        {
          int idx = i * (request->size + 1) + j;
          auto p = mesh.point(vertices[idx]);
          barrier_vertices[idx] =
            mesh.add_vertex(Point(p.x(), p.y(), p.z() + barrier_height));
        }
      }
    }

    for (int i = 0; i < request->size; ++i)
    {
      for (int j = 0; j < request->size; ++j)
      {
        if (i == 0 || j == 0 || i == request->size - 1 || j == request->size - 1)
        {
          int idx = i * (request->size + 1) + j;

          auto v0 = vertices[idx];
          auto v1 = vertices[idx + 1];
          auto v2 = vertices[idx + request->size + 1];
          auto v3 = vertices[idx + request->size + 2];

          auto t0 = barrier_vertices[idx];
          auto t1 = barrier_vertices[idx + 1];
          auto t2 = barrier_vertices[idx + request->size + 1];
          auto t3 = barrier_vertices[idx + request->size + 2];

          if (t0 != Mesh::Vertex_index() && t1 != Mesh::Vertex_index())
          {
            mesh.add_face(v0, v1, t1);
            mesh.add_face(v0, t1, t0);
          }
          if (t1 != Mesh::Vertex_index() && t3 != Mesh::Vertex_index())
          {
            mesh.add_face(v1, v3, t3);
            mesh.add_face(v1, t3, t1);
          }
          if (t3 != Mesh::Vertex_index() && t2 != Mesh::Vertex_index())
          {
            mesh.add_face(v3, v2, t2);
            mesh.add_face(v3, t2, t3);
          }
          if (t2 != Mesh::Vertex_index() && t0 != Mesh::Vertex_index())
          {
            mesh.add_face(v2, v0, t0);
            mesh.add_face(v2, t0, t2);
          }
        }
      }
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