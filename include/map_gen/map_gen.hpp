#ifndef MAP_GEN_INCLUDE_MAP_GEN_H_
#define MAP_GEN_INCLUDE_MAP_GEN_H_

#include <rclcpp/rclcpp.hpp>
#include "map_gen/srv/generate_map.hpp"
#include <filesystem>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/STL.h>
#include <CGAL/boost/graph/IO/STL.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Qt/Basic_viewer.h>
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3                     Point;
typedef CGAL::Surface_mesh<Point>      Mesh;
typedef K::Vector_3                    Vector;

namespace map_gen {
  class MapGen {
    public:
      explicit MapGen(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                                    std::vector<rclcpp::Parameter> parameters = {});
    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Service<map_gen::srv::GenerateMap>::SharedPtr map_gen_service_;
        
      /*
      * Main function. Generates a map of specified properties and writes it to a file.
      */
      void generate_map(const std::shared_ptr<map_gen::srv::GenerateMap::Request> request, std::shared_ptr<map_gen::srv::GenerateMap::Response> response);
  };
} //map_gen

#endif //MAP_GEN_INCLUDE_MAP_GEN_H_
