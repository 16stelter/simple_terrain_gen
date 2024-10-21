cmake_minimum_required(VERSION 3.12)
project(map_gen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()  

set(CGAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/CGAL-6.0")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateMap.srv"
)

include_directories(include 
                    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp 
                    ${CGAL_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused -fPIC)

set(SOURCES
  src/map_gen.cpp
)

add_executable(MapGen ${SOURCES})

ament_target_dependencies(
  MapGen
  ament_cmake
  rclcpp
  rosidl_default_runtime
  ament_index_cpp
)

rosidl_target_interfaces(MapGen ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(MapGen
                      rclcpp::rclcpp  
                      CGAL::CGAL 
                      Qt6::Core 
                      Qt6::Widgets
                      CGAL::CGAL_Basic_viewer
)

install(TARGETS MapGen DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

ament_package()
