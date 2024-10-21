# simple_terrain_gen

Provides a ROS2 rolling service that generates random terrains as STL files. 
Terrains consist of a triangulation of evenly spaced (in xy plane) vertices with randomized z coordinates. 
Thus, two triangles form a square of size 1\*1.
Terrain generation can be constrained by setting the maximum allowed slope angle.
Laplacian smoothing can be used to smoothen the terrain further.

## Requirements

This package requires CGAL-6.0 or later to be available. Place the library in the resources folder.

## How to use

This package offers a ROS2 Service Node. Start the node by calling

```ros2 run map_gen MapGen```

You can call the service using

```ros2 service call /map_gen/generate_map map_gen/srv/GenerateMap```

You can specify the following parameters:

* file_path - Location in which the stl file should be saved. If no file path is specified, defaults to just showing a rendering of the terrain instead.
* seed - Seeding for the random number generator to ensure repeatability.
* size - Number of triangle pairs per side. Total number of triangles is size\*size\*2.
* max_slope - Maximum allowed slope before smoothing, in degrees.
* smoothing_iterations - How many iterations of Laplacian smoothing should be applied.
* smoothing_alpha - Alpha factor to limit effect of neighbours in Laplacian smoothing.
