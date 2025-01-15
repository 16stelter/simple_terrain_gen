# simple_terrain_gen

Provides a ROS2 rolling service that generates terrains as STL files. Two terrain generators are included.

MapGen generates random terrains as follows:
Terrains consist of a triangulation of evenly spaced (in xy plane) vertices with randomized z coordinates. 
Thus, two triangles form a square of size 1\*1.
Terrain generation can be constrained by setting the maximum allowed slope angle.
Laplacian smoothing can be used to smoothen the terrain further.

MapGenDepth generates terrains based on depth images as follows:
Every nth pixel of a depth image is sampled and transformed into a point in 3D space.
Points are then triangulated as above.
If a transform from /world to /camera_color_optical_frame exists, the mesh is transformed into that frame instead.
Maximum slope cannot be controlled and no smoothing is applied.


## Requirements

This package requires CGAL-6.0 or later to run. Place the library in the resources folder.

## How to use

This package offers a ROS2 Service Node. Start the node by calling

```ros2 run map_gen MapGen```

or

```ros2 run map_gen MapGenDepth```

You can call the service using

```ros2 service call /map_gen/generate_map map_gen/srv/GenerateMap```

or

```ros2 service call /map_gen_depth/generate_map map_gen/srv/GenerateMap```

You can specify the following parameters:

* file_path - Location in which the stl file should be saved. If no file path is specified, defaults to just showing a rendering of the terrain instead.
* seed - Seeding for the random number generator to ensure repeatability (MapGen only).
* size - MapGen: Number of triangle pairs per side. Total number of triangles is size\*size\*2. MapGenDepth: step size between pixels that are added to the mesh.
* max_slope - Maximum allowed slope before smoothing, in degrees (MapGen only).
* smoothing_iterations - How many iterations of Laplacian smoothing should be applied (MapGen only).
* smoothing_alpha - Alpha factor to limit effect of neighbours in Laplacian smoothing (MapGen only).
