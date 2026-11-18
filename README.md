# simple_terrain_gen

Provides Functionalities to generate terrains. Multiple tools are provided.

## Requirements

This library requires Qt6. MapGen Live further requires RTAB Map and open-vins, as well as a depth camera. Currently it is configured for use with a realsense d435i. MapGen Editor requires VTK. TerrainGen requires fastnoiselite.

## MapGen

MapGen generates a random height map as a stl surface mesh using CGAL. A grid of equal sized triangles is generated and its height is randomized. Map sizes, slope limits and laplacian smoothing can be configured through several parameters.

**Usage:** Start the node by calling ```ros2 run map_gen MapGen```, then call the generation service with ```ros2 service call /map_gen/generate_map map_gen/srv/GenerateMap```

**Parameters:** 

| Parameter | Description |
|-----------|-------------|
| file_path | Location where the generated stl file should be saved. If no path is specified, defaults to only showing a rendering of the terrain instead. |
| in_file_path | Path to an input stl file if you want the generator to just reduce the vertex count instead of generating a random map |
|in_file_reduction_factor | Ratio of vertices that should be kept from the input file |
| seed | Seeding for the random number generator |
| size | Number of triangle pairs per side. Total number of triangles is size \* size \* 2 |
| scale | Side length of each triangle pair. Defaults to 1 |
| max_slope | The maximum slope in degrees allowed before smoothing |
| smoothing_iterations | How many iterations of Laplacian smoothing should be applied |
| smoothing_alpha | Alpha factor to limit effect of neighbours in Laplacian smoothing |
| mode | 0 - generates only the random map, 1 - generates random map and adds a flat border of 1 triangle width around the map for safety |

## TerrainGen

TerrainGen is the counterpart to MapGen, which is capable of generating a map of random terrain surface materials. In contrast to MapGen, where fully random height maps are reasonable, surface materials are usually more uniform. Therefore, perlin, simplex and cellular noise patterns are used for generation. Output is provided both as a yaml and a png.

**Usage:** Start the node by calling ```ros2 run map_gen TerrainGen```, then call the generation service with ```ros2 service call /map_gen/generate_terrai map_gen/srv/GenerateTerrain```

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| file_path | Location where the generated yaml file should be saved |
| seed | Seeding for the random number generator |
| size | Size in pixels of the generated terrain |
| noise_type | Type of noise pattern to be used. Supported types: "perlin", "simplex" and "cellular" |
| octaves | Octaves of the noise generator |
| frequency | Frequency of the noise generator |
| persistence | Persistence of the noise generator |
| lacunarity | Lacunarity of the noise generator |
| stretch | If true, stretches noise values to range from 0.0 to 1.0, ensuring the full range is covered. If false, uses the actual generated noise values. |

## MapGen Live

MapGen Live uses open-vins and RTAB Map to record real world heightmaps from depth camera inputs.

**Usage:** Run the launch script ```ap_gen map_gen_live.launch.py```. A depth camera must be connected in order to use this. Hold the camera still until rviz2 starts showing a camera feed. If the camera feed shows init in the top left corner, make a small but fast movement. After that, you can map the target area by slowly moving the camera to cover the target area. The database is automatically saved at ```.ros/rtabmap.db``` so you can simply close the launch script when finished or disconnect the camera. To convert the database into an object, use ```rtabmap-export --mesh --voxel 0.02 --poisson_depth 8 --min_cluster 0 --output_dir ~/maps ~/.ros/rtabmap.db``` you can modify the poisson depth, but 8 seems to give the best results so far.

## MapGen Editor

This tool can be used to finetune and modify the generated maps. Supports stl and obj files as inputs, and outputs always stl files. On loading, generates a square map that samples the loaded mesh at a fixed distance. Every sample that does not hit the loaded mesh is set to terrain type -1. This is done, as downstream processors expect all heightmaps to be square. The following tools are provided by the editor:

| Tool | Description |
|------|-------------|
| draw | Draws the selected terrain type onto the mesh. Cannot draw on vertices of terrain type -1 |
| erase | Like draw, but always draws terrain type 0 |
| block | Like draw, but always draws terrain type -1 |
| unblock | Like erase, but also works on vertices of terrain type -1 |
| pull | Moves vertices upwards based on the selected intensity |
| push | Moves vertices downwards based on the selected intensity |
| flatten | Moves vertices up or down with intensity to match surrounding vertices |

