# hector_model_conversions
Packages related to the conversion of 3d models to different representations.


## mesh_conversions
A package that uses [PyMeshLab](https://github.com/cnr-isti-vclab/PyMeshLab) to load, convert and save meshes in several file formats. The available input/output formats can be found [here](https://pymeshlab.readthedocs.io/en/latest/io_format_list.html).

### Dependencies
In order to use the mesh_conversions package, PyMeshLab needs to be installed:
```
pip3 install pymeshlab
```

### Usage
The mesh_conversions node can be startet using
```
roslaunch mesh_conversions convert_mesh.launch
```
It provides a service called `convert_mesh` which requires an input- and an output-filename. The conversion is done according to the file extensions.

<br>

## mesh_to_sampled_point_cloud

This package can be used to convert obj- or ply-meshes into a point cloud using point cloud sampling. The pointcloud is stored in a pcd file.

### Usage
The mesh_to_sampled_point_cloud node can be startet using
```
roslaunch mesh_to_sampled_point_cloud mesh_to_sampled_point_cloud.launch
```
It provides a service called `convert_mesh_to_sampled_point_cloud`. The service request has the following fields:

* input_file
* output_file
* num_sample_points (Number points to be sampled during point cloud sampling.)
* voxel_filter_leaf_size (Leaf size for filtering point cloud using a voxel grid filter.)
* translation
* rotation_euler_deg (Euler angles (Z''\*Y'\*X) in degrees.)
* scale_factor
* write_normals (Whether normals should be computed and written when saving the file.)
* write_colors (Whether colors should be converted and written when saving the file.)

<br>

## mesh_to_sdf
A package that uses [voxblox_ground_truth](https://github.com/tu-darmstadt-ros-pkg/voxblox_ground_truth) to compute TSDFs and ESDFs from a triangular mesh. Up to now it is only possible to use triangular ply files as input. The SDFs are stored in `.tsdf`- and `.esdf`-files.

### Usage
The mesh_to_sdf node can be startet using
```
roslaunch mesh_to_sdf mesh_to_sdf.launch
```
It provides two services called `convert_ply_to_tsdf` and `convert_ply_to_esdf`. The service request has for both services the following fields:

* input_file
* output_file
* voxel_size
* transform
* scale_factor
* fill_inside (whether the mesh inside is free or occupied)
* floodfill_unoccupied (floodfill unoccupied space)
* truncation_distance (truncation_distance for TSDF, if it is 0, no truncation will be done)

