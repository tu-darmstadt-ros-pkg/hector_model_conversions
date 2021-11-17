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