# ReUseX

ReUseX is a tool for processing scan data captured with an iPhone / iPad for the purpose of mapping buildings. Currently the only supported source is the [StrayScanner](https://github.com/strayrobots/scanner) app which can be found in the [App Store](https://apps.apple.com/us/app/stray-scanner/id1557051662). The app captures LiDAR data as png depth images and RGB images as a mp4 video file, as well as as odometry data  in a csv file. Together with confidence mapps stored as png images, these files are used to create a point cloud representation of the scanned environment.
The point cloud is then used to create a simplified 3d surface modle of the scanned environment, while the images are used to semantically segment the point cloud into individual components, such as walls, windows, doors, etc.


## Getting started

This project definces its dependencies in a nix flake file, to lauch a development shell first install [nix](https://nixos.org/download.html) and then run:

```shell
nix develop
```



### Build instructions:

```shell
git clone https://github.com/pfmephisto/ReUseX
cd ReUseX
nix build
```

### Dependencies:
As of now, the project uses the following dependencies, though an updated list can be found in the `flake.nix` file:

__Core dependencies:__
- pcl
- eigen
- cgal
- opencv
- boost
- tbb
- mpfr

__Logging dependencies:__
- fmt
- spdlog
- spdmon

__Solvers dependencies:__
- g2o
- embree
- scip-solver
- gurobi

__I/O dependencies:__
- opennurbs
- hdf5
- highfive
- xtensor
- xtensor-io

__Visualization dependencies:__
- glfw
- imgui
- glm
- libGLU

__Python dependencies:__
 - pybind11
 - python

## Usage
__Am example of how to access the data in the point cloud from Python:__

As long as fields are continuous in memory, `structured_to_unstructured` will provide a view of the data rather than a copy.

```python
from numpy.lib.recfunctions import structured_to_unstructured

cloud = PointCloud("path_to_file.pcd")
data = np.asarray(cloud)

# View of position values
structured_to_unstructured(data[["x","y","z"]])

# View of normal values
structured_to_unstructured(data[["normal_x","normal_y","normal_z"]])

# View of RGB values
data["rgb"].view("u4").reshape(-1, 1).view("u1")[:, :3]
```
