# LiODOM - Adaptive Local Mapping for Robust LiDAR-Only Odometry

LiODOM is an open source C++ library for LiDAR-Only pose estimation and map building. It is based on minimizing a loss function derived from a set of weighted edge-to-line correspondences with a local map. The (unoptimized) global map is represented by a fast and efficient hash-based grid structure that speeds up searches and updates. The local map is obtained from the global map according to the current position of the vehicle.

Note that LiODOM is research code. The authors are not responsible for any errors it may contain. 

**USE IT AT YOUR OWN RISK!**

# Conditions of use

LiODOM is distributed under the terms of the [GPL3 License](http://github.com/emiliofidalgo/liodom/blob/master/LICENSE).

# Related publication

The details of the algorithm are explained in the following [publication](https://www.sciencedirect.com/science/article/pii/S0921889022001324):

**LiODOM: Adaptive Local Mapping for Robust LiDAR-Only Odometry**<br/>
Emilio Garcia-Fidalgo, Joan P. Company-Corcoles, Francisco Bonnin-Pascual and Alberto Ortiz<br/>
Robotics and Autonomous Systems 156, 104226, 2022<br/>

If you use this code, please cite as:
```
@article{Garcia-Fidalgo2022,
	title = {{LiODOM: Adaptive Local Mapping for Robust LiDAR-Only Odometry}},
	journal = {Robotics and Autonomous Systems},
	volume = {156},
	pages = {104226},
	year = {2022},
	issn = {0921-8890},
	doi = {https://doi.org/10.1016/j.robot.2022.104226},
	author = {Emilio Garcia-Fidalgo and Joan P. Company-Corcoles and Francisco Bonnin-Pascual and Alberto Ortiz},
}
```

# Installation

## Prerequisites

- Tested on [Ubuntu 64-bit 24.04](http://ubuntu.com/download/desktop)
- Tested on [ROS2 Kilted](https://docs.ros.org/en/kilted/index.html)
- [CMake 3.28.3](https://cmake.org/download/)
- [Conan 2.17](https://conan.io/)
    - Conan is used for handling 3rd party libraries

## Adding ROS 2 Packages

First add ROS 2 to APT sources:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /etc/apt/trusted.gpg.d/ros.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/trusted.gpg.d/ros.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

## Installing Prerequisites

First install ROS 2, Python, build-essential, cmake and Python packages using `apt`:

```bash
sudo apt-get install ros-kilted-desktop ros-kilted-pcl-ros ros-kilted-pcl-conversions ros-kilted-ament-cmake \
    build-essential python3 python-is-python3 python3-colcon-common-extensions pipx cmake libudev-dev
```

After that install Conan and detect what compilers have been installed:

```bash
pipx install conan
conan profile detect
```

## Build

Create a workspace and clone `liodom`:

```bash
mkdir my_workspace/src
cd my_workspace/src
git clone https://github.com/emiliofidalgo/liodom.git
```

After having cloned the repository, checkout the correct branch. At the time of writing this document, ROS 2 version is in branch `ros2`.

Once you have cloned the repository, and checked out the correct branch, you need to install the 3rd party libraries using Conan.
When running this for the first time, it will take some time to build the packages (unless pre-built binaries exist).
Built binaries are stored in a local cache, so when running for the second time, these are used:

```bash
cd my_workspace
conan install src/liodom --output-folder=build/liodom --build=missing
```

And finally build the binaries:

```bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select liodom \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=${PWD}/build/liodom/conan_toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release
```

The toolchain file `conan_toolchain.cmake` tells CMake where Conan binaries can be found so that `find_package` works correctly. Above
command installs the package in `my_workspace/install`. In order to make the installed packages visible to the system, run the following:

```bash
cd my_workspace
source install/setup.bash
```

# Usage

For an example of use, see the launch file `launch/liodom.launch`.

Depending on the computer on which LiODOM is running, there are three critical parameters that can affect its performance:
- `scan_regions`: The number of regions in which each horizontal scan is divided.
- `edges_per_region`: The number of edges to detect on each region.
- `prev_frames`: The number of previous frames to be maintained in the local map.

Reducing the values of these parameters may speed up LiODOM, sacrificing accuracy. Adjust them to your needs!

# Contact

If you have problems or questions using this code, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/liodom/issues) and [contributions](http://github.com/emiliofidalgo/liodom/pulls) are totally welcome.

# Acknowledgements

Thanks to the authors of [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [F-LOAM](https://github.com/wh200720041/floam) for publishing their codes. Some parts of this library are inspired by those code bases.

