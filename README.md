# Project Documentation for Volcano GIS

**A real-time lidar-inertial odometry package. We strongly recommend users read this document thoroughly and test the package with the provided dataset first.**

## Menu

- [**System Architecture**](#system-architecture)
- [**Package Dependency**](#dependency)
- [**Package Installation**](#install)
- [**Prepare Data**](#prepare-data)
- [**Prepare IMU Data**](#prepare-imu-data)
- [**Sample Datasets**](#sample-datasets)
- [**Running the Package**](#run-the-package)
- [**Other Notes**](#other-notes)
- [**Issues**](#issues)
- [**Paper**](#paper)
- [**TODO**](#todo)
- [**Related Packages**](#related-packages)
- [**Acknowledgements**](#acknowledgements)

## System Architecture

![System Architecture](./docs/system_architecture.png)

The system comprises the following components:

- **Sensor Module:** Captures real-time data from lidar and IMU.
- **Data Preprocessing:** Filters, aligns, and formats incoming data.
- **Core Algorithm:** Implements SLAM with tightly-coupled lidar-inertial odometry.
- **Visualization:** Displays processed data in RViz.
- **Logging:** Records essential system activities for debugging.

The architecture ensures efficient, real-time performance while maintaining accuracy.

## Dependency

The project requires the following dependencies:

```bash
sudo apt-get update
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

- **ROS (Robot Operating System):** Facilitates communication between modules.
- **GTSAM (Georgia Tech Smoothing and Mapping):** Provides optimization routines.
- **Eigen:** Linear algebra computations.
- **PCL (Point Cloud Library):** Point cloud processing.

## Installation

Follow these steps to install the package:

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/your-repo.git
cd ..
catkin_make
source devel/setup.bash
```

Verify the installation with:

```bash
roslaunch lio_sam run.launch
```

## Prepare Data

Ensure the data is in the correct format:

- **Timestamp:** Each point cloud should have accurate timestamps for synchronization.
- **Format:** Data should be in .pcd or .bag format.
- **Point Structure:** Include 'x', 'y', 'z', 'intensity', 'time', and 'ring'.

## Prepare IMU Data

The IMU data should meet the following requirements:

- **Frequency:** At least 200Hz for accurate motion estimation.
- **Fields:** Include acceleration, angular velocity, and orientation.
- **Alignment:** Align IMU frame with the sensor frame.

## Sample Datasets

- **Walking Dataset:** [[Download Link]]
- **Park Dataset:** [[Download Link]]
- **Garden Dataset:** [[Download Link]]

## Running the Package

1. Launch the package:

```bash
roslaunch lio_sam run.launch
```

2. Play the sample data:

```bash
rosbag play dataset.bag -r 3
```

3. Monitor the output in RViz.

## Other Notes

- **Calibration:** Ensure proper calibration of sensors.
- **Loop Closure:** Enable for improved long-term consistency.
- **GPS Integration:** Utilize GPS data if available.

## Issues

Common issues and solutions:

- **Misaligned Point Clouds:** Check calibration files.
- **High Latency:** Optimize parameters for better performance.
- **IMU Drift:** Ensure high-quality IMU with proper alignment.

## Paper

Cite this work as follows:

```bibtex
@inproceedings{liosam2020,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Your Name},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2020}
}
```

## TODO

- [ ] Optimize algorithm for better performance.
- [ ] Add support for additional sensors.
- [ ] Enhance visualization features.

## Related Packages

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) - Original implementation.
- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM) - With Scan Context.

## Acknowledgements

Special thanks to Tixiao Shan for the original LIO-SAM implementation.

