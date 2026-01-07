# Unitree LiDAR L2 ROS2 SLAM

This repository contains a complete ROS2 SLAM solution for the Unitree LiDAR L2, integrating Point-LiO for real-time mapping and localization.

## Overview

This project provides:
- **Unitree LiDAR L2 ROS2 driver** - Real-time point cloud and IMU data publishing
- **Point-LiO SLAM** - State-of-the-art LiDAR-Inertial Odometry for mapping
- **Complete workspace** - Ready-to-use ROS2 workspace with all dependencies

## Hardware Requirements

- **Unitree LiDAR L2** - 360° LiDAR with integrated IMU
- **Ubuntu 22.04** - Recommended OS
- **ROS2 Humble** - Required ROS2 distribution

## System Dependencies

### 1. Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Build Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### 3. Install Required Libraries

```bash
# PCL and related libraries
sudo apt install libpcl-dev pcl-tools

# Boost libraries
sudo apt install libboost-all-dev

# Eigen3
sudo apt install libeigen3-dev

# Other dependencies
sudo apt install libyaml-cpp-dev libgtest-dev
```

## Installation

### 1. Clone the Repository

```bash
git clone git@github.com:Ljinzhou/unitree_ladir_l2_ros2_slam.git
cd unitree_ladir_l2_ros2_slam
```

### 2. Install Dependencies

```bash
# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
# Build all packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

## Hardware Setup

### 1. Connect Unitree LiDAR L2

- Connect the LiDAR to your computer via USB
- The device should appear as `/dev/ttyACM0`
- Verify connection:

```bash
ls -la /dev/ttyACM*
# Should show: crw-rw-rw- 1 root dialout 166, 0 [date] /dev/ttyACM0
```

### 2. Set Permissions (if needed)

```bash
sudo chmod 666 /dev/ttyACM0
# Or add your user to dialout group:
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Usage

### Quick Start - Complete SLAM System

```bash
# Terminal 1: Launch LiDAR driver
source install/setup.bash
ros2 launch unitree_lidar_ros2 node_only.py

# Terminal 2: Launch Point-LiO SLAM with RViz
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### Step-by-Step Launch

#### 1. Launch LiDAR Driver Only

```bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 node_only.py
```

**Expected output:**
```
initialize_type_ = 1
[INFO] [timestamp] [unitree_lidar_ros2_node]: LiDAR initialized successfully
```

#### 2. Verify LiDAR Data

```bash
# Check topics
ros2 topic list
# Should show:
# /unilidar/cloud
# /unilidar/imu

# Check point cloud frequency
ros2 topic hz /unilidar/cloud
# Should show ~18 Hz

# Check IMU frequency  
ros2 topic hz /unilidar/imu
# Should show ~250 Hz
```

#### 3. Launch SLAM

```bash
# Option A: With RViz visualization
ros2 launch point_lio mapping_unilidar_l2.launch.py

# Option B: SLAM node only
ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2.yaml

# Option C: LiDAR-only SLAM (if IMU is unstable)
ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2_lidar_only.yaml
```

#### 4. Launch RViz Separately (if needed)

```bash
ros2 run rviz2 rviz2 -d src/Point-LiO-ROS2-Unilidar/src/point_lio/rviz_cfg/loam_livox.rviz
```

## Configuration

### LiDAR Driver Parameters

Edit `src/unitree_lidar_ros2/src/unitree_lidar_ros2/launch/node_only.py`:

```python
parameters= [
    {'initialize_type': 1},        # 1=Serial, 2=UDP
    {'work_mode': 8},              # LiDAR work mode
    {'serial_port': '/dev/ttyACM0'}, # Serial port
    {'baudrate': 4000000},         # Baud rate
    {'cloud_frame': "unilidar_lidar"}, # Point cloud frame
    {'cloud_topic': "unilidar/cloud"}, # Point cloud topic
    {'imu_frame': "unilidar_imu"},     # IMU frame
    {'imu_topic': "unilidar/imu"},     # IMU topic
]
```

### SLAM Parameters

Edit `src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2.yaml`:

**Key parameters for tuning:**
- `imu_en: true/false` - Enable/disable IMU
- `extrinsic_est_en: true/false` - Enable automatic extrinsic calibration
- `lidar_meas_cov: 0.1` - LiDAR measurement covariance
- `det_range: 50.0` - Detection range (meters)
- `filter_size_surf: 0.2` - Surface filtering size
- `filter_size_map: 0.2` - Map filtering size

## Troubleshooting

### 1. LiDAR Connection Issues

**Problem:** `initialize_type_ = 1` but no data
```bash
# Check device permissions
ls -la /dev/ttyACM0

# Check if device is in use
sudo lsof /dev/ttyACM0

# Try different port
ls /dev/tty*
```

### 2. SLAM Instability

**Symptoms:** Map "flies away" or becomes chaotic

**Solutions:**
1. **Use LiDAR-only mode:**
   ```bash
   ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2_lidar_only.yaml
   ```

2. **Increase covariance values** in `unilidar_l2.yaml`:
   ```yaml
   lidar_meas_cov: 0.5  # Increase from 0.1
   imu_meas_acc_cov: 2.0  # Increase from 1.0
   imu_meas_omg_cov: 2.0  # Increase from 1.0
   ```

3. **Enable extrinsic estimation:**
   ```yaml
   extrinsic_est_en: true
   ```

### 3. RViz Display Issues

**Problem:** "Fixed Frame [odom] does not exist"

**Solution:** Wait for SLAM initialization (should see "IMU Initializing: 100.0%")

**Problem:** No point clouds visible

**Solutions:**
1. Check topic names in RViz match published topics
2. Verify Fixed Frame is set to "odom"
3. Check point cloud topic: `/cloud_registered`

### 4. Build Errors

**Boost linking error:**
```bash
# Install missing Boost libraries
sudo apt install libboost-date-time-dev libboost-system-dev libboost-filesystem-dev
```

**PCL version conflicts:**
```bash
# Remove conflicting PCL versions
sudo apt remove libpcl-*
sudo apt install libpcl-dev
```

## Topics and Services

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | Raw point cloud data |
| `/unilidar/imu` | `sensor_msgs/Imu` | IMU data |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Registered point cloud |
| `/Laser_map` | `sensor_msgs/PointCloud2` | Global map |
| `/Odometry` | `nav_msgs/Odometry` | Robot odometry |
| `/path` | `nav_msgs/Path` | Robot trajectory |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

### Key Frames

- `odom` - World/map frame
- `imu_link` - IMU body frame  
- `unilidar_lidar` - LiDAR frame
- `unilidar_imu` - IMU frame

## Performance Tips

1. **Smooth Motion:** Move slowly and smoothly, especially during initialization
2. **Rich Environment:** Ensure sufficient geometric features in the environment
3. **Avoid Pure Rotation:** Mix translation with rotation for better tracking
4. **Wait for Initialization:** Don't move until "IMU Initializing: 100.0%" appears

## Advanced Usage

### Save Point Cloud Map

Maps are automatically saved as PCD files in the workspace directory when the SLAM node is terminated.

### Custom Launch Files

Create custom launch files for specific scenarios:

```python
# custom_mapping.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your custom configuration
    ])
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project integrates multiple open-source components:
- Unitree LiDAR SDK (Unitree License)
- Point-LiO (BSD License)
- ROS2 packages (Apache 2.0 License)

## Acknowledgments

- [Unitree Robotics](https://www.unitree.com/) for the LiDAR hardware and SDK
- [Point-LiO](https://github.com/hku-mars/Point-LiO) team for the SLAM algorithm
- ROS2 community for the robotics framework

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Search existing GitHub issues
3. Create a new issue with detailed information:
   - System specifications
   - Error messages
   - Steps to reproduce

---

**Happy Mapping! 🗺️🤖**