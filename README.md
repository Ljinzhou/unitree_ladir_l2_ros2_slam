# Unitree LiDAR L2 ROS2 SLAM

这个仓库包含了Unitree LiDAR L2的完整ROS2 SLAM解决方案，集成了Point-LiO进行实时建图和定位。

## 项目概述

本项目提供：
- **Unitree LiDAR L2 ROS2驱动** - 实时点云和IMU数据发布
- **Point-LiO SLAM** - 先进的激光雷达-惯性里程计用于建图
- **完整工作空间** - 包含所有依赖的即用型ROS2工作空间

## 硬件要求

- **Unitree LiDAR L2** - 360°激光雷达，集成IMU
- **Ubuntu 22.04** - 推荐操作系统
- **ROS2 Humble** - 必需的ROS2发行版

## 系统依赖

### 1. 安装ROS2 Humble

```bash
# 添加ROS2 apt仓库
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# 设置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 安装构建工具

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### 3. 安装必需库

```bash
# PCL和相关库
sudo apt install libpcl-dev pcl-tools

# Boost库
sudo apt install libboost-all-dev

# Eigen3
sudo apt install libeigen3-dev

# 其他依赖
sudo apt install libyaml-cpp-dev libgtest-dev
```

## 安装

### 1. 克隆仓库

```bash
git clone https://github.com/Ljinzhou/unitree_ladir_l2_ros2_slam.git
cd unitree_ladir_l2_ros2_slam
```

### 2. 安装依赖

```bash
# 安装ROS2依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建工作空间

```bash
# 构建所有包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置工作空间环境
source install/setup.bash
```

## 硬件设置

### 1. 连接Unitree LiDAR L2

- 通过USB将激光雷达连接到计算机
- 设备应显示为`/dev/ttyACM0`
- 验证连接：

```bash
ls -la /dev/ttyACM*
# 应显示: crw-rw-rw- 1 root dialout 166, 0 [日期] /dev/ttyACM0
```

### 2. 设置权限（如需要）

```bash
sudo chmod 666 /dev/ttyACM0
# 或将用户添加到dialout组:
sudo usermod -a -G dialout $USER
# 然后注销并重新登录
```

## 使用方法

### 快速开始 - 完整SLAM系统

```bash
# 终端1: 启动激光雷达驱动
source install/setup.bash
ros2 launch unitree_lidar_ros2 node_only.py

# 终端2: 启动Point-LiO SLAM和RViz
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### 分步启动

#### 1. 仅启动激光雷达驱动

```bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 node_only.py
```

**预期输出:**
```
initialize_type_ = 1
[INFO] [时间戳] [unitree_lidar_ros2_node]: LiDAR初始化成功
```

#### 2. 验证激光雷达数据

```bash
# 检查话题
ros2 topic list
# 应显示:
# /unilidar/cloud
# /unilidar/imu

# 检查点云频率
ros2 topic hz /unilidar/cloud
# 应显示 ~18 Hz

# 检查IMU频率  
ros2 topic hz /unilidar/imu
# 应显示 ~250 Hz
```

#### 3. 启动SLAM

```bash
# 选项A: 带RViz可视化
ros2 launch point_lio mapping_unilidar_l2.launch.py

# 选项B: 仅SLAM节点
ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2.yaml

# 选项C: 仅激光雷达SLAM（如果IMU不稳定）
ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2_lidar_only.yaml
```

#### 4. 单独启动RViz（如需要）

```bash
ros2 run rviz2 rviz2 -d src/Point-LiO-ROS2-Unilidar/src/point_lio/rviz_cfg/loam_livox.rviz
```

## 配置

### 激光雷达驱动参数

编辑 `src/unitree_lidar_ros2/src/unitree_lidar_ros2/launch/node_only.py`:

```python
parameters= [
    {'initialize_type': 1},        # 1=串口, 2=UDP
    {'work_mode': 8},              # 激光雷达工作模式
    {'serial_port': '/dev/ttyACM0'}, # 串口
    {'baudrate': 4000000},         # 波特率
    {'cloud_frame': "unilidar_lidar"}, # 点云坐标系
    {'cloud_topic': "unilidar/cloud"}, # 点云话题
    {'imu_frame': "unilidar_imu"},     # IMU坐标系
    {'imu_topic': "unilidar/imu"},     # IMU话题
]
```

### SLAM参数

编辑 `src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2.yaml`:

**调优关键参数:**
- `imu_en: true/false` - 启用/禁用IMU
- `extrinsic_est_en: true/false` - 启用自动外参标定
- `lidar_meas_cov: 0.1` - 激光雷达测量协方差
- `det_range: 50.0` - 检测范围（米）
- `filter_size_surf: 0.2` - 表面滤波尺寸
- `filter_size_map: 0.2` - 地图滤波尺寸

## 故障排除

### 1. 激光雷达连接问题

**问题:** `initialize_type_ = 1` 但无数据
```bash
# 检查设备权限
ls -la /dev/ttyACM0

# 检查设备是否被占用
sudo lsof /dev/ttyACM0

# 尝试不同端口
ls /dev/tty*
```

### 2. SLAM不稳定

**症状:** 地图"飞走"或变得混乱

**解决方案:**
1. **使用仅激光雷达模式:**
   ```bash
   ros2 run point_lio pointlio_mapping --ros-args --params-file src/Point-LiO-ROS2-Unilidar/src/point_lio/config/unilidar_l2_lidar_only.yaml
   ```

2. **增加协方差值** 在 `unilidar_l2.yaml` 中:
   ```yaml
   lidar_meas_cov: 0.5  # 从0.1增加
   imu_meas_acc_cov: 2.0  # 从1.0增加
   imu_meas_omg_cov: 2.0  # 从1.0增加
   ```

3. **启用外参估计:**
   ```yaml
   extrinsic_est_en: true
   ```

### 3. RViz显示问题

**问题:** "固定坐标系 [odom] 不存在"

**解决方案:** 等待SLAM初始化（应看到"IMU初始化: 100.0%"）

**问题:** 看不到点云

**解决方案:**
1. 检查RViz中的话题名称是否匹配发布的话题
2. 验证固定坐标系设置为"odom"
3. 检查点云话题: `/cloud_registered`

### 4. 构建错误

**Boost链接错误:**
```bash
# 安装缺失的Boost库
sudo apt install libboost-date-time-dev libboost-system-dev libboost-filesystem-dev
```

**PCL版本冲突:**
```bash
# 移除冲突的PCL版本
sudo apt remove libpcl-*
sudo apt install libpcl-dev
```

## 话题和服务

### 发布的话题

| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | 原始点云数据 |
| `/unilidar/imu` | `sensor_msgs/Imu` | IMU数据 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 配准点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 全局地图 |
| `/Odometry` | `nav_msgs/Odometry` | 机器人里程计 |
| `/path` | `nav_msgs/Path` | 机器人轨迹 |
| `/tf` | `tf2_msgs/TFMessage` | 变换树 |

### 关键坐标系

- `odom` - 世界/地图坐标系
- `imu_link` - IMU本体坐标系  
- `unilidar_lidar` - 激光雷达坐标系
- `unilidar_imu` - IMU坐标系

## 性能提示

1. **平滑运动:** 缓慢平滑地移动，特别是在初始化期间
2. **丰富环境:** 确保环境中有足够的几何特征
3. **避免纯旋转:** 将平移与旋转混合以获得更好的跟踪
4. **等待初始化:** 在"IMU初始化: 100.0%"出现之前不要移动

## 高级用法

### 保存点云地图

当SLAM节点终止时，地图会自动保存为工作空间目录中的PCD文件。

### 自定义启动文件

为特定场景创建自定义启动文件:

```python
# custom_mapping.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 你的自定义配置
    ])
```

## 贡献

1. Fork仓库
2. 创建功能分支
3. 进行更改
4. 彻底测试
5. 提交拉取请求

## 许可证

本项目集成了多个开源组件:
- Unitree LiDAR SDK (Unitree许可证)
- Point-LiO (BSD许可证) - 来自 [LycanW/Point-LiO-ROS2-Unilidar](https://github.com/LycanW/Point-LiO-ROS2-Unilidar)
- ROS2包 (Apache 2.0许可证)

## 致谢

- [Unitree Robotics](https://www.unitree.com/) 提供激光雷达硬件和SDK
- [Point-LiO](https://github.com/hku-mars/Point-LiO) 团队提供SLAM算法
- [LycanW](https://github.com/LycanW/Point-LiO-ROS2-Unilidar) 提供Point-LiO的ROS2移植
- ROS2社区提供机器人框架

## 支持

如有问题和疑问:
1. 查看上面的故障排除部分
2. 搜索现有的GitHub问题
3. 创建新问题并提供详细信息:
   - 系统规格
   - 错误消息
   - 重现步骤

---

**祝建图愉快! 🗺️🤖**