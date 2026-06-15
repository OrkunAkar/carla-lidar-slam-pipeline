# Environment Re-installation and Restore Guide

This guide details how to restore and rebuild your entire development environment from scratch on a clean Ubuntu 22.04 LTS installation.

---

## Step 1: System Dependencies & ROS2 Humble Installation

ROS2 Humble is compatible with **Ubuntu 22.04 LTS**.

### 1. Install ROS2 Humble Desktop
Follow the official ROS2 installation commands:
```bash
# Set locale
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Desktop & Dev Tools
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y
```

### 2. Install Additional Libraries & System Packages
```bash
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  terminator \
  imagemagick \
  scrot \
  libomp-dev
```

### 3. Install Python Dependencies
```bash
pip3 install numpy scikit-learn matplotlib open3d
```

---

## Step 2: Download & Install CARLA 0.9.15

1. Download the CARLA package and the additional assets tarball from the [official CARLA releases page](https://github.com/carla-simulator/carla/releases/tag/0.9.15).
2. Extract the files:
```bash
mkdir -p ~/CARLA_PROJECT/CARLA_0.9.15
tar -xzf CARLA_0.9.15.tar.gz -C ~/CARLA_PROJECT/CARLA_0.9.15
tar -xzf AdditionalMaps_0.9.15.tar.gz -C ~/CARLA_PROJECT/CARLA_0.9.15
```

---

## Step 3: Clone Your Git Repository

Clone your saved project codebase back into your workspace:
```bash
cd ~/CARLA_PROJECT
git clone <YOUR_GIT_REPOSITORY_URL> .
```

---

## Step 4: Rebuild the ROS2 Workspaces

Your repository contains the directories `ros2_bridge_ws` and `carla_ros2_ws`. These must be compiled using `colcon`.

### 1. Build the CARLA ROS Bridge
```bash
cd ~/CARLA_PROJECT/ros2_bridge_ws/ros-bridge
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Build the Application Workspaces
```bash
cd ~/CARLA_PROJECT/carla_ros2_ws
source /opt/ros/humble/setup.bash
source ~/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
colcon build --symlink-install
```

---

## Step 5: Install and Build LIO-SAM

LIO-SAM requires GTSAM (Georgia Tech Smoothing and Mapping library) to optimize factor graphs.

### 1. Build GTSAM from source
```bash
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2a9  # Stable tag compatible with ROS2 Humble LIO-SAM
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 2. Compile LIO-SAM in your workspace
LIO-SAM is already located within `~/CARLA_PROJECT/carla_ros2_ws/src/LIO-SAM/`.
```bash
cd ~/CARLA_PROJECT/carla_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam
```

---

## Step 6: Verify and Run the Stack

1. **Verify executable permissions:**
   ```bash
   cd ~/CARLA_PROJECT
   chmod +x *.sh
   ```
2. **Launch the simulator and planner stack:**
   ```bash
   ./launch_stack.sh
   ```
   This will verify that CARLA opens and the ROS bridge correctly spawns the ego vehicle.
3. **Run your custom nodes and SLAM:**
   Ensure bag files are placed in `~/CARLA_PROJECT/bags/` to run playback tests.
