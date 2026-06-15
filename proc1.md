terminal-1
bash $HOME/CARLA_PROJECT/CARLA_0.9.15/CarlaUE4.sh -prefernvidia -quality-level=Low

terminal-2
source /opt/ros/humble/setup.bash
source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

terminal-3
source /opt/ros/humble/setup.bash
source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py

terminal-4
source /opt/ros/humble/setup.bash
source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
ros2 run carla_ad_agent local_planner

terminal-5
source /opt/ros/humble/setup.bash
source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
ros2 run navigation navigation_hmi

term-6
source /opt/ros/humble/setup.bash    
rviz2

TWEAKS
add/marker_array/carla_road_network
add/marker_array/carla/markers
path/carla/waypoints //2d goalpose
add/pointcloud2/carla/hero/lidar
