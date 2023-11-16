unity-ros2 guides and tutorials:
`https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md`


clone and build ros_tcp_endpoint (ros2 branch):
`git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git`

`cd ROS-TCP-Endpoint`

`git checkout main-ros2`

`cd /workspaces/ros2_devcontainer`

`colcon build --symlink-install`

`source /workspaces/ros2_devcontainer/install/setup.bash`

`ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10001`

`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`