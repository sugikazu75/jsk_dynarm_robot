```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt install -y python3-wstool python3-catkin-tools
mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws
sudo rosdep init
rosdep update --include-eol-distros
wstool init src
wstool set -u -t src jsk_dynarm_robot http://github.com/sugikazu75/jsk_dynarm_robot --git
wstool merge -t src src/jsk_dynarm_robot/.rosinstall
wstool update -t src
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin config  --cmake-args -DCMAKE_BUILD_TYPE=Release  -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_BENCHMARK=OFF -DBUILD_WITH_COLLISION_SUPPORT=OFF
catkin build
```
