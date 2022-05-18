sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
sudo apt-get install git
sudo apt-get install -y libgazebo11-dev
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
catkin_make
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro noetic
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
cd ~/catkin_ws/
catkin_make
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/mario-serna/pioneer_p3dx_model.git
cd ..
catkin_make
cd ..
source ~/catkin_ws/devel/setup.sh