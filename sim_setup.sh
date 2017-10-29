#!/bin/cd

sudo -E apt-get update &&
sudo apt-get remove gazebo2* &&
sudo apt-get remove ros-indigo-gazebo-* &&
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list' &&
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&
sudo -E apt-get update &&
sudo -E apt-get install gazebo4 &&
sudo -E apt-get install libgazebo4-dev &&
sudo -E apt-get install ros-indigo-gazebo4-ros-pkgs ros-indigo-gazebo4-ros-control &&
sudo -E apt-get install gawk make git curl cmake &&
sudo -E apt-get install g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache realpath libopencv-dev &&
sudo -E pip install future &&
sudo -E apt-get install zlib1g-dev &&
#sudo -E pip2 install pymavlink 
sudo -E pip2 install MAVProxy==1.5.2 catkin_pkg --upgrade &&

cd ~/ark_simulation/jsbsim &&
./autogen.sh --enable-libraries &&
make -j2 &&
sudo make install &&

sudo -E apt-get install python-rosinstall python-catkin-tools ros-indigo-mavlink ros-indigo-octomap-msgs ros-indigo-joy ros-indigo-geodesy ros-indigo-octomap-ros unzip &&

cd ~/ark_simulation/sim_catkin_ws/src &&
catkin_init_workspace || true &&
cd ~/ark_simulation/sim_catkin_ws &&
catkin_make

# To run a demo

# ==== in Terminal 1
# source ~/ark_simulation/sim_catkin_ws/devel/setup.bash
# cd ~/ark_simulation/ardupilot/ArduCopter/
# ../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo --map --console

# ==== in Terminal 2
# cd ~/ark_simulation/sim_catkin_ws
# source devel/setup.bash
# roslaunch ardupilot_sitl_gazebo_plugin erlecopter_spawn.launch

# ==== in Terminal 1
# param load /[path_to_your_home_directory]/ark_simulation/ardupilot/Tools/Frame_params/Erle-Copter.param
