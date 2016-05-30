#!/bin/sh
sudo apt-get update &&
sudo apt-get remove gazebo2* &&
sudo apt-get remove ros-indigo-gazebo-* &&
sudo apt-get install gazebo4 &&
sudo apt-get install ros-indigo-gazebo4-ros-pkgs ros-indigo-gazebo4-ros-control &&
sudo apt-get install gawk make git curl cmake &&
sudo apt-get install g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache realpath libopencv-dev &&
sudo pip2 install pymavlink MAVProxy catkin_pkg --upgrade &&

mkdir -p ~/ark_simulation; cd ~/ark_simulation &&
wget -O aruco.tgz https://sourceforge.net/projects/aruco/files/1.3.0/aruco-1.3.0.tgz/download &&
tar -xvzf aruco.tgz &&
cd aruco-1.3.0/ &&
mkdir build && cd build &&
cmake .. &&
make &&
sudo make install &&

cd ~/ark_simulation &&
git clone https://github.com/erlerobot/ardupilot &&
cd ardupilot &&
git checkout gazebo &&

cd ~/ark_simulation &&
git clone https://github.com/tridge/jsbsim.git &&
sudo apt-get install libtool automake autoconf libexpat1-dev  &&
cd jsbsim &&
./autogen.sh --enable-libraries &&
make -j2 &&
sudo make install &&

sudo apt-get install python-rosinstall ros-indigo-octomap-msgs ros-indigo-joy ros-indigo-geodesy ros-indigo-octomap-ros ros-indigo-mavlink unzip &&
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/drc-latest.list' &&
cd ~/ark_simulation &&
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add - &&
sudo apt-get update &&
sudo apt-get install drcsim &&

mkdir -p ~/ark_simulation/sim_catkin_ws/src &&
cd ~/ark_simulation/sim_catkin_ws/src &&
catkin_init_workspace &&
cd ~/ark_simulation/sim_catkin_ws &&
catkin_make &&
source devel/setup.bash &&
cd src/ &&
git clone https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin &&
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/ &&
git clone https://github.com/erlerobot/rotors_simulator -b sonar_plugin &&
git clone https://github.com/PX4/mav_comm.git &&
git clone https://github.com/ethz-asl/glog_catkin.git &&
git clone https://github.com/catkin/catkin_simple.git &&

sudo apt-get install python-catkin-tools python-rosinstall-generator &&
cd ~/ark_simulation/sim_catkin_ws &&
wstool init src &&
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall &&
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall &&
wstool merge -t src /tmp/mavros.rosinstall &&
wstool update -t src &&
cd ~/ark_simulation/sim_catkin_ws/src/mavros &&
git checkout -b 0.17.3-manash &&
git remote add manash https://github.com/ManashRaja/mavros.git &&
git pull manash 0.17.3-indigo &&

cd ~/ark_simulation/sim_catkin_ws &&
touch src/mavlink/CATKIN_IGNORE &&
catkin_make --pkg mav_msgs &&
source devel/setup.bash &&
catkin_make &&
catkin_make &&

cd ~/ark_simulation &&
mkdir -p ~/.gazebo/models &&
git clone https://github.com/erlerobot/erle_gazebo_models &&
mv erle_gazebo_models/* ~/.gazebo/models

# To run a demo
# ------- Terminal 1
# cd ~/ark_simulation/ardupilot/ArduCopter/ 
# ../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo --map --console
# ------- Terminal 2 (New terminal)
# cd ~/ark_simulation/sim_catkin_ws
# source devel/setup.bash
# roslaunch ardupilot_sitl_gazebo_plugin erlecopter_spawn.launch
# ------- Terminal 1
# param load /[path_to_your_home_directory]/ark_simulation/ardupilot/Tools/Frame_params/Erle-Copter.param
