#!/bin/sh
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
sudo -E pip2 install pymavlink MAVProxy catkin_pkg --upgrade &&

mkdir -p ~/ark_simulation; cd ~/ark_simulation &&
wget -O aruco.tgz https://sourceforge.net/projects/aruco/files/1.3.0/aruco-1.3.0.tgz/download &&
tar -xvzf aruco.tgz &&
cd aruco-1.3.0/ &&
mkdir -p build && cd build &&
cmake .. &&
make &&
sudo make install &&

cd ~/ark_simulation &&
git clone https://github.com/erlerobot/ardupilot || true &&
cd ardupilot &&
git checkout gazebo &&

cd ~/ark_simulation &&
git clone https://github.com/tridge/jsbsim.git || true &&
sudo -E apt-get install libtool automake autoconf libexpat1-dev  &&
cd jsbsim &&
./autogen.sh --enable-libraries &&
make -j2 &&
sudo make install &&

sudo -E apt-get install python-rosinstall ros-indigo-octomap-msgs ros-indigo-joy ros-indigo-geodesy ros-indigo-octomap-ros ros-indigo-mavlink unzip &&
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/drc-latest.list' &&
cd ~/ark_simulation &&
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add - &&
sudo -E apt-get update &&
sudo -E apt-get install drcsim &&

mkdir -p ~/ark_simulation/sim_catkin_ws/src &&
cd ~/ark_simulation/sim_catkin_ws/src &&
catkin_init_workspace || true &&
cd ~/ark_simulation/sim_catkin_ws &&
catkin_make &&
source devel/setup.bash &&
cd src/ &&
git clone https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin || true &&
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/ || true &&
git clone https://github.com/erlerobot/rotors_simulator -b sonar_plugin || true &&
git clone https://github.com/PX4/mav_comm.git || true &&
git clone https://github.com/ethz-asl/glog_catkin.git || true &&
git clone https://github.com/catkin/catkin_simple.git || true &&

sudo -E apt-get install python-catkin-tools python-rosinstall-generator &&
cd ~/ark_simulation/sim_catkin_ws &&
wstool init src || true &&
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
git clone https://github.com/erlerobot/erle_gazebo_models || true &&
mv erle_gazebo_models/* ~/.gazebo/models || true

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
