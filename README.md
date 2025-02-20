# Volumetric Flying Object Detection (VoFOD)

This is an implementation of the Volumetric Flying Object Detection (VoFOD) proposed in the paper "On Onboard LiDAR-based Flying Object Detection" (see bottom of this Readme).
VoFOD is a system developed for robust and accurate detection of UAVs using a LiDAR (or a similar spatial sensor) onboard another UAV without relying on markers of any kind.
For more details, see the paper.

## Installation

Prequisites:

* Ubuntu 20.04 (may work on other distros, this one is tested)
* git (`sudo apt install git`)
* curl (`sudo apt install curl`)

Installation instructions (you can just copy & paste them into a terminal):

1. Install [ROS Noetic](http://wiki.ros.org/noetic):
```
curl https://ctu-mrs.github.io/ppa-unstable/add_ros_ppa.sh | bash   # add the PPA repository
sudo apt install ros-noetic-desktop-full                            # install ROS Noetic
```
2. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) and the [MRS Ouster ROS driver](https://github.com/ctu-mrs/ouster-ros):
```
curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash   # add the PPA repository
sudo apt install ros-noetic-mrs-uav-system-full               # install the full MUS1.5-stable (including the Ouster driver)
```
3. Setup a catkin workspace:
```
source /opt/ros/noetic/setup.bash             # source the general ROS workspace so that the local one will extend it and see all the packages
mkdir -p ~/workspace/src && cd ~/workspace    # create the workspace folder in home and cd to it
catkin init -w ~/workspace                    # initialize the new workspace
# setup basic compilation profiles
catkin config --profile debug --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -Og' -DCMAKE_C_FLAGS='-Og'
catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17'
catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17'
catkin profile set reldeb                     # set the reldeb profile as active
```
4. Clone this repository and build it:
```
# it is good practice to not clone ROS packages directly into a workspace, so let's use a separate directory for this
git clone git@github.com:ctu-mrs/vofod.git ~/git/vofod        # clone this repository
# git clone https://github.com/ctu-mrs/vofod.git ~/git/vofod  # if you do not have a private key set up on Github, you can use https instead of ssh
ln -s ~/git/vofod ~/workspace/src                             # create a symbolic link of the repository to the workspace
cd ~/workspace/src && catkin build vofod                      # build the package within the workspace
```

That's it! Now you can start the basic simulation example:
```
source ~/workspace/devel/setup.bash     # source the workspace to see the packages within
roscd vofod/tmux/simulation             # change directory to the one containing the tmux script
./start.sh
```

After this, you should see something like the following:

![VoFOD running in the Gazebo simulator](https://github.com/ctu-mrs/vofod/raw/gifs/vofod_gazebo.gif)

The red sphere in Rviz on the right is the detection of the other UAV, and the green cubes represent the occupied voxels.

## See also

* Complementary pointcloud-based multi-target tracking and estimation algorithm: [https://github.com/ctu-mrs/lidar_tracker](https://github.com/ctu-mrs/lidar_tracker)
* MRS UAV System: [https://github.com/ctu-mrs/mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system)
* Webpage with related multimedia materials: [https://mrs.felk.cvut.cz/flying-object-detection](https://mrs.felk.cvut.cz/flying-object-detection)

## Cite as

Matouš Vrba, Viktor Walter, Václav Pritzl, Michal Pliska, Tomáš Báča, Vojtěch Spurný, Daniel Heřt, and Martin Saska, "On Onboard LiDAR-based Flying Object Detection," IEEE Transactions on Robotics, vol. 41, pp 593-611, 2025.

```
@article{vrba2025OnboardLiDARBasedFlying,
  title = {On Onboard LiDAR-Based Flying Object Detection},
  author = {Vrba, Matouš and Walter, Viktor and Pritzl, Václav and Pliska, Michal and Báča, Tomáš and Spurný, Vojtěch and Heřt, Daniel and Saska, Martin},
  year = {2025},
  journal = {IEEE Transactions on Robotics},
  volume = {41},
  pages = {593--611},
  issn = {1941-0468},
  doi = {10.1109/TRO.2024.3502494}
}
```
