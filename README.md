# Growbot Simulations

- The urdf portion is based on this [ROS tutorial](https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
- The publisher portion is based on this [ROS tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- The heightmap for the gazebo world was created from following this [Open Robotics video](https://vimeo.com/58409707)

## Install `growbot_simulations` on Ubuntu 20.04

### Prerequisites

- You need to have Ubuntu 20.04 and ROS Noetic installed, if you need help with installing ROS you can follow the [guide here](install_noetic_bare_metal.md).

### Clone the repository

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:OSHE-Github/growbot_simulations
```

If you do not intend to contribute code you can clone with HTTPS instead `git clone https://github.com/OSHE-Github/growbot_simulations.git`

### Update ROS Dependencies

```sh
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

### Install the controllers package needed for simulation

```sh
sudo apt-get update
sudo apt-get install ros-noetic-ros-controllers
```

### Get the `aruco_ros` dependency

```sh
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/aruco_ros.git
```

### Compile `growbot_simulations` source code

First build the project and source the setup file so that the system knows where to look for your build files (i.e. If you do not run the source command in your terminal none of the roslaunch commands will work)

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### One last install step, copy this repo's models to `~/.gazebo/models`

This will have to be run every time you update a model in
`~/catkin_ws/src/growbot_simulations/models/`

```sh
cp -r ~/catkin_ws/src/growbot_simulations/models ~/.gazebo/models
```

## Running the simulations

Again, every new terminal opened needs to first source the setup.bash file before any of the following commands will work.

```sh
cd ~/catkin_ws
source devel/setup.bash
```

If you do not want to have to type this every time you open a new termainal window, you can add `source
~/catkin_ws/devel/setup.bash` to your `~/.bashrc`:

```sh
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Launch the main Gazebo simulation for the Growbot

```sh
roslaunch growbot_simulations gazebo.launch
```

### Launch `rviz` to only see robot model and sensor data

```sh
roslaunch growbot_simulations rviz.launch
```

### View the ROS nodes and topics as a graph
 
```sh
rqt_graph
```

### Drive the robot around

```sh
roslaunch growbot_simulations nc_teleop.launch
```

### Launch the ArUco marker detection node

```sh
roslaunch growbot_simulations aruco_marker_finder_gazebo.launch
```

### View the ArUco detectors results

```sh
rosrun image_view image_view image:=/aruco_single/result
```

### View just the robot's webcam

```sh
rosrun image_view image_view image:=/growbot/camera1/image_raw
```
