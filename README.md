# Growbot Simulations

- The urdfs are based on this [ROS tutorial](https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
- The publisher portion is based on this [ROS tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- The heightmap for the gazebo world was created from following this [Open Robotics video](https://vimeo.com/58409707)
- Quick ArUco tags came from [chev.me/arucogen](https://chev.me/arucogen/)
- ArUco maker models for Gazebo came from [github.com/joselusl/aruco_gazebo](https://github.com/joselusl/aruco_gazebo)

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
sudo apt-get install python3-catkin-tools
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch `rviz` to only see robot model and sensor data

```sh
roslaunch growbot_simulations rviz.launch
```

### View the ROS nodes and topics as a graph
 
```sh
rqt_graph
```

### TODO
- [  ] Add ROS Serial Instructions and example
- [  ] Add ROS Subscriber Instructions 
- [  ] Add ROS Publisher Instructions 

