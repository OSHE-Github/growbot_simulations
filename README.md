# Growbot Simulations

- The urdf portion is based on this [ROS tutorial](https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
- The publisher portion is based on this [ROS tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- The heightmap for the gazebo world was created from following this [Open Robotics video](https://vimeo.com/58409707)

## Install `growbot-simulations` on Ubuntu 20.04

### Prerequisites

- You need to have Ubuntu 20.04 and ROS Noetic installed, if you need help with installing ROS you can follow the [guide here](install_noetic_bare_metal.md).

### Clone the repository

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:OSHE-Github/growbot_simulations
```

If you do not intend to contribute code to the github
you can clone with HTTPS instead `git clone https://github.com/OSHE-Github/growbot_simulations.git`

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


### Compile `growbot_simulations` source code

First build the project and source the setup file so that the system knows where to look for your build files (i.e. If you do not run the source command in your terminal none of the roslaunch commands will work)

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running the simulations

Again, every new terminal opened needs to first source the setup.bash file before any of the following commands will work.

```sh
cd ~/catkin_ws
source devel/setup.bash
```

### Launch the Gazebo simulation for the Growbot

```sh
roslaunch growbot_simulations gazebo.launch
```

### Launch rviz to see robot model and sensor data

```sh
roslaunch growbot_simulations rviz.launch
```

### View the ROS nodes and topics as a graph
 
```sh
rqt_graph
```
