# growbot_simulations

- The urdf portion is based on this [ROS tutorial](https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
- The publisher portion is based on this [ROS tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## Prerequisites
- have Ubuntu 20.04 and ROS Noetic installed

## Cloning the repo
Create `~/catkin_ws/src` and run the following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/OSHE-Github/growbot_simulations.git
```

## Updating Dependencies
```
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

## Installing Controllers For Robot
```
sudo apt-get update
sudo apt-get install ros-noetic-ros-controllers
```


## Building tutorial package
First build the project and source the setup file so that the system knows where to look for your build files
```
cd ~/catkin_ws
catkin build tutorial
source devel/setup.bash # Or: source devel/setup.zsh if using zsh
```

## Launch the robot
```
roslaunch growbot_simulations gazebo.launch
```

To see the ROS node tree run this command
```
rqt_graph
```
