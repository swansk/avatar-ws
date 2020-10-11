# Avatar Workspace
> Karl Swanson, Spencer Sochin, Peter Albanese

## Description
This is the ROS workspace for the ANA AVATAR 2020 Northeastern Capstone Team. It contains the software and tools necessary to
operate the AVATAR robotic arm and chassis.

## Installation
We are using ROS Noetic for this workspace, installation instructions can be found below or at: http://wiki.ros.org/noetic/Installation/Ubuntu

### Setup Sources List
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up keys
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Install
Update Debian package index
```
sudo apt update
```
Desktop-Full Install of ROS
```
sudo apt install ros-noetic-desktop-full
```
Install effort controllers for `ros_control`
```
sudo apt install ros-noetic-effort-controllers
```

### Environment Setup
It is a good idea to source the ROS setup script in your bash terminal. If you have multiple ROS installations, you need to comment out the previous source in your bashrc. Add this to your `~/.bashrc` file

```
source /opt/ros/noetic/setup.bash
```

## Building / Sourcing
Before using this workspace and following any modifications to this workspace, make sure to build with catkin:
```
cd WORKSPACE_DIR
catkin_make
```
Next make sure to source the development space:
```
. devel/setup.bash
```
If you are going to be working in this workspace a lot, feel free to add the source command to your bashrc file, so your
terminal session will automatically source the development space of the workspace.

## Usage
Currently, this workspace can visualize the generated URDF in RVIZ / Gazebo.

### Launch Gazebo Simulation
```
roslaunch avatar_gazebo avatar_sim.launch
```

### Launch Effort Controllers
It is necessary to launch the effort controllers before trying to move joints around using RQT or other means.
```
roslaunch avatar_control robot_control.launch
```

### Launch RQT
Using RQT we can tune our PID, set effort commands, and a whole lot more. We can launch RQT with the following:
```
roslaunch avatar_control rqt_dashboard.launch
```

### Launch RVIZ
RVIZ allows us to better visualize how our robot plans to move in space, we can launch it with:
```
roslaunch avatar_description avatar_rviz.launch
```

NOTE: Currently RVIZ should only be launched after Gazebo, I need to add some conditionals in there so that we can launch it
by itself without overwriting `robot_description` on parameter server.

