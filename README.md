# learning_ros
This repository accompanies the text "A Systematic Approach to Learning Robot Programming with ROS".
Code examples reside in folders corresponding to chapters.

This entire repository should be cloned to: `~/ros_ws/src` (assuming your ros workspace is named "ros_ws" and resides within your home directory).

To do so, navigate to ~/ros_ws/src from a terminal and enter:

```
git clone --recursive https://github.com/wsnewman/learning_ros.git
```

And also clone the external packages used with:

```
git clone --recursive https://github.com/wsnewman/learning_ros_external_pkgs_noetic.git
```


Then, from a terminal, navigate to ~/ros_ws and compile the code with the command:
```
catkin_make
```

If you are installing ROS for the first time or encounter missing dependencies, see the instructions here:
[installation scripts](//github.com/wsnewman/learning_ros_setup_scripts)

The scripts located at this site automate installation of ROS (consistent with the version and packages used with the learning-ROS code examples).  These scripts also install a variety of useful tools.


### Copy-and-paste shortcut for noetic users
```
mkdir -p ~/ros_ws/src/
cd ~/ros_ws/src/
git clone --recursive https://github.com/wsnewman/learning_ros.git
git clone --recursive https://github.com/wsnewman/learning_ros_external_pkgs_noetic.git
cd ~/ros_ws/
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-noetic-moveit
catkin_make
```
