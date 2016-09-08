# learning_ros

[![Build Status](https://travis-ci.org/wsnewman/learning_ros.svg?branch=master)](https://travis-ci.org/wsnewman/learning_ros)

This repository accompanies the text **_"A Systematic Approach to Learning Robot Programming with ROS"_**.

Code examples reside in folders corresponding to chapters.

### Installation

#### Setup Scripts

You can use some handy setup scripts to install ROS and all required dependencies, and download and build the example code.  They are located [here](https://github.com/wsnewman/learning_ros_setup_scripts).

#### Manual Installation

This entire repository should be cloned to: ~/ros_ws/src (assuming your ros workspace is named "ros_ws" and resides within your home directory).  

To do so, navigate to ~/ros_ws/src from a terminal and enter:
 - `git clone https://github.com/wsnewman/learning_ros.git`

and also clone the external packages used with:
 - `git clone https://github.com/wsnewman/learning_ros_external_packages.git`

Then, from a terminal, navigate to ~/ros_ws and compile the code with the command:
`catkin_make`