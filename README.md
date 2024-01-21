# README #

This is the Remote Machine Intelligence Lab Final Project 2 Repository.

Package Description
===================

This package provides a haptic control implementation for the ardrone using the sdk library. Upon launching the package, the user will see two windows: an rviz window and the front camera perspective of the drone with control instructions. The rviz environment contains the world, the position of the drone and its coordinate frame, and an interactive marker representing the reference pose of the drone. After commanding the drone to take off with the 'T'-key and changing the mode to automatic with the 'RCTRL'-key, the teleoperator can move the reference pose of the drone with the haptic device. The drone will then follow this reference in real time and provide force feedback to the teleoperator, thereby informing them of the nearby obstacles to ensure a collision-free flight.

Setup
=====

Ubuntu 18.04 or 20.04
---------------------

Install system dependencies -- this has been done for all the machines in GBR room R020:

    sudo apt-get install libsdl1.2-dev libsdl2-dev

Install ROS -- this has been done for all the machines in GBR room R020:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update

Then, on Ubuntu 20.04

    sudo apt-get install ros-noetic-desktop-full

... or on Ubuntu 18.04

    sudo apt-get install ros-melodic-desktop-full

Other Systems
-------------

Make sure to have SDL 1.2 and 2 installed. Regarding ROS, please see http://wiki.ros.org/noetic/Installation
    

Setup of ROS Workspace
======================

See Remote Machine Intelligence Lab Part 3 Lab 1 task sheet. Follow steps to create ROS workspace and clone necessary repositories to use this package.


Launch
======

Source the ROS workspace from its root:

    source devel/setup.bash

Launch the simulation:

    roslaunch rmil_drone_practicals rmil_drone_rviz_sim.launch

Launch the drone:

    roslaunch rmil_drone_practicals rmil_drone_rviz.launch

