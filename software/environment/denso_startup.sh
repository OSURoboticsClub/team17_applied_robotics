#!/bin/bash
VBoxManage startvm Windows --type headless

source /opt/ros/kinetic/setup.bash
source /home/denso/catkin_workspace/devel/setup.bash

roslaunch denso_main denso.launch
