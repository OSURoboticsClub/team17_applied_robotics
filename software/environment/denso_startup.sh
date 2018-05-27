#!/bin/bash

until ls /dev/vboxdrv
do
    echo "Waiting for /dev/vboxdrv"
    sleep 1
done

VBoxManage startvm Windows --type headless

source /opt/ros/kinetic/setup.bash
source /home/denso/catkin_workspace/devel/setup.bash

roslaunch denso_main denso.launch
