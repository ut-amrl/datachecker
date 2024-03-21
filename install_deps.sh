#!/bin/bash

echo "export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc

cd /home/datachecker
make clean
make