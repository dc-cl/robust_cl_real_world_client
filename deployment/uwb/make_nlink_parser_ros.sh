#!/bin/bash

echo "Making and Going to workspace"
mkdir -p /home/CL_ws/src && cd /home/CL_ws/src

# 在仓库中下载ROS包
echo "Cloning repository..."
git clone --recursive https://github.com/RonghaiHe/nlink_parser.git 

cd ../

# 编译
echo "Making..."
catkin_make
