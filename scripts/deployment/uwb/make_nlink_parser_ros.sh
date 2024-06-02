#!/bin/bash

# 在仓库中下载ROS包
echo "Cloning repository..."
git clone --recursive https://github.com/RonghaiHe/nlink_parser.git 

cd ../

# 编译
echo "Making..."
catkin_make
