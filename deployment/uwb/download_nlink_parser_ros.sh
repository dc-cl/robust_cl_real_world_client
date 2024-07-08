#!/bin/bash

dir_found=$(find ~/cl_es/src/ -type d -name "nlink_parser" -print -quit)

if [ -z "$dir_found" ]; then
    echo "Directory 'nlink_parser' not found in workspace. Cloning repository..."
    git clone --recursive https://github.com/nooploop-dev/nlink_parser.git 
else
    echo "Package 'nlink_parser' have been found in: $dir_found"
fi

# 打开串口权限
sudo usermod -a -G dialout $USER
