#!/bin/bash

# 检查是否存在串口驱动文件
if [ ! -e "/dev/ttyUSB*" ] && [ ! -e "/dev/ttyCH343USB*" ]; then
    if [ -e "/dev/ttyACM*" ]; then
        echo "Removing cdc-acm module..."
        sudo rmmod cdc-acm
    fi
    echo "Driver not found. Searching for ch343ser_linux directory..."

    # 在 /home 目录下递归查找 ch343ser_linux 目录
    dir_found=$(find /home -type d -name "ch343ser_linux" -print -quit)

    if [ -z "$dir_found" ]; then
        echo "Directory ch343ser_linux not found in /home. Cloning repository..."
        git clone https://github.com/WCHSoftGroup/ch343ser_linux
        dir_found="./ch343ser_linux"
    else
        echo "Directory found: $dir_found"
    fi
    # 进入目录并执行操作
    cd "$dir_found/driver" && {
    	echo "Uninstalling previous driver..."
    	sudo make uninstall
    	echo "Unloading driver module..."
	sudo make unload
    	echo "Compiling and installing the driver..."
    	make
    	sudo make load
    	sudo make install
    	sudo chmod 666 /dev/ttyUSB*
    } || {
        echo "Failed to compile and install the driver."
        exit 1
    }
else
    echo "Device files exist, no action required."
fi
