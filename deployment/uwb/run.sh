#!/bin/bash

# 运行例子：$ bash run.sh 0 

id=$1

source devel/setup.bash
roslaunch linktrack _multiple_tags.launch id:=${id}
