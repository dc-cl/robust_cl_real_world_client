#!/bin/bash

# 运行例子：$ bash run.sh 0 

# 尝试使用第一个参数作为id，如果未提供，则使用默认值0
id=${1:0}

source devel/setup.bash
roslaunch nlink_parser linktrack_multi_tags.launch id:=${id}
