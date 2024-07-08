#!/bin/bash

dir_found=$(find ~/cl_es/src/ -type d -name "wpb_move" -print -quit)

if [ -z "$dir_found" ]; then
    echo "Directory 'wpb_move' not found in workspace. Cloning repository..."
    git clone https://github.com/dc-cl/wpb_move.git 
else
    echo "Package 'wpb_move' have been found in: $dir_found"
fi