# robust_cl_real-world_client
Code about RobustCL for real-world experiment(client part). Still editting...

# How to Deploy
1. Create the workspace
```bash
mkdir -p ~/cl_ws/src && cd ~/cl_ws/src
```
2. Go to workspace and install this repo by:
```bash
cd ~/cl_ws/src/
git clone https://github.com/dc-cl/robust_cl_real_world_client

# Give execute authority
chmod -R +x ./robust_cl_real_world_client/scripts/

# (Option) If using virtual environment
conda create -n CL python=3.9 # version of Python is not important. If there are conflicts while installing dependencies, just change the version
conda activate CL

# Install necessary Python package
pip install -r ./robust_cl_real_world_client/scripts/requirements.txt
```

2. Deploy the environment for UWB(NoopLoop)
```bash
# (If needed)Install serial driver
# First, go to any directory, such as "~"
cd ~
sudo bash ~/cl_ws/src/robust_cl_real_world_client/deployment/uwb/install_serial_driver.sh

# Make ROS wrapper about UWB
cd ~/cl_ws/src
bash ./robust_cl_real_world_client/deployment/uwb/download_nlink_parser_ros.sh
```

3. Deploy the environment for movement(6-robot, other type needs DIY)
```bash
# Make ROS wrapper about movement
cd ~/cl_ws/src
bash ./robust_cl_real_world_client/deployment/uwb/download_6_robot_ros.sh
```

4. make
```bash
# If using virtual environment, need to set up. we used miniconda 
cd ~/cl_ws
catkin_make -DPYTHON_EXECUTABLE=~/miniconda3/envs/CL/bin/python3
```

# Usage
```bash
# Execute file: "run.sh" with id: 0, for example.
cd ~/cl_ws
bash ./src/robust_cl_real_world_client/deployment/uwb/run.sh 0
```
