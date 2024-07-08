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
bash ./robust_cl_real_world_client/deployment/uwb/make_nlink_parser_ros.sh
```

# Usage
```bash
# Execute file: "run.sh" with id: 0, for example.
cd ~/cl_ws
bash ./src/robust_cl_real_world_client/deployment/uwb/run.sh 0
```
