# robust_cl_real-world_client
Code about RobustCL for real-world experiment(client part). Still editting...

# How to Deploy
1. Create the workspace
```bash
mkdir -p ~/CL_ws/src && cd ~/CL_ws/src
```
2. Go to workspace and install this repo by:
```bash
cd ~/CL_ws/src/
git clone https://github.com/dc-cl/robust_cl_real_world_client
chmod -R +x ./robust_cl_real_world_client/scripts/
```

2. Deploy the environment for UWB(NoopLoop)
```bash
# (If needed)Install serial driver
# First, go to any directory
cd ~
sudo bash ~/CL_ws/src/robust_cl_real_world_client/deployment/uwb/install_serial_driver.sh

# Make ROS wrapper about UWB
cd ~/CL_ws/src
bash ./robust_cl_real_world_client/deployment/uwb/make_nlink_parser_ros.sh
```

# Usage
```bash
# Execute file: "run.sh" with id: 0, for example.
cd ~/CL_ws
bash ./src/robust_cl_real_world_client/deployment/uwb/run.sh 0
```
