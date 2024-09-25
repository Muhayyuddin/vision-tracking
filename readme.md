# Benchmarking Vision-Based Object Tracking for USVs in Complex Maritime Environments


## System Requirements 

- Ubuntu 20.04
- ROS Galactic
- Pytorch

## Installing simulator
To run the code, we first need to install the MBZIRC simulator. Below link contains the detailed instructions to install the simulator

https://github.com/osrf/mbzirc


## Environment
In order to deal with the dependencies issue, we will use two workspaces, one for the simulator and the controllers and the other workspace will be for vision trackers. 

### Sim and Control workspace

To be consistent with the simulator installation instructions, we will use the same workspace for controllerz, which is called mbzirc_ws. below are the instructions. 

Clone the repository inside the simulator workspace. 

```
pip install transforms3d
cd mbzirc_ws/src
git clone https://github.com/Muhayyuddin/vision-tracking.git
```
from the mbzirc_ws build the code using the following command 

```
cd ..
IGNITION_VERSION=fortress colcon build --merge-install
```
### Vision workspace 
for the vision workspace from the above-cloned repository, copy the zip files "pytracking" and 'trackers' and past them into a seperate folder called "tracker_ws/src"
to build the vision workspace, create a virtual environment using the following instructions

```
virtualenv -p python3 tracking
source tracking/bin/activate
```
## Run the Code
open a terminal and run the following launch file. This launch fille will launch the simulator, spawn the USV into the simulator, launch the robot state publisher, and controller
```
ros2 launch usv_description launch_env_controller.launch.py 
```
open another terminal and run the following command to launch the tracker.

```
cd ~/tracker_ws/src/pytracking_image
python3 tracker_node.py
```
