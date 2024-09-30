# Benchmarking Vision-Based Object Tracking for USVs in Complex Maritime Environments


## System Requirements 

- Ubuntu 20.04
- ROS Galactic
- Pytorch

## Installing simulator
To run the code, we first need to install the MBZIRC simulator. The below link contains detailed instructions to install the simulator

https://github.com/osrf/mbzirc


## Environment
To deal with the dependencies issue, we will use two workspaces, one for the simulator and the controllers and the other workspace will be for vision trackers. 

### Sim and Control workspace

To be consistent with the simulator installation instructions, we will use the same workspace for controllers, which is called mbzirc_ws. below are the instructions. 

Clone the repository inside the simulator workspace. 

```
pip install transforms3d
cd ~/mbzirc_ws/src
git clone https://github.com/Muhayyuddin/vision-tracking.git
```
from the mbzirc_ws build the code using the following command 

```
cd ..
IGNITION_VERSION=fortress colcon build --merge-install
```
### Vision workspace 
for the vision workspace from the above-cloned repository, copy the "pytracking_image" folder and past it into a separate folder called "tracker_ws/src"
to build the vision workspace, create a virtual environment using the following instructions


```
virtualenv -p python3 tracking
source tracking/bin/activate
cd ~/mbzirc/src
mv -r pytracking_image/ ~/tracker_ws/src/
```
## Run the Code
open a terminal and run the following launch file. This launch file will launch the simulator, spawn the USV into the simulator, launch the robot state publisher, and controller
```
ros2 launch usv_description launch_env_controller.launch.py 
```
open another terminal and run the following command to launch the tracker.

```
cd ~/tracker_ws/src/pytracking_image
python3 tracker_node.py 
```
To change the tracker in tracker_node.py modify the name of the tracker such as tomp, tamos, or seqtrack.
### Acknowledgment 
(Pytracking) https://github.com/visionml/pytracking
(SeqTrack)   https://github.com/microsoft/VideoX/tree/master/SeqTrack
