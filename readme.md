# Benchmarking Vision-Based Object Tracking for USVs in Complex Maritime Environments
Abstract:
Vision-based target tracking is crucial for unmanned surface vehicles (USVs) to perform tasks such as inspection, monitoring, and surveillance. However, real-time tracking in complex maritime environments is challenging due to dynamic camera movement, low visibility, and scale variation. Typically, object detection methods combined with filtering techniques are commonly used for tracking, but they often lack robustness, particularly in the presence of camera motion and missed detections. Although advanced tracking methods have been proposed recently, their application in maritime scenarios is limited. To address this gap, this study proposes a vision-guided object tracking framework for USVs, integrating state-of-the-art tracking algorithms with low-level control systems to enable precise tracking in dynamic maritime environments. We benchmarked the performance of seven distinct trackers, developed using advanced deep learning techniques such as Siamese Networks and Transformers, by evaluating them on both simulated and real-world maritime datasets. In addition, we evaluated the robustness of various control algorithms in conjunction with these tracking systems. The proposed framework was validated through simulations and real-world sea experiments, demonstrating its effectiveness in handling dynamic maritime conditions. The results show that SeqTrack, a Transformer-based tracker, performed best in adverse conditions, such as dust storms. Among the control algorithms evaluated, the linear quadratic regulator controller (LQR) demonstrated the most robust and smooth control, allowing for stable tracking of the USV. Videos and code can be found here: https://muhayyuddin.github.io/tracking/.

```cite
@ARTICLE{10848073,
  title={Benchmarking Vision-Based Object Tracking for USVs in Complex Maritime Environments},
  journal={IEEE Access}, 
  author={Muhayy, {Ud Din} and Ahsan, {Baidar Bakht} and Waseem, Akram and Yihao, Dong and Lakmal, Seneviratne and Irfan, Hussain},
  year={2025},
  volume={13},
  pages={15014-15027},
  doi={10.1109/ACCESS.2025.3532299}}

```

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
