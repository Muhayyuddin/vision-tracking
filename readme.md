Detailed instructions will be updated soon.. 

## Dependencies

```
pip install transforms3d
sudo apt install ros-galactic-robot-localization 
sudo apt install ros-galactic-xacro 
sudo apt install ros-galactic-joint-state-publisher
```
```

run the flowing commands in seperate terminals 
```

```
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"
```
```
ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  slot0:=mbzirc_rgbd_camera   slot0_rpy:="0 -15 0" 


slot1:=mbzirc_planar_lidar 

ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv type:=usv x:=15 y:=0 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_rgbd_camera   slot0_rpy:="0 -14 0" 
```
```
export PYTHONPATH=$PYTHONPATH:~/mbzirc_ws/src/nav_packages/usv_control/src
export PYTHONPATH=$PYTHONPATH:~/portsecurity_ws/src/inspection/mbzirc/usv_control/src
export PYTHONPATH=$PYTHONPATH:~/mbzirc_ws/src/marine-nav-ontologies/usv_control/src

ros2 run usv_control twist_publisher 
```
```
ros2 run navigation navigate 
```
```
ros2 launch usv_description usv.launch.py
```
```
ros2 run custom_tf_broadcaster publish_odometry_and_tf 
```
