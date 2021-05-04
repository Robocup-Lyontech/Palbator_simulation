# Palbator_simulation

## Authors
- Sai Kishor Kothakota

## Contributors
- Thomas Curé
- Simon Ernst

## Installation

Go to your workspace sources and execute the following commands :
```bash
git clone https://github.com/Robocup-Lyontech/Palbator_simulation.git 
cd ..
rosdep update
sudo rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys "pal_gazebo_plugins speed_limit_node sensor_to_cloud pmb2_rgbd_sensors pal_vo_server pal_karto pal_usb_utils pal_local_planner pal_filters hokuyo_node rrbot_launch robot_pose pal_pcl rviz_plugin_covariance pal-orbbec-openni2 slam_toolbox"
```
Do not hesitate to restart the last command if all the dependencies are not correctly installed.
Then, build the new modules :
```bash
catkin build
```

Start with:
```bash
roslaunch hsrb_wrs_gazebo_launch wrs_practice0_easy_tmc.launch public_sim:=true
```
wrs_practice0_easy_tmc.launch can be change for different object position

## launch arguments

You can optionally set following arguments when you launch the simulator:

- seed
- fast_physics
- highrtf
- mapping

"seed" is used to set random number seed to determine object placement. You can use this argument to test your algorithm with more variations.

If you set "fast_physics" to "true", the simulator will launch in fast physics mode.
Please note that in fast physics mode, we can enjoy faster simulation, on the other hand preciseness of the simulation will be reduced.

If "highrtf" is set to "true", the simulator will run in faster-than-realtime mode.

If "mapping" is set to "true", the simulator will run in mapping mode.

Each arguments can be used in combination as the follows:

```bash
$ roslaunch hsrb_wrs_gazebo_launch wrs_practice0_easy_tmc.launch seed:=10 fast_physics:=true highrtf:=true public_sim:=true
```



Based on PAL-Robotics PMB2 Simulation 
http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation
