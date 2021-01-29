# Palbator_simulation


# Palbator_simulation

## Authors
- Sai Kishor Kothakota

## Contributors
- Thomas Curé
- Simon Ernst

## Installation

Go to your workspace sources and execute the following commands :
```bash
git clone https://github.com/CureThomas/Palbator_simulation.git
cd ..
rosdep update
sudo rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys "pal_gazebo_plugins speed_limit_node sensor_to_cloud pmb2_rgbd_sensors pal_vo_server pal_karto pal_usb_utils pal_local_planner pal_filters hokuyo_node rrbot_launch robot_pose pal_pcl rviz_plugin_covariance pal-orbbec-openni2 slam_toolbox"
```
Do not hesitate to restart the last command if all the dependencies are not correctly installed.
Then, build the new modules :
```bash
catkin build
```



Based on PAL-Robotics PMB2 Simulation 
http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation
