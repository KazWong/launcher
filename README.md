# Launcher package
A package to lauch project related nodes for robot.

## How to Use

launcher is package to call the boot.launch in ghost package and launch other nodes.  

There are three main parameter for launching a robot.
1. robot (name of the robot urdf in config_robot package)
2. param (name of the navigation param in config_robot package)
3. map/world (name of the map or world in config_env package)

To create new launch file, for example, in robot.launch, there are three main parts:

1. Load the args:
```
<!-- BOOT -->
<arg name="ns" default=""/>
<arg name="robot" default="robot"/>
<arg name="param" default="eband_1"/>
<arg name="map" default="env7"/>
<arg name="amcl_map" default=""/>
```

2. Call boot.launch:
```
<include file="$(find ghost)/launch/boot.launch">
  <arg name="ns" value="$(arg ns)"/>
  <arg name="param" value="$(arg param)"/>
  <arg name="tf" value="true"/>
  <arg name="robot" value="$(arg robot)"/>
  <arg name="navigation" value="true"/>
  <arg name="map_name" value="$(arg map)"/>
  <arg name="amcl_map_name" value="$(arg amcl_map)"/>
  <arg name="manual" value="false"/>
  <arg name="joystick" value="false"/>
  <arg name="rosbag" value="false"/>
  <arg name="duration" value="5m"/>
  <arg name="split" value="100"/>
</include>
```

3. Call other nodes:
```
<!-- Drive -->
<include file="$(find drive)/launch/slam.launch"/>
<!-- Velodyne lidar -->
<include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
  <arg name="device_ip" value="192.168.1.201" />
  <arg name="max_range" value="13.0" />
  <arg name="laserscan_resolution" value="0.0087" />
</include>
<!-- velodyne tf -->
<node pkg="tf" type="static_transform_publisher" name="agv_lidar2velodyne_publisher"
  args="0 0 0 0 0 0 agv_lidar velodyne 10"/>
<!-- data transformer -->
<node pkg="data_transformer" type="data_transformer2" name="data_transformer2"/>
```

## Use of ROS
There are three main ROS workspaces in a robot.
1. ROS main distro workspace (/opt/ros/[distro])
2. Common workspace (general packages)
3. Project workspace (hardware depended / tasks)

| Distro Workspace        |      | Common Workspace        |      | Project Workspace  |  
| -----------             |      | -----------             |      | -------            |  
|                         | <->  | ghost                   | <->  | launcher           |  
|                         |      |                         |      | config_robot       |  
|                         |      |                         |      | config_env         |  
|                         |      |                         |      |                    |  
|                         |      | navigation              |      | tasks              |  

ROS distro workspace is a workspace for original packages from the internet.  
Common workspace is a prebuild workspace which install to the robot.  
Project workspace is the development workspace for each project or robot.

Common workspace source distro workspace to build, project workspace source common workspace to build.
