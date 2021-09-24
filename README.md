# Launcher package
A package to lauch project related nodes for robot.

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
