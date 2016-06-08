 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Fanuc grinding
==============

# Travis CI

[![Travis-CI](https://api.travis-ci.org/InstitutMaupertuis/fanuc_grinding.svg?branch=indigo-devel)](https://travis-ci.org/InstitutMaupertuis/fanuc_grinding/branches)

This is a ROS package of a Fanuc R1000iA 80f with a grinding end effector and a 3D sensor mounted on the end effector.
This package aims to allow to automatically grind mechanical defects of parts given their CAD model.

![fanuc_grinding](documentation/fanuc_grinding.png)

The [documentation](documentation/README.md) folder contains explanations about how this package works.

Directories in the project
--------------------------

| Directory  | Description
------------ | -----------
`alignment` | Description/implementation of the alignment service
`comparison` | Description/implementation of the comparison service
`documentation` | Contains explication about this package usage
`execute_joint_state` | Description/implementation of a service to move the robot to a specific joint defined pose
`fanuc_grinding` | Contains the meta-package files
`grinding_rviz_plugin` | Contains the definition and implementation of the grinding RViz plugin
`path_planning` | Description/implementation of the path planning service. This makes use of the `bezier` package
`post_processor` | Description/implementation of the post processor service to create a Fanuc TP program. This makes use of the `fanuc_post_processor` package
`publish_meshfile` | Description/implementation of a service that publishes meshes/point clouds on markers
`scanning`| Description/implementation of the scanning service, the RViz plugin also allows to load a mesh/point cloud.

Dependencies
------------
- [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- [`industrial-core`](http://wiki.ros.org/industrial_core)
- [`fanuc`](http://wiki.ros.org/fanuc)
- [`fanuc experimental`](http://wiki.ros.org/fanuc_experimental)
- [`institut_maupertuis_robots_descriptions`](https://github.com/InstitutMaupertuis/institut_maupertuis_robots_descriptions)
- [`bezier`](https://github.com/ros-industrial-consortium/bezier)
- [`fanuc_post_processor`](https://github.com/InstitutMaupertuis/fanuc_post_processor)

This package has been tested with Ubuntu 14.04 and ROS Indigo.
The package was designed to be used with a Fanuc R1000iA-80f robot, however it should be easy to port it on other ROS compatible robots.

Install
-------
Install the dependencies by cloning the repositories into your catkin workspace.

`cd` to your catkin workspace source directory:
```
git clone https://github.com/InstitutMaupertuis/fanuc_grinding.git &&\
cd .. &&\
catkin build
```

Usage
-----
Simulation:
```
roslaunch grinding_rviz_plugin grinding_application_r1000ia.launch
```

Load a scan from the drive to skip the scanning part.

Real hardware:
```
roslaunch grinding_rviz_plugin grinding_application_r1000ia.launch \
sim:=false \
robot_ip:=192.168.100.200
```

Make sure to test the David SLS-2 configuration before testing with this application.

Documentaton
------------
It is [here](documentation).
