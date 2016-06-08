Goals
=====
Design a fully automated robotized grinding application

Functions
- F1: Being able to scan the part with SLS-2
- F2: Ensure a constant material removal rate (CAM)
- F3: Being able to respect the manufacturer's specifications (dimensions, manage tool wear)

Automatically grind defects on a mechanical part thanks to:
- A robot
- 3D vision

Defects types
- Shocks, deformations, welding spatters, weld bead ...
- Surface roughness


How this plugin works
=====================

1. Scanning 3D
--------------

With first 'Browse' button, you can import a CAD file, and visualize it if button 'Import CAD file' is pressed.
With second 'Browse' button, you can import a YAML file which contains a list of joint values for robot.
When 'Start scan' button is pressed, the YAML file will be parsed and the joint values will be executed in order to numerize the piece.

| Parameters  | Description
------------- | -----------
`CAD file` | Location of the CAD file in the computer
`YAML file` | Location of the YAML file in the computer

2. Alignment
------------
# :construction: In Construction! :construction: 

This tab will make alignment between CAD file ans the point cloud resulting from scan 3D. It will also make alignment between these two files and the robot.

3. Comparison
-------------
# :construction: In Construction! :construction:

This tab will compare the CAD file and the point cloud resulting from scan 3D.

4. Path planning
----------------
# :construction: In Construction! :construction:

The path planning will be generated with Bezier planner.
Both labels on the top are used to indicate with which files we are dealing with ( CAD file and scan 3d )
This tab has to be filled with these Bezier's parameters:

| Parameters  | Description
------------- | -----------
`Covering percentage` | Percentage of covering (decimal value)
`Extrication frequency` | New extrication mesh generated each 1/`Extrication frequency` times
`Extrication coefficient` | Extrication depth equal of the percentage of `Depth of path`
`Grind diameter` | Width of the tool which is used
`Depth of path` | Grinding depth
`Axis of rotation` | Rotation axis for the lean angle of the effector
`Angle value` | Value around `Axis of rotation`

When 'Compute trajectories' button is pressed, Bezier algorithm will compute trajectories with parameters given.
Buttons 'Visualize trajectory' and 'Simulate trajectory' are respectivly visualize trajectories in each pass and simulate robot positions during grinding process on each path.
Warning: A warning will appear if parameters given in Qt have been changed! You will have to press again on 'Compute trajectory' button if you want to compute trajectories with the new parameters, otherwise the trajectory will be the old one!

5. Post processor
-----------------
# :construction: In Construction! :construction:

This tab is used to convert path planning generated before in TP/LS Fanuc program.
So we have to fill different parameters to do it:

| Parameters  | Description
------------- | -----------
`Program name` | Name for this program
`Location` | Location of the program in the computer
`Comment` | To fill in order to give useful information in program as comment.
`IP adress` | Robot's IP adress

Hardware needed
---------------
Generally speaking, this is what you need to get this package running on a real application case:
- A computer with ROS installed
- A 6 axis robot supported by ROS
- A grinding effector
- A 3D sensor (attached to the robot) supported by ROS
- A part to be grinded (you need the CAD of this part)

In this application case, here is a list of the hardware:
- A computer with ROS installed
- A 6 axis Fanuc R1000iA robot with ROS installed
- A grinding effector
- A David SLS-2 3D sensor (attached to the robot) and a Windows PC acting as David software server
- A part to be grinded and it's CAD

