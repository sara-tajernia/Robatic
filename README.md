# Robotics Coursework with ROS

This repository collects reports and source snapshots from a robotics course. The exercises use ROS Python nodes, TurtleBot3/Gazebo simulations, custom messages and services, odometry, laser scans, path tracking, and PID-style motion control.

## Repository guide

| Path | Contents |
| --- | --- |
| [`robot1/`](robot1/) | A publisher generates random student records, a splitter routes them by department, and two subscribers print the routed messages. The custom message definition is [`Stu.msg`](robot1/Codes/msg/Stu.msg). |
| [`robot2/`](robot2/) | Three path-following controller experiments, a path monitor, launch files, and the second assignment report. The controllers define rectangular, spiral, circular, and eight-sided reference paths. |
| [`robot3/Codes/s1/`](robot3/Codes/s1/) | Obstacle-distance and direction experiments with a custom message and service, plus a Gazebo world and launch files. |
| [`robot3/Codes/s2/`](robot3/Codes/s2/) | Wall following, maze following, and goal-seeking experiments using laser scans, odometry, and velocity commands. |
| [`robot_project/`](robot_project/) | The final mapping/navigation assignment brief, report, and a `map_gapping` ROS package snapshot. |

The assignment documentation is included in the PDF reports beside each exercise. Most report text is in Persian.

## Technologies represented

- ROS and catkin
- Python ROS nodes using `rospy`
- ROS messages from `geometry_msgs`, `nav_msgs`, and `sensor_msgs`
- TF quaternion-to-Euler conversion
- TurtleBot3 and Gazebo launch/world files
- NumPy and Matplotlib for reference paths and error plots

No dependency lock file or ROS distribution is recorded in the repository, so no specific version is asserted here.

## Using the source snapshots

The folders are independent coursework snapshots rather than one installable workspace. Read the report for the selected exercise, inspect its `package.xml`, `CMakeLists.txt`, and launch file, then place that package in a compatible catkin workspace together with its referenced ROS, TurtleBot3, and Gazebo packages.

The following limitations are present in the current tree:

- `robot1/Codes` contains the custom message and nodes but no tracked catkin manifest/build file for the `rand_stu` package referenced by the imports.
- Several controller and obstacle scripts contain unresolved syntax, import, or variable-name errors and should be reviewed before execution.
- Some launch files refer to package names or world files that do not match the directory containing them.
- No automated tests or reproducible environment definition are included.

For those reasons, the repository should currently be treated as an archive of reports and implementation work, not as a verified one-command ROS package.
