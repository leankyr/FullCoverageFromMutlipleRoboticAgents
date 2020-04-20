# FullCoverageFromMutlipleRoboticAgents

My thesis repository.

As long as you have the willow garage world on the gazebo resource path and GAZEBO simulator which comes out of the box with ROS, gazebo_ros_pkgs, and the navigation stack you seem to be fine.

A python dependency is bresenham which can be installed through pip

how to execute:

1.0) clone master repo

2.0) open one terminal

2.1) type: `roslaunch FullCoverageFromMutlipleRoboticAgents amcl_one_to_rule_them_all.launch`

2.2) press Enter

3.0) open another terminal(second different from the one above)

3.1) type: `roslaunch FullCoverageFromMutlipleRoboticAgents target_select.launch`

3.2) press Enter

you will se the turtlebot trying to cover the whole map and setting goals on its own.
Soon more features to come...
