# my_2d_nav

My thesis repository.

As long as you have the willow garage world on the gazebo resource path and GAZEBO simulator which comes out of the box with ROS, gazebo_ros_pkgs, and the navigation stack you seem to be fine... However there may be some dependencies I have yet to figure out.

A python dependency is bresenham which can be installed through pip

how to execute:

>>>>>>> d846ca27b1e9b41eaebd0ec274404148a0d3ad34
1.0) clone master repo

2.0) open one terminal

2.1) type: `roslaunch my_2d_nav amcl_one_to_rule_them_all.launch`

3.0) open another terminal(second different from the one above)

3.1) type: `roslaunch my_2d_nav target_select.launch`

you will se the turtlebot trying to cover the whole map and setting goals on its own.
Soon more features to come.
