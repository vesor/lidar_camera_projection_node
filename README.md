# lidar to camera projection

This app project the lidar points onto camera images.

You need to provide a rough estimate for the initial Rt matrix, then fine tune it manually using this app. 

## Hotkeys

Change rotation: w/s e/d r/f

Change translation: u/j i/k o/l

Print Rt matrix: Enter

## how to build

This is a ROS node, put it in some catkin_ws/src folder and use catkin_make to build.





