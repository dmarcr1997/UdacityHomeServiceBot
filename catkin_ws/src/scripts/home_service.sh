#!/bin/sh
xterm -e "roslaunch my_robot world.launch" &
sleep 5
xterm -e "roslaunch my_robot view_navigation.launch" &
sleep 5
xterm -e "roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
echo "DONE!!"