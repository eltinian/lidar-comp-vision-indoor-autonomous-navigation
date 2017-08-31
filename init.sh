#!/bin/bash

cd src
catkin_init_workspace
cd ..
rosdep install autobot # installs dependencies required by autobot package
catkin_make
# source devel/setup.bash ## doesn't seem to work here. call this manually
echo "Initialization is done"
echo "Remember to run: source devel/setup.bash"
