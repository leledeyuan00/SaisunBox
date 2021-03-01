export MAKEFLAGS=-j8

#export Python path
export PYTHONPATH=$PYTHONPATH:/home/jiang/saisun_ws/install/sensing/script/sensing/box

#export LD
export LD_LIBRARY_PATH=/home/jiang/saisun_ws/install/sensing/lib/sensing:$LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=/home/andylee/demo_time_fix_ws/install/grasping_platform/lib/grasping_platform:$LD_LIBRARY_PATH

#source ROS2
source /opt/ros/foxy/setup.bash

