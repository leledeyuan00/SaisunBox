export MAKEFLAGS=-j8

#export Python path
export PYTHONPATH=$PYTHONPATH:/home/conicacui/depallet_ws/install/sensing/script/sensing/box:/home/conicacui/python-pcl

#export LD
export LD_LIBRARY_PATH=/home/conicacui/depallet_ws/install/sensing/lib/sensing:$LD_LIBRARY_PATH

#source ROS2
source /opt/ros/foxy/setup.bash

