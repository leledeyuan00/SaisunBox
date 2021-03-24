export MAKEFLAGS=-j8

#export Python path
export PYTHONPATH=$PYTHONPATH:home/jiangxin/saisun_ws/install/sensing/script/sensing/box

#export LD
export LD_LIBRARY_PATH=home/jiangxin/saisun_ws/install/sensing/lib/sensing:$LD_LIBRARY_PATH

#source ROS2
source /opt/ros/foxy/setup.bash

