# BoxDemo

## Basic requirements
- Ubuntu 20.04
- ROS2 Foxy
- Python3
  
## Prerequisites
### 1. numpy, scipy, opencv, numba, and open3d
```
pip3 install -r requirements.txt
```

### 2. pcl
```bash
pip3 install cython
sudo apt-get install libpcl-dev pcl-tools
```

### 3. python-pcl
```
git clone https://github.com/conica-cui/python-pcl.git
cd python-pcl
python3 setup.py build_ext -i
python3 setup.py install
```


## Build package
```
mkdir box_ws
cd box_ws
git clone $repo-url
colcon build
```

## Validate sensing script
```
python3 src/sensing/script/box/example_unstack.py
python3 src/sensing/script/box/detect_interface.py  


# Change the model.yml path in config.py to "install/sensing/script/sensing/box/model.yml"
# By default, these two scripts loaded point cloud from ./data/ply_0.ply, change the path if you want to test other files.
# If success, the first script should pop up visualization window with detection result, the seconds script will only print the detected pose and plane size. 
```



## Run ros2 node
```
source setup.bash

cd ~/WorkSpace

colcon build

. /install/setup.bash

ros2 run vision_node
```



## Setup Real Camera

1. Do calibration
2. Copy the camera configuration folder "config" to current workspace
3. Adjust the RegionOfInterest of the camera. Ref: test.cpp line 8-14
4. Switch to use SMARTEYE_HV1000. Ref: test.cpp line 16

```
    server.config(CAMERALMODEL::SMARTEYE_HV1000, roi);
```

## Main Parameters in config.py
```
PREPROCESS_THRESHOLD_DEFAULT (default = 30): tuning this value for detecting edges in the image
filter_ground_plane_para: the parameters of the ground plane. We can crop the point cloud of the ground and pre-compute the paras and save it.
dist2plane: the distance threshold for filtering points near a plane (within dist2plane)
camera_intrinsics: intrinsic paras of camera for converting between point cloud and the corresponding detpth/grey image
```

## Device requirements
```
>= GeForce RTX 2080 Ti 
```

## Questions & Solutions ###
1. module 'cv2.cv2' has no attribute 'ximgproc'

```
pip3 uninstall opencv-contrib-python opencv-python
pip3 install opencv-contrib-python
```  
