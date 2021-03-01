# BoxDemo

## Basic requirements
```bash
python == 3.6
```

## Installation
### 1. numpy, scipy, opencv, numba, and open3d
```bash
$pip install -r requirements.txt
```
### 2. pcl
```bash
$pip install cython==0.26.0
```
```bash
$sudo apt-get install libpcl-dev pcl-tools
```

### 3. python-pcl
```bash
$git clone https://github.com/strawlab/python-pcl.git
$cd python-pcl
```

Before build install python-pcl, edit the setup.py to avoid possible conflicts:

1. line 728: change vtk_version = '7.0' to vtk_version = '6.3'

2. line 752: remove some unnecessary libs, including: vtkexpat, vtkfreetype, vtkgl2ps, vtkhdf5, vtkhdf5_hl, vtkjpeg, vtkjsoncpp, vtklibxml2, 
vtkNetCDF, vtkNetCDF_cxx, vtkoggtheora, vtkpng, vtkproj4, vtksqlite, vtktiff, vtkzlib

Then, install python-pcl
```bash
$python setup.py build_ext -i
$python setup.py install
```

## Usage
```bash
$python example_for_UNSTACK.py
```

## Main Parameters in config.py
```bash
PREPROCESS_THRESHOLD_DEFAULT (default = 30): tuning this value for detecting edges in the image
filter_ground_plane_para: the parameters of the ground plane. We can crop the point cloud of the ground and pre-compute the paras and save it.
dist2plane: the distance threshold for filtering points near a plane (within dist2plane)
camera_intrinsics: intrinsic paras of camera for converting between point cloud and the corresponding detpth/grey image
```

## Device requirements
```bash
>= GeForce RTX 2080 Ti 
```
