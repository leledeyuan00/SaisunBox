#include "sensing/BoxLocalizationAlgo.hpp"
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core/mat.hpp>
#include <Python.h>
#include <numpy/arrayobject.h>
#include <pcl_conversions/pcl_conversions.h>

PyObject* pyCreateImageArg(cv::Mat img, uchar* m);
PyObject* pyCreatePointCloudMonoArg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, float* m);
PyObject* pyCreatePointCloudColorArg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float* m);

BoxLocalizationAlgo::BoxLocalizationAlgo() {
    init();
    run_once_ = true;
}

BoxLocalizationAlgo::~BoxLocalizationAlgo() {
    Py_XDECREF(pyModule_);
    Py_XDECREF(pyFunc_);
}

void BoxLocalizationAlgo::config(RegionOfInterest roi) {
    roi_ = roi;
}

int BoxLocalizationAlgo::init(){
    Py_Initialize(); 
    PyEval_InitThreads();

    // this macro is defined be NumPy and must be included
    import_array();

    // add the current folder to the Python's PATH
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"/home/jiangxin/saisun_ws/install/sensing/script/sensing/box\")");

    // load our python script
    pyModule_ = PyImport_ImportModule("stack_detection"); 
    if (pyModule_ == NULL) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get python module failed!");
    }

    PyObject *pyDict = PyModule_GetDict(pyModule_);

    // grab the functions we are interested in
    pyFunc_ = PyDict_GetItemString(pyDict, "detect");
    if (pyFunc_ == NULL) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get python function failed!");
    }

    Py_XDECREF(pyDict);
}

bool BoxLocalizationAlgo::getObjectPose(PointCloudColor::Ptr cloud_ptr, cv::Mat color_img, 
    geometry_msgs::msg::Pose &pose, double &width, double &height) {
    

    if(!run_once_)
    {
        PyEval_ReleaseLock();
        run_once_ = true;
    }

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    float *cloud_m = NULL;
    PyObject* pCloud = pyCreatePointCloudColorArg(cloud_ptr, cloud_m);
    uchar *img_m = NULL;
    PyObject* pImag = pyCreateImageArg(color_img, img_m);
    PyObject* pMinZ =  PyFloat_FromDouble(roi_.z_offset * 1.0 /1000);
    PyObject* pMaxZ =  PyFloat_FromDouble((roi_.z_offset + roi_.depth) * 1.0 / 1000);
    PyObject *pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, pCloud); 
    PyTuple_SetItem(pArgs, 1, pImag);
    // PyTuple_SetItem(pArgs, 2, pMinZ);
    // PyTuple_SetItem(pArgs, 3, pMaxZ);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "z: [%f %f]", roi_.z_offset / 1000.0, (roi_.z_offset + roi_.depth)/1000.0);

    bool success = false;
    // execute the function
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    PyObject* pResult = PyEval_CallObject(pyFunc_, pArgs);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "<<<<<<<<<<<<<<<<<<<<<<<<<<<<-----------------------");
    // process the result
    if (pResult != NULL) 
    {
        
        PyArg_ParseTuple(pResult, "i|d|d|d|d|d|d|d|d|d", &success, &pose.position.x, &pose.position.y, &pose.position.z,
            &pose.orientation.x, &pose.orientation.y, &pose.orientation.z, &pose.orientation.w, &width, &height);	
        if(success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detected box pose!");
        } else{
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Detected nothing!");
        }

        Py_DECREF(pResult);

    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get python execution result failed!");
    }
    
    // decrement the object references

    Py_XDECREF(pCloud);
    //Py_XDECREF(pArgs);
    delete[] cloud_m;

    Py_XDECREF(pImag);
    delete[] img_m;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Python script finished!");
    PyGILState_Release(gstate);
    return success;
}


PyObject* pyCreateImageArg(cv::Mat img, uchar* m){
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pyCreateImageArg %d %d %d", img.rows, img.cols, img.channels());

	// total number of elements (here it's a grayscale 640x480)
	int nElem = img.rows * img.cols * img.channels();
	
	// create an array of apropriate datatype
	m = new uchar[nElem];
	
	// copy the data from the cv::Mat object into the array
	std::memcpy(m, img.data, nElem * sizeof(uchar));


	// the dimensions of the matrix
	npy_intp mdim[] = {img.rows, img.cols, img.channels()};
	
	// convert the cv::Mat to numpy.array
	PyObject* mat = PyArray_SimpleNewFromData(3, mdim, NPY_UINT8, (void*)m);
  
	return mat;
}

PyObject* pyCreatePointCloudColorArg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float* m){
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pyCreatePointCloudColorArg point size %d", cloud_ptr->size());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[1000000] %f %f %f %d %d %d %d %f", cloud_ptr->points[1000000].x, cloud_ptr->points[1000000].y, cloud_ptr->points[1000000].z, cloud_ptr->points[1000000].r,
        cloud_ptr->points[1000000].g, cloud_ptr->points[1000000].b, cloud_ptr->points[1000000].a, cloud_ptr->points[1000000].rgb);
	int nElem = 8 * cloud_ptr->size();
	// create an array of apropriate datatype
	m = new float[nElem];

	// copy the data from the cv::Mat object into the array
	std::memcpy(m, &cloud_ptr->points[0], nElem * sizeof(float));

	// the dimensions of the matrix
	npy_intp mdim[] = {cloud_ptr->size(), 8};
	
	// convert the cv::Mat to numpy.array
	PyObject* mat = PyArray_SimpleNewFromData(2, mdim, NPY_FLOAT32, (void*)m);

	return mat;
}

PyObject* pyCreatePointCloudMonoArg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, float* m){
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pyCreatePointCloudMonoArg point size %d", cloud_ptr->size());

	int nElem = 4 * cloud_ptr->size();
	// create an array of apropriate datatype
	m = new float[nElem];

	// copy the data from the cv::Mat object into the array
	std::memcpy(m, &cloud_ptr->points[0], nElem * sizeof(float));

	// the dimensions of the matrix
	npy_intp mdim[] = {cloud_ptr->size(), 4};
	
	// convert the cv::Mat to numpy.array
	PyObject* mat = PyArray_SimpleNewFromData(2, mdim, NPY_FLOAT32, (void*)m);

	return mat;
}
