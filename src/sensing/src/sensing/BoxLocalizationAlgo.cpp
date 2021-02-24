#include "sensing/BoxLocalizationAlgo.hpp"
#include <numpy/arrayobject.h>

PyObject* createArg(PointCloudColor::Ptr cloud_ptr, float* m);

BoxLocalizationAlgo::BoxLocalizationAlgo() {
    init();
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

    // this macro is defined be NumPy and must be included
    import_array();

    // add the current folder to the Python's PATH
    //PyRun_SimpleString("import sys");
    //PyRun_SimpleString("sys.path.append(\"/home/conicacui/dev_ws/src/hkclr_smart_grasping_platform/grasping_platform/script/box\")");

    // load our python script
    pyModule_ = PyImport_ImportModule("detect_interface"); 
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

bool BoxLocalizationAlgo::getObjectPose(PointCloudColor::Ptr cloud_ptr, geometry_msgs::msg::Pose &pose,
    double &width, double &height) {
    float *cloud_m = NULL;
    PyObject* pCloud = createArg(cloud_ptr, cloud_m);
    PyObject* pMinZ =  PyFloat_FromDouble(roi_.z_offset * 1.0 /1000);
    PyObject* pMaxZ =  PyFloat_FromDouble((roi_.z_offset + roi_.depth) * 1.0 / 1000);
    PyObject* pWidth =  PyFloat_FromDouble(cloud_ptr->width);
    PyObject* pHeight =  PyFloat_FromDouble(cloud_ptr->height);
    PyObject *pArgs = PyTuple_New(5);
    PyTuple_SetItem(pArgs, 0, pCloud); 
    PyTuple_SetItem(pArgs, 1, pMinZ);
    PyTuple_SetItem(pArgs, 2, pMaxZ);
    PyTuple_SetItem(pArgs, 3, pWidth);
    PyTuple_SetItem(pArgs, 4, pHeight);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "z: [%f %f]", roi_.z_offset / 1000.0, (roi_.z_offset + roi_.depth)/1000.0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size: [%d %d]", cloud_ptr->width, cloud_ptr->height);

    // execute the function
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    PyObject* pResult = PyEval_CallObject(pyFunc_, pArgs);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "<<<<<<<<<<<<<<<<<<<<<<<<<<<<-----------------------");
    // process the result
    if (pResult != NULL) 
    {
        PyArg_ParseTuple(pResult, "d|d|d|d|d|d|d|d|d", &pose.position.x, &pose.position.y, &pose.position.z,
            &pose.orientation.x, &pose.orientation.y, &pose.orientation.z, &pose.orientation.w, &width, &height);	
        Py_DECREF(pResult);

    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get python execution result failed!");
    }
    
    // decrement the object references

    Py_XDECREF(pCloud);
    //Py_XDECREF(pArgs);
    delete[] cloud_m;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Python script finished!");
    return true;
}

PyObject* createArg(PointCloudColor::Ptr cloud_ptr, float* m){
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