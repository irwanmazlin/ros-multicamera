// #include <gflags/gflags.h>
// #include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
// #include <iostream>
// #include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/bind.hpp>

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <ros_vino/Object.h>
// #include <ros_vino/Objects.h>

// #include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <inference_engine.hpp>
// #include <samples/common.hpp>
// #include <samples/ocv_common.hpp>



#include <ros/ros.h>
#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <detecttrack/Object.h>
#include <detecttrack/Objects.h>
#include <detecttrack/ObjectBox.h>
#include <detecttrack/ObjectBoxList.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
// #ifdef WITH_EXTENSIONS
//     #include <ext_list.hpp>
// #endif

using namespace InferenceEngine;

class ObjectDetectionSSD
{

public:

    // ObjectDetectionSSD();

    ~ObjectDetectionSSD();

    void run();

private:

    // void initInferenceEngine();

    bool getParameters(ros::NodeHandle n);

    void initROSInterface();
    void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    void depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg);

    void rosCallbackImage(const sensor_msgs::Image::ConstPtr& image_msg);
    void frame_to_blob(const cv::Mat& image,InferRequest::Ptr& analysis,const std::string& descriptor);


    void frameToBlob(const cv::Mat& frame, InferRequest::Ptr& inferRequest, const std::string& inputName);

    std_msgs::Header getHeader();


    // **************************
    // ROS INTERFACE
    // **************************
    ros::NodeHandle         n;
    // ros::NodeHandle         nh_private;

    ros::Subscriber         sub_image_rgb;
    ros::Publisher          pub_image_rects;
    ros::Publisher          pub_objects;
    ros::Publisher result_pub;
    ros::Publisher image_pub;
    ros::Publisher marker_pub;
    ros::Publisher boxlist_pub;
    ros::Subscriber camerainfo_sub;
    ros::Subscriber depth_sub; 


    std::string             image_input;
    std::string             image_output;
    std::string             objects_output;

    // **************************
    // IE VARIABLES
    // **************************
    std::string device;
    float confidence_threshold;
    std::string network_path;
    std::string weights_path;
    std::string labels_path;
    std::string colors_path;
    bool output_as_image;
    bool output_as_list;
    bool depth_analysis;
    bool output_markers;
    bool output_markerslabel;
    std::string depth_frameid;
    float markerduration;
    bool output_boxlist;


    //ROS messages
    sensor_msgs::Image output_image_msg;
    detecttrack::Object tmp_object;
    detecttrack::Objects results_list;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_label;
    visualization_msgs::MarkerArray markers;
    detecttrack::ObjectBox tmp_box;
    detecttrack::ObjectBoxList box_list;

    Core                    ie;


    bool                    bool_auto_resize;
    bool                    bool_pc;
    bool                    bool_raw;

    // **************************
    // Application Variables
    // **************************
    InferRequest::Ptr       async_infer_request_curr;
    InferRequest::Ptr       async_infer_request_next;
    InferRequest::Ptr       engine_curr;
    InferRequest::Ptr       engine_next;

    std::string             inputName;
    std::string             imageInfoInputName;
    std::string             outputName;



    SizeVector              outputDims;
    int                     maxProposalCount;
    int                     objectSize;


    int results_number;
    int object_size;


    // DataPtr& output_data;
    std::vector<std::string> labels;

    std::vector<std::string> vector_labels;
    // std::ifstream inputFileLabel(labels_path);
    // std::copy(std::istream_iterator<std::string>(inputFileLabel),std::istream_iterator<std::string>(),std::back_inserter(vector_labels));
    std::vector<std::string> vector_colors;
    // std::ifstream inputFileColor(colors_path);

    cv::Mat frame_now;  
    cv::Mat frame_next;
    cv::Mat depth_frame; 




    cv::Mat                 image;
    cv::Mat                 curr_frame;
    cv::Mat                 next_frame;

    size_t                  width;
    size_t                  height;

        //Frames sizes
    size_t color_width;
    size_t color_height;
    size_t depth_width;
    size_t depth_height;

    bool                    is_new_image;
    bool                    is_first_image;


    //Parameters for camera calibration
    float fx;
    float fy;
    float cx;
    float cy;


    //lockers
    bool cf_available=false;
    bool is_last_frame=true;

};