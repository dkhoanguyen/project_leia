#ifndef _ROS_DEPTH_IMAGE_PROCESSOR_H_
#define _ROS_DEPTH_IMAGE_PROCESSOR_H_

#include <vector>
#include <map>
#include <thread>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "reality_stream/depth_image_processor.h"
#include "reality_stream/depth_image_splitter.h"

/**
 * @brief 
 * 
 */
struct DepthCameraInfo
{
    public:
        void update(sensor_msgs::CameraInfo cam_info)
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _cam_info = cam_info;
            _available = true;
        };
        
        sensor_msgs::CameraInfo get()
        {
            std::unique_lock<std::mutex> lock(_mtx);
            return _cam_info;
        };

        bool available()
        {
            return (bool)_available;
        };

    private:
        sensor_msgs::CameraInfo _cam_info;
        std::mutex _mtx;
        std::atomic<bool> _available{false};

};

/**
 * @brief 
 * 
 */
class RosDepthImageSplitter
{
    public:

        RosDepthImageSplitter(ros::NodeHandle nh,
                              std::string sub_path,
                              std::string lower_pub_path,
                              std::string upper_pub_path);
        ~RosDepthImageSplitter();
        void init();

        void setProcessor(const std::shared_ptr<DepthImageSplitter> &processor);
        void run();

        void publishProcessedData();

    protected: // Ros related

        ros::NodeHandle _nh;
        ros::Subscriber _raw_data_info_sub;

        image_transport::ImageTransport _it;
        image_transport::Subscriber _raw_data_sub;
        image_transport::Publisher _lower_depth_pub;
        image_transport::Publisher _upper_depth_pub;

        cv_bridge::CvImagePtr _cv_ptr_depth;
        
        std::string _sub_path;
        std::string _lower_pub_path;
        std::string _upper_pub_path;

        void depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg);
        void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_msg);

        DepthCameraInfo _depth_img_info;

    protected: // Non-ros related
    
        std::shared_ptr<DepthImageSplitter> _processor;
    
};

#endif