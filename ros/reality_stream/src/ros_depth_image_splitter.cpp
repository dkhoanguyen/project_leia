#include "reality_stream/ros_depth_image_splitter.h"

RosDepthImageSplitter::RosDepthImageSplitter(ros::NodeHandle nh,
                                             std::string sub_path,
                                             std::string lower_pub_path,
                                             std::string upper_pub_path) 
    : _nh(nh),_it(nh)
{
    _sub_path = sub_path;
    _lower_pub_path = lower_pub_path;
    _upper_pub_path = upper_pub_path;
}

RosDepthImageSplitter::~RosDepthImageSplitter()
{

}

void RosDepthImageSplitter::init()
{
    _raw_data_sub = _it.subscribe(_sub_path,10,&RosDepthImageSplitter::depthImageCallback,this);
    _lower_depth_pub = _it.advertise(_lower_pub_path,1);
    _upper_depth_pub = _it.advertise(_upper_pub_path,1);
}

void RosDepthImageSplitter::setProcessor(const std::shared_ptr<DepthImageSplitter> &processor)
{
    _processor = processor;
}

void RosDepthImageSplitter::run()
{
    _processor->start();
}

void RosDepthImageSplitter::depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    try
    {
        _cv_ptr_depth = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
        _processor->setImage(_cv_ptr_depth->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RosDepthImageSplitter::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_msg)
{
    _depth_img_info.update(*camera_msg);
}

void RosDepthImageSplitter::publishProcessedData()
{
    cv::Mat image = _processor->getLowerImage();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    _lower_depth_pub.publish(msg);

    image = _processor->getUpperImage();   
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    _upper_depth_pub.publish(msg);
}
