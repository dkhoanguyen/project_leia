#include "ros/ros.h"
#include "image_transport/image_transport.h"

#include "reality_stream/depth_image_splitter.h"
#include "reality_stream/depth_image_processor.h"
#include "reality_stream/ros_depth_image_splitter.h"

int main(int argc, char **argv)
{
    // start splitter
    std::shared_ptr<DepthImageSplitter> depth_splitter(new DepthImageSplitter());
    ros::init(argc, argv, "reality_stream_node");
    ros::NodeHandle nh;

    std::vector<std::shared_ptr<DepthImageSplitter>> depth_splitter_list;
    std::vector<std::shared_ptr<RosDepthImageSplitter>> ros_splitter_list;

    int num_cam = 0;
    nh.getParam("/reality_stream_node/num_cam", num_cam);
    for(int cam_id = 1; cam_id <= num_cam; cam_id++)
    {
        std::vector<std::string> pub_list,sub_list;
        std::string param_path;
        std::string sub_path,lower_pub_path,upper_pub_path;

        param_path = "/reality_stream_node/cam_" + std::to_string(cam_id) + "/sub_path";
        nh.getParam(param_path,sub_list);

        param_path = "/reality_stream_node/cam_" + std::to_string(cam_id) + "/pub_path";
        nh.getParam(param_path,pub_list);

        sub_path = sub_list.at(0);
        for(auto path : pub_list)
        {
            if(path.find("lower") != std::string::npos)
            {
                lower_pub_path = path;
                continue;
            }
            if(path.find("upper") != std::string::npos)
            {
                upper_pub_path = path;
                continue;
            }
        }

        std::shared_ptr<DepthImageSplitter> depth_splitter(new DepthImageSplitter());
        std::shared_ptr<RosDepthImageSplitter> ros_splitter(new RosDepthImageSplitter(nh,
                                                                                      sub_path,
                                                                                      lower_pub_path,
                                                                                      upper_pub_path));
        ros_splitter->init();
        ros_splitter->setProcessor(depth_splitter);

        depth_splitter_list.push_back(depth_splitter);
        ros_splitter_list.push_back(ros_splitter);

    }
    // kick depth_splitter
    for(auto ros_splitter : ros_splitter_list)
    {
        ros_splitter->run();
    }
    
    // set rate
    ros::Rate rate(30);

    while(ros::ok())
    {
        for(auto ros_splitter : ros_splitter_list)
        {
            ros_splitter->publishProcessedData();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}