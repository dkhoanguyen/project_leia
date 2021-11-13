#include "reality_stream/depth_image_processor.h"

DepthImageProcessor::DepthImageProcessor()
{
    
}

DepthImageProcessor::~DepthImageProcessor()
{
    if(_thread_started)
    {
        _process_thread->join();
    }
}

void DepthImageProcessor::start()
{
    _process_thread = new std::thread(&DepthImageProcessor::run,this);
    _thread_started = true;
}

void DepthImageProcessor::setImage(cv::Mat image)
{
    _raw_data.add(image);
    if(_thread_started)
    {
        _process_cv.notify_all();
    }
}

cv::Mat DepthImageProcessor::getProcessedImage()
{
    return _raw_data.get();
}

void DepthImageProcessor::run()
{
    std::unique_lock<std::mutex> temp_lock(_process_cv_mtx);
    while(true)
    {
        // Wait to be notified by setImage -> synchronisation with new data
        _process_cv.wait(temp_lock);
        process();
    }
}
