#ifndef _DEPTH_IMAGE_PROCESSOR_H_
#define _DEPTH_IMAGE_PROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#define DEFAULT_MAX_SIZE 10

/**
 * @brief Struct for storing depth image data, with guarantee thread safe and data push pop features
 * 
 */
struct DepthImageDeque
{
    public:
        /**
         * @brief Construct a new Depth Image Deque with a custom size
         * 
         * @param max_size 
         */
        explicit DepthImageDeque(unsigned int max_size):_max_size(max_size){};

        /**
         * @brief Default constructor with default max size
         * 
         */
        DepthImageDeque():_max_size(DEFAULT_MAX_SIZE){};

        /**
         * @brief We need a destructor here since we have declared a constructor.
         * In the future, if the code needs to be optimised further, by introducing
         * move semantic, this should come in handy.
         * 
         */
        ~DepthImageDeque(){};

        /**
         * @brief Setter to update the data deque with a new value, while also checks
         * remove abundant old data that overflows a defined max size
         * 
         * @param target 
         */
        void add(cv::Mat &target)
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _data.push_back(target);
            if(_data.size() > _max_size)
            {
                _data.pop_front();
            }
            _received = true;
        }

        /**
         * @brief Get the latest data point in the deque
         * 
         * @return cv::Mat 
         */
        cv::Mat get()
        {
            std::unique_lock<std::mutex> data_lock(_mtx);
            if(!_data.empty())
            {
                return _data.back();
            }
            else
            {
                // Prevent seg fault here. Can modify error output to provide more info
                std::cerr << "No data"<< std::endl;
                return cv::Mat(1,1,CV_8UC1);
            }
        }

        unsigned int size()
        {
            return _data.size();
        }

        bool ready()
        {
            return (bool)_received;
        }

    private:
        unsigned int _max_size;
        std::deque<cv::Mat> _data;
        std::mutex _mtx;
        std::atomic<bool> _received{false};
}; 
/**
 * @brief Abstract class for processing depth images. Derive class can be constructed from this class
 * by simply redefine the process function
 * 
 */
class DepthImageProcessor
{
    public:

        DepthImageProcessor();
        virtual ~DepthImageProcessor();
        
        /**
         * @brief Function that starts the processing thread
         * 
         */
        void start();

        /**
         * @brief Set the target image for processing
         * 
         */
        void setImage(cv::Mat image);

        /**
         * @brief Get the processed image
         * 
         */
        cv::Mat getProcessedImage();

        /**
         * @brief Pure virtual function for 
         * 
         */
        virtual void process() = 0;

    protected: 

        std::thread *_process_thread;       // This should be a smart pointer - will need to change later
        DepthImageDeque _raw_data;

    private:

        // Simple mutex and condvars for simulating events
        std::mutex _process_cv_mtx;
        std::condition_variable _process_cv;
        bool _thread_started = false;

        void run();
};

#endif