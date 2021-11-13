#ifndef _DEPTH_IMAGE_SPLITTER_H_
#define _DEPTH_IMAGE_SPLITTER_H_

#include "reality_stream/depth_image_processor.h"

/**
 * @brief 
 * 
 */
class DepthImageSplitter : public DepthImageProcessor
{
    public:
        /**
         * @brief Construct a new Depth Image Splitter object
         * 
         */
        DepthImageSplitter();

        /**
         * @brief Destroy the Depth Image Splitter object
         * 
         */
        ~DepthImageSplitter();

        /**
         * @brief Redefine process function to add custom processing feature, in this function
         * the process is redefined to split the original depth image (16 bit resolution per pixel)
         * to 2 8 bit resolution per pixel. One for the first 8 upper bits and the other 8 lower bits.
         * 
         */
        void process();

        /**
         * @brief Respective getters for obtaining the splitted images
         * 
         * @return cv::Mat 
         */
        cv::Mat getLowerImage();
        cv::Mat getUpperImage();

    private:

        DepthImageDeque _lower_data; // respective data deque for storing the spitted data
        DepthImageDeque _upper_data;
};

#endif