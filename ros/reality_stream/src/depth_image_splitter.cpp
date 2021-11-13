#include "reality_stream/depth_image_splitter.h"

DepthImageSplitter::DepthImageSplitter()
{

}

DepthImageSplitter::~DepthImageSplitter()
{
    
}

cv::Mat DepthImageSplitter::getLowerImage()
{
    return _lower_data.get();
}

cv::Mat DepthImageSplitter::getUpperImage()
{
    return _upper_data.get();
}

void DepthImageSplitter::process()
{
    cv::Mat raw_depth = _raw_data.get();
    int height = raw_depth.rows;
    int width = raw_depth.cols;

    cv::Mat image_lower(height, width, CV_8UC1);
    cv::Mat image_upper(height, width, CV_8UC1);

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            image_lower.at<uchar>(i, j) = static_cast<uchar>(raw_depth.at<unsigned short>(i, j) / 256);
            image_upper.at<uchar>(i, j) = static_cast<uchar>(raw_depth.at<unsigned short>(i, j) - (((unsigned short)(raw_depth.at<unsigned short>(i, j) / 256)) * 256));
        }
    }

    _lower_data.add(image_lower);
    _upper_data.add(image_upper);
}