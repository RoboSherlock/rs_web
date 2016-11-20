#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>

class MSSegmenter {
public:
MSSegmenter();
void ms_segment_image(const cv::Mat &,cv::Mat &);

};
