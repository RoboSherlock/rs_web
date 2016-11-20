#include <rs_queryanswering/MSSegmenter.h>

MSSegmenter::MSSegmenter(){

}
void MSSegmenter::ms_segment_image(const cv::Mat &imIn, cv::Mat &imOut){

cv::ocl::oclMat oclImgRaw, oclImgConverted;
oclImgRaw.upload(imIn);
cv::ocl::cvtColor(oclImgRaw, oclImgConverted, CV_BGR2BGRA);
cv::ocl::meanShiftSegmentation(oclImgConverted, imOut ,30,30,(imIn.rows*imIn.cols)*0.01);


}
