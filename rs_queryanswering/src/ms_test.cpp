#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <string>
#include <stdio.h>
#include <rs_queryanswering/MSSegmenter.h>

#ifdef _DEBUG
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")   //MAT processing
#pragma comment(lib, "opencv_gpu249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#else
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_gpu249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#endif

using namespace cv;
using namespace std;


void ProccTimePrint( unsigned long Atime , string msg)
{
 unsigned long Btime=0;
 float sec, fps;
 Btime = getTickCount();
 sec = (Btime - Atime)/getTickFrequency();
 fps = 1/sec;
 printf("%s %.4lf(sec) / %.4lf(fps) \n", msg.c_str(),  sec, fps );
}


int main(int argc, char** argv)
{
 unsigned long AAtime=0;

 //image load
 Mat img = imread("googlestest.jpg");
 Mat outImg, outImgCuda, outImgOcl;

 //cpu version meanshift
// AAtime = getTickCount();
// pyrMeanShiftFiltering(img, outImg, 30, 30, 3);
// ProccTimePrint(AAtime , "cpu");

 //ocl version MeanShift
// ocl::oclMat oclImgRaw,oclImgConverted;
 AAtime = getTickCount();
// oclImgRaw.upload(img);
// ocl::cvtColor(oclImgRaw,oclImgConverted,CV_BGR2BGRA);
// ocl::meanShiftSegmentation(oclImgConverted, outImgOcl,30,30,(img.rows*img.cols)*0.01);
MSSegmenter segmenter; 
 segmenter.ms_segment_image(img, outImgOcl);
 ProccTimePrint(AAtime , "ocl");

// gpu::GpuMat pimgGpu, imgGpu, outImgGpu;
// gpu::setDevice(0);
// AAtime = getTickCount();
// std::cout<<"Uploading"<<std::endl;
// pimgGpu.upload(img);
// std::cout<<"Uploaded"<<std::endl;
// //gpu meanshift only support 8uc4 type.
// gpu::cvtColor(pimgGpu, imgGpu, CV_BGR2BGRA);
// std::cout<<"Converted"<<std::endl;
// gpu::meanShiftSegmentation(imgGpu, outImgCuda, 30, 30,1000);
// std::cout<<"Shifted"<<std::endl;
// ProccTimePrint(AAtime , "cuda");

 //show image
 // imshow("origin", img);
 imshow("MeanShift Filter OCL",  outImgOcl);
// imshow("MeanShift Filter cpu", outImg);
// imshow("MeanShift Filter Cuda", outImgCuda);


 waitKey();
 return 0;
}
