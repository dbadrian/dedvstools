#include <iostream>

#include "DEdvs/config.h"
#include "DEdvs/xtion_io.h"
#include "DEdvs/opencv_iop.h"

#include "core/core.hpp"
#include "highgui/highgui.hpp"

#include "OpenNI.h"

int main()
{
  openni::Device device;
  openni::VideoStream vs_color;

  if(dedvs::intializeOpenNI() != dedvs::DEDVS_STATUS_OK)       return dedvs::DEDVS_STATUS_ERROR;
  if(dedvs::openXtionDevice(&device) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;
  if(dedvs::createStream(dedvs::COLOR, &device, &vs_color) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;




  openni::CameraSettings* cs = vs_color.getCameraSettings();
  std::cout << cs->getAutoExposureEnabled() << " " << cs->getAutoWhiteBalanceEnabled() << std::endl;
  cs->setAutoExposureEnabled(true);
  cs->setAutoWhiteBalanceEnabled(false);
  std::cout << "Exposure: " << cs->getExposure() << std::endl;
  std::cout << "Exposure: " << cs->getGain() << std::endl;

  cs->setGain(2230);
  std::cout << "Exposure: " << cs->getExposure() << std::endl;

  if(dedvs::startStream(&vs_color) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;

  // cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE );
  cv::namedWindow("COLOR", CV_WINDOW_AUTOSIZE );
  cv::Mat col_mat;

  bool loop(true);
  while(loop)
  {
    dedvs::getColorFrameCVMat(&vs_color,&col_mat);
    imshow("COLOR", col_mat);
    
    if(cv::waitKey(30)>=0)
    {
      loop = false;
      break;
    }
  }

    dedvs::shutdownVideoStream(&vs_color);
    dedvs::shutdownDevice(&device);
    dedvs::shutdownOpenNI();


  std::cout << "end"<< std::endl;
}