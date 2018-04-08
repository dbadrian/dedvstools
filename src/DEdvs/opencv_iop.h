#ifndef OPENCV_XTION_IOP_H
#define OPENCV_XTION_IOP_H

#include <cstdint>
#include <vector>

//OPENCV2
#include "core/core.hpp"

#include "Edvs/Event.hpp"

#include "xtion_io.h"
#include "config.h"

namespace dedvs {

  DEdvsStatus getDepthFrameCVMat(openni::VideoStream* video_stream, cv::Mat* cv_mat);
  void getDepthFrameCVMat(const uint16_t* frame, const size_t size_of_frame, cv::Mat* cv_mat);

  DEdvsStatus getColorFrameCVMat(openni::VideoStream* video_stream, cv::Mat* cv_mat);
  void getColorFrameCVMat(const openni::RGB888Pixel* frame, const size_t size_of_frame, cv::Mat* cv_mat);

  void getEventFrame(const std::vector<Edvs::Event>& events, cv::Mat* event_frame);

  void colorizeEvents(const std::vector<dedvs::DEvent>& depth_events, cv::Mat* col_event_map);

  void getDepthMapCVMat(const std::vector<dedvs::DEvent>& depth_events,
                        const std::vector<dedvs::Point3D>& depth_map,
                        cv::Mat* depth_map_frame);

  void colorizeDepthCVMat(const cv::Mat& depth_mat, cv::Mat* depth_col_mat);
}

//OPENCV_XTION_IOP_H
#endif 
