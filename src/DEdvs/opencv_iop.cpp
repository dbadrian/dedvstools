#include "opencv_iop.h"

#include <cstdint>
#include <iostream>
#include <vector>

#include "OpenNI.h"

#include "core/core.hpp"
#include "imgproc/imgproc.hpp"
#include "contrib/contrib.hpp"

#include "Edvs/Event.hpp"

#include "xtion_io.h"
#include "config.h"

namespace dedvs {

  DEdvsStatus getDepthFrameCVMat(openni::VideoStream* video_stream, cv::Mat* cv_mat)
  {
    if(video_stream->getSensorInfo().getSensorType() != openni::SENSOR_DEPTH)
    {
      std::cerr << "ERROR: Trying to create depth-data OpenCV Mat from wrong" 
                    "VideoStream with incorrect openni::SensorType!" << std::endl;
      return DEDVS_STATUS_BAD_PARAMETER;
    }
    if(video_stream->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
    {
      std::cerr << "ERROR: Trying to create depth-data OpenCV Mat from wrong"
                    "VideoStream with incorrect PixelFormat!" << std::endl;
      return DEDVS_STATUS_BAD_PARAMETER;
    }

    const uint16_t* frame = NULL;
    size_t size_of_frame = 0;
    
    DEdvsStatus status = getDepthFrame(video_stream, &frame, &size_of_frame);
    if(DEDVS_STATUS_OK == status)
    {
      getDepthFrameCVMat(frame, size_of_frame, cv_mat);
    }

    return status;
  }  

  void getDepthFrameCVMat(const uint16_t* frame, const size_t size_of_frame, cv::Mat* cv_mat)
  {
    if(cv_mat->cols != kXtionDepthResolutionX || cv_mat->rows != kXtionDepthResolutionY) 
    {
          cv_mat->create(cv::Size(kXtionDepthResolutionX, kXtionDepthResolutionY), CV_16U);
    }

    memcpy(cv_mat->data, frame, size_of_frame);
  }  

  DEdvsStatus getColorFrameCVMat(openni::VideoStream* video_stream, cv::Mat* cv_mat)
  {
    if(video_stream->getSensorInfo().getSensorType() != openni::SENSOR_COLOR)
    {
      std::cerr << "ERROR: Trying to create color-data OpenCV Mat from wrong" 
                    "VideoStream with incorrect openni::SensorType!" << std::endl;
      return DEDVS_STATUS_BAD_PARAMETER;
    }
    if(video_stream->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888)
    {
      std::cerr << "ERROR: Trying to create color-data OpenCV Mat from wrong" 
                    "VideoStream with incorrect PixelFormat!" << std::endl;
      return DEDVS_STATUS_BAD_PARAMETER;
    }

    const openni::RGB888Pixel *frame = NULL;
    size_t size_of_frame = 0;
    
    DEdvsStatus status = getColorFrame(video_stream, &frame, &size_of_frame);
    if(DEDVS_STATUS_OK == status)
    {
      getColorFrameCVMat(frame, size_of_frame, cv_mat);
    }

    return status;
  }  

  void getColorFrameCVMat(const openni::RGB888Pixel* frame, const size_t size_of_frame, cv::Mat* cv_mat)
  {
    if(cv_mat->cols != kXtionColorResolutionX || cv_mat->rows != kXtionColorResolutionY) 
    {
          cv_mat->create(cv::Size(kXtionColorResolutionX, kXtionColorResolutionY), CV_8UC3);
      }

    memcpy(cv_mat->data, frame, size_of_frame);
    cvtColor(*cv_mat, *cv_mat, CV_RGB2BGR);
  }  

  void getEventFrame(const std::vector<Edvs::Event>& events, cv::Mat* event_frame)
  {
    for(std::vector<Edvs::Event>::const_iterator iter = events.cbegin();
        iter != events.cend(); ++iter )
    {
      event_frame->at<cv::Vec3b>(iter->y,iter->x)[0] = 0;
      event_frame->at<cv::Vec3b>(iter->y,iter->x)[1] = 0;
      event_frame->at<cv::Vec3b>(iter->y,iter->x)[2] = 255;
    }
  }

  void colorizeEvents(const std::vector<dedvs::DEvent>& depth_events, cv::Mat* col_event_map)
  {
    double min = dedvs::kMinDepth;
    double max = dedvs::kMaxDepth;
    double scale = 255 / (max-min);

    cv::Mat event_frame_depth(kEdvsResolution_U,kEdvsResolution_V, CV_16U, 1);
    cv::Mat event_frame_bgr;

    int min_y, min_x, max_y, max_x;

    for(std::vector<dedvs::DEvent>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      event_frame_depth.at<uint16_t>(iter->v,iter->u) = static_cast<uint16_t>(iter->depth);
    }

    event_frame_depth.convertTo(event_frame_bgr,CV_8UC1, scale, -min*scale); 
    applyColorMap(event_frame_bgr, *col_event_map, cv::COLORMAP_RAINBOW);

    //RESET ALL POINTS WHERE NO EVENT HAPPENED TO GREY (HACKY :))
    for (int j = 0; j < kEdvsResolution_V; ++j)
    {
      for (int i = 0; i < kEdvsResolution_U; ++i)
      {
        if(col_event_map->at<cv::Vec3b>(j,i)[2] > 250 &&
           col_event_map->at<cv::Vec3b>(j,i)[1] < 5   && 
           col_event_map->at<cv::Vec3b>(j,i)[0] < 5)
        {
          col_event_map->at<cv::Vec3b>(j,i)[0] = 0;
          col_event_map->at<cv::Vec3b>(j,i)[1] = 0;
          col_event_map->at<cv::Vec3b>(j,i)[2] = 0;
        }
      }
    }
  }

  void getDepthMapCVMat(const std::vector<dedvs::DEvent>& depth_events,
                        const std::vector<dedvs::Point3D>& depth_map,
                        cv::Mat* depth_map_frame)
  {
    double min = dedvs::kMinDepth;
    double max = dedvs::kMaxDepth;
    double scale = 255 / (max-min);

    cv::Mat depth_map_frame_grey(dedvs::kEdvsResolution_U,dedvs::kEdvsResolution_V, CV_16U, 1);
    cv::Mat depth_map_frame_bgr;
    
    for(int y = 0; y < dedvs::kEdvsResolution_V; ++y)
    {
      for(int x = 0; x < dedvs::kEdvsResolution_V; x++)
      {
        depth_map_frame_grey.at<uint16_t>(y,x) = depth_map[x + dedvs::kEdvsResolution_U * y].depth;
      }
    }

    depth_map_frame_grey.convertTo(depth_map_frame_bgr,CV_8UC1, scale, -min*scale); 
    applyColorMap(depth_map_frame_bgr, *depth_map_frame, cv::COLORMAP_RAINBOW);

    for(std::vector<dedvs::DEvent>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      depth_map_frame->at<cv::Vec3b>(iter->v,iter->u)[0] = 0;
      depth_map_frame->at<cv::Vec3b>(iter->v,iter->u)[1] = 0;
      depth_map_frame->at<cv::Vec3b>(iter->v,iter->u)[2] = 0;
    }
  }

  void colorizeDepthCVMat(const cv::Mat& depth_mat, cv::Mat* depth_col_mat)
  {
    double min = dedvs::kMinDepth;
    double max = dedvs::kMaxDepth;
    double scale = 255 / (max-min);

    cv::Mat depth_frame_norm_mat;
    depth_mat.convertTo(depth_frame_norm_mat,CV_8UC1, scale, -min*scale); 
    applyColorMap(depth_frame_norm_mat, *depth_col_mat, cv::COLORMAP_RAINBOW);

    for(int i=0;i<dedvs::kXtionDepthResolutionX*dedvs::kXtionDepthResolutionY;i++)
    {
      if(depth_mat.at<uint16_t>(i) < 10)
      {
        depth_col_mat->at<cv::Vec3b>(i)[0] = 0;
        depth_col_mat->at<cv::Vec3b>(i)[1] = 0;
        depth_col_mat->at<cv::Vec3b>(i)[2] = 0;
      }
    }
  }

}