#include "record_calib_data.h"

//STL
#include <iostream>
#include <cstdint>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>

#include <OpenNI.h>

//OPENCV2
#include "core/core.hpp"
#include "imgproc/imgproc.hpp"
#include "calib3d/calib3d.hpp"
#include "highgui/highgui.hpp"

//EDVSTOOLS
#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

//DEDVS
#include "DEdvs/config.h"
#include "DEdvs/xtion_io.h"
#include "DEdvs/opencv_iop.h"
#include "DEdvs/dedvs_auxiliary.h"

namespace record_calib_data {

  dedvs::DEdvsStatus recordCalibrationData(const std::string& file_path)
  {
    openni::Device device;
    openni::VideoStream vs_depth, vs_color;
    Edvs::EventStream* event_stream = NULL;
    
    if(setupHardware(&device, &vs_depth, &vs_color, &event_stream) != dedvs::DEDVS_STATUS_OK)
      return dedvs::DEDVS_STATUS_ERROR;

    std::vector<Edvs::Event> buf_events;

    std::atomic<bool> thread_running(true);
    std::mutex event_mutex;

    std::thread read_events_thread(readEventStream, std::cref(thread_running), &buf_events,
                      event_stream, &event_mutex);
    std::thread match_data_thread(matchEvents, std::cref(file_path), &thread_running,
                   &buf_events, &event_mutex, &vs_depth, &vs_color);

    read_events_thread.join();
    match_data_thread.join();

    shutdownHardware(&device, &vs_depth, &vs_color, event_stream);
    return dedvs::DEDVS_STATUS_OK;
  }


  dedvs::DEdvsStatus setupHardware(openni::Device* device, openni::VideoStream* vs_depth,
                    openni::VideoStream* vs_color, Edvs::EventStream** event_stream)
  {
    if(dedvs::intializeOpenNI() != dedvs::DEDVS_STATUS_OK)       return dedvs::DEDVS_STATUS_ERROR;

    if(dedvs::openXtionDevice(device) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;

    if(dedvs::createStream(dedvs::DEPTH, device, vs_depth) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;
    if(dedvs::createStream(dedvs::COLOR, device, vs_color) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;

    if(dedvs::setImageRegistrationMode(true, device) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;
    if(dedvs::setDepthColorSyncEnable(true, device) != dedvs::DEDVS_STATUS_OK)  return dedvs::DEDVS_STATUS_ERROR;

    if(dedvs::startStream(vs_depth) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;
    if(dedvs::startStream(vs_color) != dedvs::DEDVS_STATUS_OK) return dedvs::DEDVS_STATUS_ERROR;


    *event_stream = dedvs::setupEdvs();

    return dedvs::DEDVS_STATUS_OK;
  }

  void shutdownHardware(openni::Device* device, openni::VideoStream* vs_depth,
                    openni::VideoStream* vs_color, Edvs::EventStream* event_stream)
  {
    //In order: Close and destroy streams, shutdown device, and unload drivers
    dedvs::shutdownVideoStream(vs_depth);
    dedvs::shutdownVideoStream(vs_color);
    dedvs::shutdownDevice(device);
    dedvs::shutdownOpenNI();

    // Close EDVS event stream
    event_stream->close();
  }

  void readEventStream(const std::atomic<bool>& thread_running, std::vector<Edvs::Event>* events,
                       Edvs::EventStream* event_stream, std::mutex* event_mutex)
  {
    std::vector<Edvs::Event> buf_stream_events, buf_filtered_events;

    //A timestamp map of all pixels, only being updated at the local if an event appeared and the
    //time before that. Events which don't seem to appear at a constant frequency are filtered.
    std::vector<uint64_t> ts_events1(dedvs::kEdvsResolution_U * dedvs::kEdvsResolution_V, 0);
    std::vector<uint64_t> ts_events2(dedvs::kEdvsResolution_U * dedvs::kEdvsResolution_V, 0);
    buf_filtered_events.reserve(1024);

    while(thread_running)
    {
      buf_stream_events.clear();
      buf_stream_events.resize(1024);

      event_stream->read(buf_stream_events);
      dedvs::filterEventsByFrequency(buf_stream_events, &buf_filtered_events, &ts_events1, &ts_events2);

      // if(event_mutex->try_lock() == -1)
      // {
      event_mutex->lock();
        dedvs::transformEvents(&buf_filtered_events);
        events->insert(events->end(),buf_filtered_events.begin(),buf_filtered_events.end());
        
        buf_filtered_events.clear();
        buf_filtered_events.reserve(1024);
      event_mutex->unlock();
      // }

    }
  }

  void matchEvents(const std::string& filename, std::atomic<bool>* thread_running,
                   std::vector<Edvs::Event>* buf_events, std::mutex* event_mutex,
                   openni::VideoStream* vs_depth, openni::VideoStream* vs_color)
  {
    const uint16_t *depth_frame;
    cv::Mat color_frame, grey_frame;
    cv::Mat depth_frame_mat, depth_frame_col_mat;
    cv::Mat event_frame(dedvs::kEdvsResolution_U,dedvs::kEdvsResolution_V,CV_8UC3,cv::Scalar(128,128,128));
    cv::Mat resized_events_frame(512,512,CV_8UC3,cv::Scalar(128,128,128));

    std::vector<dedvs::DEventExtF> buf_depth_events;
    std::vector<Edvs::Event> buf_coverage_events;
    dedvs::DEventExtF temp_event;
    buf_depth_events.reserve(1024*256);

    dedvs::Point3DF old_diode_location, new_diode_location;
    uint64_t time_first_event, time_last_event;

    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Events",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Depth",CV_WINDOW_AUTOSIZE);    

    do {
      dedvs::getDepthFrame(vs_depth, &depth_frame);
      dedvs::getDepthFrameCVMat(depth_frame, 
                                dedvs::kXtionDepthResolutionX*dedvs::kXtionDepthResolutionY*sizeof(uint16_t), 
                                &depth_frame_mat);
      dedvs::colorizeDepthCVMat(depth_frame_mat, &depth_frame_col_mat);

      dedvs::getColorFrameCVMat(vs_color, &color_frame);
      cv::cvtColor(color_frame, grey_frame, CV_BGR2GRAY);

      imshow("Events", resized_events_frame);
      imshow("Color", color_frame);
      imshow("Depth", depth_frame_col_mat);

      if(cv::waitKey(10)>=0)
      {
        *thread_running = false;
        break;
      }

    } while(!findDiodeInCheckerboard(grey_frame, depth_frame, &new_diode_location)); //!findDiodeInCheckerboard(grey_frame, depth_frame, &new_diode_location)

    //Clear accumulated events, to ensure a "clean" entry point
    event_mutex->lock();
      buf_events->clear();
      buf_events->reserve(1024);
    event_mutex->unlock();  
    
    while(*thread_running)
    {
      event_frame = cv::Mat(dedvs::kEdvsResolution_U,dedvs::kEdvsResolution_V,CV_8UC3,cv::Scalar(128,128,128)); //TODO FIXME EFFECIENTLY
      old_diode_location = new_diode_location;

      dedvs::getDepthFrame(vs_depth, &depth_frame);
      dedvs::getColorFrameCVMat(vs_color, &color_frame);

      event_mutex->lock();
      {
        cv::cvtColor(color_frame, grey_frame, CV_BGR2GRAY);
        imshow("Color", color_frame);

        if(!findDiodeInCheckerboard(grey_frame, depth_frame, &new_diode_location))
        { 
          event_mutex->unlock();  

          do {
            std::cout << "Cant find diode" << std::endl;
            dedvs::getDepthFrame(vs_depth, &depth_frame);
            dedvs::getColorFrameCVMat(vs_color, &color_frame);
            cv::cvtColor(color_frame, grey_frame, CV_BGR2GRAY);
            imshow("Color", color_frame);            
          } while(!findDiodeInCheckerboard(grey_frame, depth_frame, &new_diode_location));
          
          event_mutex->lock();
          {
            buf_events->clear();
            buf_events->reserve(1024);
          }
          event_mutex->unlock();
          continue;
        }

        if(buf_events->size() < 2 || (buf_events->front().t == buf_events->back().t))
        {
          std::cout << "no events!!!" << std::endl;
          buf_events->clear();
          buf_events->reserve(1024);
          event_mutex->unlock();            

          if(cv::waitKey(2)>=0)
          {
            // event_mutex->unlock(); //So the other thread can finish
            *thread_running = false;
            break;
          }

          continue;
        }

        time_first_event = buf_events->front().t;
        time_last_event  = buf_events->back().t;

        temp_event.time = static_cast<uint64_t>(llround(static_cast<double>(time_last_event-time_first_event)/2.0)) + time_first_event;
        temp_event.parity = 0; //parity is useless here

        float mw_u(0.0), mw_v(0.0);

        for(std::vector<Edvs::Event>::iterator iter = buf_events->begin();
          iter != buf_events->end(); ++iter)
        {
          //TODO guassian weighting
          mw_u += static_cast<float>(iter->x);
          mw_v += static_cast<float>(iter->y);
        }
        temp_event.u = mw_u/static_cast<float>(buf_events->size());
        temp_event.v = mw_v/static_cast<float>(buf_events->size());

        interpolatePos(old_diode_location,new_diode_location, temp_event.time, 
                       time_first_event, time_last_event, &temp_event);

        buf_depth_events.push_back(temp_event);

        buf_coverage_events.insert(buf_coverage_events.end(),buf_events->begin(),buf_events->end());
        dedvs::getEventFrame(buf_coverage_events,&event_frame);
        //Clear the cache
        buf_events->clear();
        buf_events->reserve(1024);
      }
      event_mutex->unlock();

      cv::resize(event_frame,resized_events_frame,cv::Size(512,512),0,0,cv::INTER_LINEAR);

      cv::circle(color_frame,cv::Point(temp_event.x,temp_event.y),4,cv::Scalar(255,0,0));
      imshow("Events", resized_events_frame);
      imshow("Color", color_frame);

      if(cv::waitKey(2)>=0)
      {
        // event_mutex->unlock(); //So the other thread can finish
        *thread_running = false;
        break;
      }


    }  

    std::cout << "Writing Events to drive ...." << std::endl;
    //write in bigger chunks to harddrive
    std::cout << "Size of buf_depth_events " << buf_depth_events.size() << std::endl;
      writeDEventsToTVSFile(filename, buf_depth_events);
    std::cout << " Done!" << std::endl;
  }

  bool findDiodeInCheckerboard(const cv::Mat& grey_map, const uint16_t* &frame, dedvs::Point3DF* loc_diode)
  {
    cv::Size patternsize(4,4); //interior number of corners
    std::vector<cv::Point2f> corners; //this will be filled by the detected corners

    bool patternfound = cv::findChessboardCorners(grey_map, patternsize, corners,
                                                   cv::CALIB_CB_ADAPTIVE_THRESH 
                                                   +
                                                   cv::CALIB_CB_NORMALIZE_IMAGE 
                                                  //+ cv::CALIB_CB_FAST_CHECK 
                                                      );
    if(patternfound){  //TODO OPTIMIZE PARAMETERS
      cv::cornerSubPix(grey_map, corners, cv::Size(11, 11), cv::Size(-1, -1),
                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    } 
    else 
    {
      std::cerr << "WARNING: No checkerboard found or out of bounds!" << std::endl;
      return false;
    }

    //Calculate x,y posititions
    loc_diode->x = (corners[5].x + corners[6].x + corners[9].x + corners[10].x)/4.0;
    loc_diode->y = (corners[5].y + corners[6].y + corners[9].y + corners[10].y)/4.0;

    float depth = static_cast<float>(
                      frame[lround(corners[5].x) + dedvs::kXtionDepthResolutionX * lround(corners[5].y)] + 
                      frame[lround(corners[6].x) + dedvs::kXtionDepthResolutionX * lround(corners[6].y)] +
                      frame[lround(corners[9].x) + dedvs::kXtionDepthResolutionX * lround(corners[9].y)] + 
                      frame[lround(corners[10].x) + dedvs::kXtionDepthResolutionX * lround(corners[10].y)]
                  );

    loc_diode->depth = depth/4.0;

    return true;
  }

  void interpolatePos(const dedvs::Point3DF& old_diode_location, const dedvs::Point3DF& new_diode_location, 
                      uint64_t time_event, uint64_t time_first_event, uint64_t time_last_event,
                      dedvs::DEventExtF* depth_event)
  {
    // float alpha = static_cast<float>(time_event - time_first_event) / 
    //               static_cast<float>(time_last_event - time_first_event);

    // depth_event->x     = (1.0f - alpha) * old_diode_location.x + alpha * new_diode_location.x;
    // depth_event->y     = (1.0f - alpha) * old_diode_location.y + alpha * new_diode_location.y;
    // depth_event->depth = (1.0f - alpha) * old_diode_location.depth + alpha * new_diode_location.depth;

    // SIMPLE INTERPOLATION
    depth_event->x     = 0.5 * (old_diode_location.x + new_diode_location.x);
    depth_event->y     = 0.5 * (old_diode_location.y + new_diode_location.y);
    depth_event->depth = 0.5 * (old_diode_location.depth + new_diode_location.depth);
  }
}
