#ifndef RECORD_CALIB_DATA_H
#define RECORD_CALIB_DATA_H

//STL
#include <vector>
#include <cstdint>
#include <atomic>
#include <mutex>

//OpenNI
#include <OpenNI.h>

//OPENCV2
#include "core/core.hpp"

//EDVSTOOLS
#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

//DEDVS
#include "DEdvs/config.h"

namespace record_calib_data {

  dedvs::DEdvsStatus recordCalibrationData(const std::string& file_path);

  dedvs::DEdvsStatus setupHardware(openni::Device* device, openni::VideoStream* vs_depth,
                                   openni::VideoStream* vs_color, Edvs::EventStream** event_stream);

  void shutdownHardware(openni::Device* device, openni::VideoStream* vs_depth,
                        openni::VideoStream* vs_color, Edvs::EventStream* event_stream);

  void readEventStream(const std::atomic<bool>& thread_running, std::vector<Edvs::Event>* events,
                       Edvs::EventStream* event_stream, std::mutex* event_mutex);

  void matchEvents(const std::string& filename, std::atomic<bool>* thread_running,
                   std::vector<Edvs::Event>* buf_events, std::mutex* event_mutex,
                   openni::VideoStream* vs_depth, openni::VideoStream* vs_color);

  bool findDiodeInCheckerboard(const cv::Mat& grey_map, const uint16_t* &frame, dedvs::Point3DF* loc_diode);

  void interpolatePos(const dedvs::Point3DF& old_diode_location, const dedvs::Point3DF& new_diode_location, 
                      uint64_t time_event, uint64_t time_first_event, uint64_t time_last_event,
                      dedvs::DEventExtF* depth_event);
}


#endif