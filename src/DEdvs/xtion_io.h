#ifndef XTION_IO_H
#define XTION_IO_H

//OpenNI2
#include <OpenNI.h>
#include <vector>

//DEdvs
#include "config.h"

namespace dedvs {

  DEdvsStatus intializeOpenNI();

  DEdvsStatus openXtionDevice(openni::Device* device);
  DEdvsStatus openXtionDevice(const char* uri, openni::Device* device);
  
  void shutdownOpenNI();
  void shutdownDevice(openni::Device* device);
  void shutdownVideoStream(openni::VideoStream* video_stream);

  void enumerateXtionDevices();

  DEdvsStatus createStream(OpenNIStreamType stream_type, openni::Device* device, openni::VideoStream* video_stream);
  DEdvsStatus createStream(OpenNIStreamType stream_type, openni::Device* device, openni::VideoStream* video_stream, StreamInfo* stream_info);
  DEdvsStatus startStream(openni::VideoStream* video_stream);

  DEdvsStatus getStreamInfo(const openni::VideoStream& video_stream, StreamInfo* stream_info);
  DEdvsStatus getAvailableVideoModes(const openni::VideoStream& video_stream, std::vector<StreamInfo>* stream_infos);
  openni::VideoMode getVideoModeFromConfig(OpenNIStreamType stream_type);
  DEdvsStatus setImageRegistrationMode(bool enable, openni::Device* device);
  DEdvsStatus setDepthColorSyncEnable(bool enable, openni::Device* device);

  DEdvsStatus getDepthFrame(openni::VideoStream* video_stream, const uint16_t** frame);
  DEdvsStatus getDepthFrame(openni::VideoStream* video_stream, const uint16_t** frame, size_t* size_of_frame);

  DEdvsStatus getColorFrame(openni::VideoStream* video_stream, const openni::RGB888Pixel** frame);
  DEdvsStatus getColorFrame(openni::VideoStream* video_stream, const openni::RGB888Pixel** frame, size_t* size_of_frame);

}

#endif