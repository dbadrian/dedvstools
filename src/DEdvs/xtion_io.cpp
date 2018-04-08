#include "xtion_io.h"

//STL
#include <iostream>
#include <vector>

//OpenNI2
#include <OpenNI.h>

//DEdvs
#include "config.h"


namespace dedvs {

  DEdvsStatus intializeOpenNI()
  {
    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: Initialization OpenNI Framework / Drivers failed!" <<  std::endl;
      std::cerr << openni::OpenNI::getExtendedError() << std::endl;
      return DEDVS_STATUS_ERROR;
    }
    else
    {
      return DEDVS_STATUS_OK;
    }
  }

  void shutdownOpenNI()
  {
    openni::OpenNI::shutdown();
  }

  void shutdownDevice(openni::Device* device)
  {
    device->close();
  }

  void shutdownVideoStream(openni::VideoStream* video_stream)
  {
    video_stream->stop();
    video_stream->destroy();
  }

  DEdvsStatus openXtionDevice(openni::Device* device)
  {
    openni::Status status = device->open(openni::ANY_DEVICE);
    if (status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: OpenNI device could not be opened!" <<  std::endl;
      std::cerr << openni::OpenNI::getExtendedError() << std::endl;
      return DEDVS_STATUS_ERROR;
    }
    else
    {
      return DEDVS_STATUS_OK;
    }

  }

  DEdvsStatus openXtionDevice(const char* uri, openni::Device* device)
  {
    openni::Status status = device->open(uri);
    if (status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: OpenNI device could not be opened!" <<  std::endl;
      std::cerr << openni::OpenNI::getExtendedError() << std::endl;
      return DEDVS_STATUS_ERROR;
    }
    else
    {
      return DEDVS_STATUS_OK;
    }
  }

  void enumerateXtionDevices()
  {
    openni::Array<openni::DeviceInfo> deviceinfos;
    openni::OpenNI::enumerateDevices(&deviceinfos);

    for(int i=0; i<deviceinfos.getSize(); ++i)
    {
      std::cout << "\n### DEVICE INFO ###" << std::endl;
      std::cout << "Name: " << deviceinfos[i].getName() << " | URI: " << deviceinfos[i].getUri() << std::endl;
      std::cout << "Vendor: " << deviceinfos[i].getVendor() << " USB Product Id" << deviceinfos[i].getUsbProductId() << "\n" << std::endl; 
    }

    std::cout << std::endl;
  }

  DEdvsStatus createStream(OpenNIStreamType stream_type, openni::Device* device, openni::VideoStream* video_stream)
  {
    StreamInfo will_not_be_used;
    return createStream(stream_type, device, video_stream, &will_not_be_used);
  }

  DEdvsStatus createStream(OpenNIStreamType stream_type, openni::Device* device, openni::VideoStream* video_stream, StreamInfo* stream_info)
  {
    if(device == NULL) return DEDVS_STATUS_BAD_PARAMETER;

    openni::SensorType sensor_type = (stream_type == DEPTH) ? openni::SENSOR_DEPTH : openni::SENSOR_COLOR;
    const openni::VideoMode vm_setup = getVideoModeFromConfig(stream_type);

    if(device->hasSensor(sensor_type))
    {
      openni::Status status = video_stream->create(*device, sensor_type);

      if (status != openni::STATUS_OK)
      {
        std::cerr << "ERROR: OpenNI stream could not be created!" <<  std::endl;
        std::cerr << openni::OpenNI::getExtendedError() << std::endl;
        return DEDVS_STATUS_STREAM_ERROR;
      }

      if(video_stream->isValid())
        {
        status = video_stream->setVideoMode(vm_setup); 
        if (status != openni::STATUS_OK || !video_stream->isValid())
        {
          std::cerr << "ERROR: OpenNI stream could not be configured as defined in config.h (VideoMode)" <<  std::endl;
          std::cerr << openni::OpenNI::getExtendedError() << std::endl;
          return DEDVS_STATUS_STREAM_ERROR;
        }   

        stream_info->mirroring_enabled = kXtionMirroring;
        video_stream->setMirroringEnabled(kXtionMirroring);

        getStreamInfo(*video_stream, stream_info);
      }
      else 
      {
        std::cerr << "ERROR: OpenNI stream was created, but was not valid!" <<  std::endl;
        return DEDVS_STATUS_STREAM_ERROR;
      }
    }
    else 
    {
      std::cerr << "ERROR: The intended sensor stream is not available!" << std::endl;
      return DEDVS_STATUS_HW_ERROR;
    }

    return DEDVS_STATUS_OK;
  }

  DEdvsStatus startStream(openni::VideoStream* video_stream)
  {
    openni::Status status = video_stream->start();

    if(status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: Video Stream could not be started!" <<  std::endl;
      std::cerr << openni::OpenNI::getExtendedError() << std::endl;
      return DEDVS_STATUS_STREAM_ERROR;
    }
    else
    {
      return DEDVS_STATUS_OK;
    }
  }

  DEdvsStatus getStreamInfo(const openni::VideoStream& video_stream, StreamInfo* stream_info)
  {
    if(video_stream.isValid())
      {
        openni::VideoMode vm = video_stream.getVideoMode();  
        stream_info->x_resolution = vm.getResolutionX();
        stream_info->y_resolution = vm.getResolutionY();
        stream_info->fps          = vm.getFps();
        stream_info->mirroring_enabled = video_stream.getMirroringEnabled();
        return DEDVS_STATUS_OK;
    }
    else 
    {
      std::cerr << "ERROR: OpenNI stream accessed was not valid!" <<  std::endl;
      return DEDVS_STATUS_STREAM_ERROR;
    }
  }

  DEdvsStatus getAvailableVideoModes(const openni::VideoStream& video_stream, std::vector<StreamInfo>* stream_infos)
  {
    const openni::Array<openni::VideoMode>& vm = video_stream.getSensorInfo().getSupportedVideoModes();
    for(int i=0; i < vm.getSize(); ++i)
    {
      if(vm[i].getPixelFormat() == 101 || vm[i].getPixelFormat() == 201 || vm[i].getPixelFormat() == 202)
        continue;

      StreamInfo s_info = {
        vm[i].getResolutionX(),
        vm[i].getResolutionY(),
        vm[i].getFps(),
        video_stream.getMirroringEnabled()
      };

      stream_infos->push_back(s_info);
    }

  }

  openni::VideoMode getVideoModeFromConfig(OpenNIStreamType stream_type)
  {
    openni::VideoMode vm;
    switch(stream_type)
    {
      case DEPTH: vm.setFps(kXtionDepthFPS);
                  vm.setResolution(kXtionDepthResolutionX,kXtionDepthResolutionY);
                  vm.setPixelFormat(static_cast<openni::PixelFormat>(kXtionDepthPixelFormat));
                  break;

      case COLOR: vm.setFps(kXtionColorFPS);
                  vm.setResolution(kXtionColorResolutionX,kXtionColorResolutionY);
                  vm.setPixelFormat(static_cast<openni::PixelFormat>(kXtionColorPixelFormat));
                  break;
    }
    return vm;
  }

  DEdvsStatus setImageRegistrationMode(bool enable, openni::Device* device)
  {
    openni::Status status;
    if(enable == true)
      status = device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    else
      status = device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

    return (status == openni::STATUS_OK) ? DEDVS_STATUS_OK : DEDVS_STATUS_ERROR;
  }

  DEdvsStatus setDepthColorSyncEnable(bool enable, openni::Device* device)
  {
    openni::Status status = device->setDepthColorSyncEnabled(enable);
    return (status == openni::STATUS_OK) ? DEDVS_STATUS_OK : DEDVS_STATUS_ERROR;
  }


  DEdvsStatus getDepthFrame(openni::VideoStream* video_stream, const uint16_t** frame)
  {
    size_t will_not_be_used;
    return getDepthFrame(video_stream, frame, &will_not_be_used);
  }
  
  DEdvsStatus getDepthFrame(openni::VideoStream* video_stream, const uint16_t** frame, size_t* size_of_frame)
  {
    openni::VideoFrameRef frame_ref;  
    openni::Status status = video_stream->readFrame(&frame_ref);
    if (status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: Reading of depth frame failed!" << std::endl;
      *frame = NULL;
      *size_of_frame = 0;
      return DEDVS_STATUS_STREAM_ERROR;
    }

    *frame = static_cast<const uint16_t*>(frame_ref.getData());
    *size_of_frame = static_cast<size_t>(frame_ref.getStrideInBytes() * frame_ref.getHeight());

    return DEDVS_STATUS_OK;
  }

  DEdvsStatus getColorFrame(openni::VideoStream* video_stream, const openni::RGB888Pixel** frame)
  {
    size_t will_not_be_used;
    return getColorFrame(video_stream, frame, &will_not_be_used);
  }

  DEdvsStatus getColorFrame(openni::VideoStream* video_stream, const openni::RGB888Pixel** frame, size_t* size_of_frame)
  {
    openni::VideoFrameRef frame_ref;  
    openni::Status status = video_stream->readFrame(&frame_ref);
    if (status != openni::STATUS_OK)
    {
      std::cerr << "ERROR: Reading of color frame failed!" << std::endl;
      *frame = NULL;
      *size_of_frame = 0;
      return DEDVS_STATUS_STREAM_ERROR;
    }

    *frame = static_cast<const openni::RGB888Pixel*>(frame_ref.getData());
    *size_of_frame = static_cast<size_t>(frame_ref.getStrideInBytes() * frame_ref.getHeight());

    return DEDVS_STATUS_OK;
  }


}