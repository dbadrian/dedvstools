#ifndef DEDVS_AUXILIARY_H
#define DEDVS_AUXILIARY_H

#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

//OpenNI
#include "OpenNI.h"

#include "config.h"

#include <memory>

namespace dedvs {

  const Eigen::Matrix3f kR = (Eigen::Matrix3f() << kR11, kR12, kR13, kR21, kR22, kR23, kR31, kR32, kR33).finished();
  const Eigen::Matrix3f kRInv = kR.inverse();
  const Eigen::Vector3f kT = (Eigen::Vector3f() << kT1, kT2, kT3).finished();

  struct PrimesenseToEdvsLUT
  {
    int nx, ny, nd;
    std::vector<uint16_t> ptr;

    void create(int nx_, int ny_, int nd_) {
      nx = nx_;
      ny = ny_;
      nd = nd_;
      ptr.resize(nx*ny*nd);
    }

    uint16_t at(int x, int y, int d) const {
      // return depth_lut[x][y][d];
      return ptr[x*ny*nd + y*nd + d];
    }
  };


  ////////////////////////////////////// 
  void transformEvents(std::vector<Edvs::Event>* events);

  void filterEventsByFrequency(const std::vector<Edvs::Event>& unfiltered_events, 
                               std::vector<Edvs::Event>* filtered_events,
                               std::vector<uint64_t>* ts_events1, std::vector<uint64_t>* ts_events2);


  std::vector<DEvent> augmentEvents(const std::vector<Point3D>& depth_map, const ctp last_depth_frame, 
                                    const ctp ts_init, std::vector<DEvent>* events);

  uint16_t findNearestEdgeDepth(int u, int v, const std::vector<dedvs::Point3D>& depth_map);
  ////////////////////////////////////// IO
  
  void getTimeString(char* buffer);
  
  Edvs::EventStream* setupEdvs();
  void flushEventsFromStream(Edvs::EventStream *event_stream);

  PrimesenseToEdvsLUT loadDepthLUT(const std::string& path_to_lut);

  void generateDepthMap(const uint16_t* depth_frame, const PrimesenseToEdvsLUT& lut,
                        std::vector<dedvs::Point3D>* depth_map);
  
  DEvent createDEvent(const Edvs::Event &event, uint16_t depth);
  DEventExt createDEvent(const Edvs::Event &event, const Point3D &point);
  DEventExt createDEvent(const Edvs::Event &event, const Point3DF &point);
  DEventExtF createDEventF(const Edvs::Event &event, const Point3DF &point);
  void createDEvent(const Edvs::Event &event, const Point3DF &point, DEventExt* depth_event);

  void writeDepthFrameToBinaryFile(const std::string path_to_output, const uint16_t* depth_frame);
  void writeRGBFrameToBinaryFile(const std::string path_to_output, const openni::RGB888Pixel* rgb_frame);

  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEvent>& depth_events);
  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEventExt>& depth_events);
  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEventExtF>& depth_events);

  template<typename T>
  void writeDEventsToFANNFile(const std::string filename, const std::vector<T>& depth_events)
  {
    std::ofstream ofile(filename, std::ios::out);
    if(!ofile.is_open())
      std::cerr << "ERROR: File " << filename << "could not be opened!" << std::endl;

    ofile << depth_events.size() << " 3 2\n";
    for(typename std::vector<T>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      ofile << (*iter).x    << " " << (*iter).y << " " << (*iter).depth << "\n"
          << (*iter).u    << " "  << (*iter).v    << "\n";
    }
    ofile.close();
  }

  void writeDEventsToFANNFile(const std::string& filename, const std::vector<dedvs::DEvent>& depth_events);

  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEvent>* depth_events);
  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEventExt>* depth_events);
  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEventExtF>* depth_events_float); //TODO template?

  Eigen::Vector3f unprojectEDVS(const DEvent& de);

  inline Eigen::Vector3f unprojectEDVS(uint16_t u, uint16_t v, uint16_t depth)
  {
    const float scl = 0.001f * static_cast<float>(depth) / kEDVSFocalPX;
    Eigen::Vector2f uv(static_cast<float>(u)-0.5f*kEdvsResolution_U,
                       static_cast<float>(v)-0.5f*kEdvsResolution_V);
    double l = 1.0 + kEDVSKappa1 * uv.norm() + kEDVSKappa2 * uv.squaredNorm();
    uv = uv * l;
    Eigen::Vector3f q(uv(0),uv(1),kEDVSFocalPX);
    q = kRInv * (q-kT);
    return (scl*q);
  }

  Eigen::Vector3f unprojectKinect(const DEvent& de);

  inline Eigen::Vector3f unprojectKinect(uint16_t x, uint16_t y, uint16_t depth)
  {
    const float scl = 0.001f * static_cast<float>(depth) / kKinectFocalPX;
    Eigen::Vector3f q(static_cast<float>(x)-0.5f*kXtionDepthResolutionX,
                      static_cast<float>(y)-0.5f*kXtionDepthResolutionY,
                      kKinectFocalPX);
    return (scl*q);
  }


  inline void projectEDVS(const Eigen::Vector3f& position_3d, float* u, float* v)
  {
    Eigen::Vector3f q = kR * position_3d + kT;
    Eigen::Vector2f uv(kEDVSFocalPX * q(0)/q(2),kEDVSFocalPX * q(1)/q(2));
    // Eigen::Vector2d uv = q.block(0,2) * (fe / q(2));
    float l = 1.0 / (1.0 + kEDVSKappa1 * uv.norm() + kEDVSKappa2 * uv.squaredNorm());
    uv = l * uv + 0.5f*Eigen::Vector2f(kEdvsResolution_U,kEdvsResolution_V);

    *u = uv(0);
    *v = uv(1);
  }

  inline void projectKinect(const Eigen::Vector3f& position_3d, float* x, float* y)
  {
    Eigen::Vector2f xy(kKinectFocalPX * position_3d(0)/position_3d(2),
                       kKinectFocalPX * position_3d(1)/position_3d(2));
    *x = xy(0) + 0.5f*kXtionDepthResolutionX;
    *y = xy(1) + 0.5f*kXtionDepthResolutionY;
  }




}

#endif