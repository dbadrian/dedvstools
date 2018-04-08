#include "dedvs_auxiliary.h"

#include <vector>
#include <ctime>
#include <fstream>
#include <chrono>
#include <ratio>

#include <boost/format.hpp>

//OpenNI
#include "OpenNI.h"

//EDVSTOOLS
#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

#include <Eigen/Dense>

#include "config.h"

namespace dedvs {

  void transformEvents(std::vector<Edvs::Event>* events)
  {
    for(std::vector<Edvs::Event>::iterator iter = events->begin();
      iter != events->end(); ++iter)
    {
      if(kEdvsFlipXY)
      {
        iter->x = (kEdvsMirrorY) ? (kEdvsResolution_V-1) - iter->y : iter->y;
        iter->y = (kEdvsMirrorX) ? (kEdvsResolution_U-1) - iter->x : iter->x;
      } 
      else
      {
        iter->x = (kEdvsMirrorX) ? (kEdvsResolution_U-1) - iter->x : iter->x;
        iter->y = (kEdvsMirrorY) ? (kEdvsResolution_V-1) - iter->y : iter->y;
      }
    }
  }

  void filterEventsByFrequency(const std::vector<Edvs::Event>& unfiltered_events, 
                               std::vector<Edvs::Event>* filtered_events,
                               std::vector<uint64_t>* tse1, std::vector<uint64_t>* tse2)
  {
    size_t idx;
    uint64_t t0, t1, t2, ts_diff1, ts_diff2;

    for(std::vector<Edvs::Event>::const_iterator iter = unfiltered_events.begin();
      iter != unfiltered_events.end(); ++iter)
    {
      idx = (*iter).x + kEdvsResolution_U * (*iter).y;
      t0 = static_cast<uint64_t>((*iter).t);
      t1 = static_cast<uint64_t>(tse1->at(idx));
      t2 = static_cast<uint64_t>(tse2->at(idx));
      ts_diff1 = abs( t0 - t1 - kFreqFilterDT );
      ts_diff2 = abs( t1 - t2 - kFreqFilterDT );
      
       //Adjust to the frequency of the LED with some "lag" range 
      if(ts_diff1 < kFreqFilterTolerance && ts_diff2 < kFreqFilterTolerance)
        filtered_events->push_back(*iter);

      tse2->at(idx) = tse1->at(idx);
      tse1->at(idx) = (*iter).t;
    }
  }

  Eigen::Vector3f unprojectEDVS(const DEvent& de)
  {
    return unprojectEDVS(de.u,de.v,de.depth);
  }

  Eigen::Vector3f unprojectKinect(const DEvent& de)
  {
    return unprojectKinect(de.x,de.y,de.depth);
  }

  void generateDepthMap(const uint16_t* depth_frame,
                        const PrimesenseToEdvsLUT& lut,
                        std::vector<dedvs::Point3D>* depth_map)
  {
    long int depth_idx;
    depth_map->assign(kEdvsResolution_U * kEdvsResolution_V, Point3D(0,0,10000));

    for(int y = 0; y < kXtionDepthResolutionY; ++y)
    {
      for(int x = 0; x < kXtionDepthResolutionX; ++x)
      {
        uint16_t depth = depth_frame[x + kXtionDepthResolutionX * y];

        if(depth < kMinDepth || depth >= kMaxDepth) continue;

        if(kDepthLUTResolution == 1)
          depth_idx = depth;
        else
          long int depth_idx = lround((depth-kMinDepth) / static_cast<float>(kDepthLUTResolution));

        int uv_pos = lut.at(x, y, depth_idx);

        dedvs::Point3D& q = depth_map->at(uv_pos);

        //Data might be overwritten due to incorrect projection by the NN, but we probably want the closest point.
        if(depth < q.depth) {
          q.x = x;
          q.y = y;
          q.depth = depth;
        }
      }
    }
  }

  std::vector<DEvent> augmentEvents(const std::vector<Point3D>& depth_map, const ctp last_depth_frame, 
                                    const ctp ts_init, std::vector<DEvent>* events)
  {
    uint16_t depth;
    DEvent temp_event;
    std::vector<DEvent> depth_events;

    if(depth_map.empty()) return depth_events;

    for(auto iter = events->begin(); iter != events->end(); ++iter)
    {
      //Check if this event happened before the last frame
      // if(iter->ct > last_depth_frame)//+ std::chrono::nanoseconds(1))
      // {
      //   if(iter != events->begin())
      //     events->erase(events->begin(),iter);
      //   break;
      // }

      depth = findNearestEdgeDepth(iter->u,iter->v,depth_map);
      //depth = depth_map[iter->u + kEdvsResolution_U * iter->v].depth;

      if(depth < kMinDepth || depth >= kMaxDepth) continue;

      if(iter->time == 0 || iter->time == 8) //TODO IT SHOULD BE ZERO NOT EIGHT!
        temp_event.time = static_cast<uint64_t>(
                  (std::chrono::duration_cast<std::chrono::microseconds>(iter->ct-ts_init)).count());
      else
      temp_event.time = iter->time;
        
      temp_event.u = iter->u;
      temp_event.v = iter->v;
      temp_event.parity = iter->parity;

      temp_event.x = depth_map[iter->u + kEdvsResolution_U * iter->v].x;
      temp_event.y = depth_map[iter->u + kEdvsResolution_U * iter->v].y;
      temp_event.depth = depth;

      temp_event.ct = iter->ct;

      if(temp_event.x == 0 && temp_event.y == 0) continue; //FIXME
      //TODO BUG FIXME HACK: Double-events are produced (possibly by the sensor)
      if( !depth_events.empty() &&
         temp_event.u == depth_events.back().u &&
         temp_event.v == depth_events.back().v 
         //&&         temp_event.ct == depth_events.back().ct
        )
      {
        continue;
      }

      depth_events.push_back(temp_event);
    }
    events->clear(); // ONLY IF THE IS NO CT FILTERING

    return depth_events;
  }

  uint16_t findNearestEdgeDepth(int u, int v, const std::vector<dedvs::Point3D>& depth_map)
  {
    constexpr unsigned U = kEdvsResolution_U;
    constexpr unsigned V = kEdvsResolution_V;

    uint16_t depth = depth_map[u + dedvs::kEdvsResolution_U * v].depth;

    uint16_t dmin = depth;

    bool has_invalid_neighbour = false;

    if(1 <= u && u < U && 0 <= v && v < V) {
      uint16_t d = depth_map[(u-1)+(v  )*U].depth;
      if(d >= 10000) {
        has_invalid_neighbour = true;
      }
      dmin = std::min(dmin, d);
    }

    if(0 <= u && u+1 < U && 0 <= v && v < V) {
      uint16_t d = depth_map[(u+1)+(v  )*U].depth;
      if(d >= 10000) {
        has_invalid_neighbour = true;
      }
      dmin = std::min(dmin, d);
    }

    if(0 <= u && u < U && 1 <= v && v < V) {
      uint16_t d = depth_map[(u  )+(v-1)*U].depth;
      if(d >= 10000) {
        has_invalid_neighbour = true;
      }
      dmin = std::min(dmin, d);
    }

    if(0 <= u && u < U && 0 <= v && v+1 < V) {
      uint16_t d = depth_map[(u  )+(v+1)*U].depth;
      if(d >= 10000) {
        has_invalid_neighbour = true;
      }
      dmin = std::min(dmin, d);
    }

    if(has_invalid_neighbour) {
      return 10000;
    }

    if(std::abs(depth - dmin) > kThreshold) 
    {
      return dmin;
    } 
    else {
      return depth;
    }
  }

  PrimesenseToEdvsLUT loadDepthLUT(const std::string& path_to_lut)
  {
    PrimesenseToEdvsLUT lut;
    lut.create(dedvs::kXtionDepthResolutionX, dedvs::kXtionDepthResolutionY, dedvs::kDepthLUTEntries);
    std::cout << "Loading Depth-LUT into memory...";
    std::ifstream ifs(path_to_lut, std::ios::in | std::ios::binary);
    ifs.read((char*)lut.ptr.data(),
            dedvs::kXtionDepthResolutionX * dedvs::kXtionDepthResolutionY * 
            dedvs::kDepthLUTEntries * sizeof(uint16_t));
    std::cout << "  Done!" << std::endl;
    return lut;
  }


  void getTimeString(char* buffer)
  {
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime (buffer,200,"-%F-%X",timeinfo);
  }

  Edvs::EventStream* setupEdvs()
  {
    return (new Edvs::EventStream(kEdvsURI));
  }

  void flushEventsFromStream(Edvs::EventStream *event_stream)
  {
    const std::size_t num = 2048;
    std::vector<Edvs::Event> flush_buffer(2048);
    while(true) {
        //flush_buffer.resize(num);
        event_stream->read(flush_buffer);
        if(flush_buffer.size() < 100) break;
    }
  }

  DEvent createDEvent(const Edvs::Event &event, uint16_t depth)
  {
    DEvent depth_event;
    depth_event.u = event.x;
    depth_event.v = event.y;
    depth_event.depth  = depth;
    depth_event.time   = event.t;
    depth_event.parity = event.parity;
    return depth_event;
  }

  DEventExt createDEvent(const Edvs::Event &event, const Point3D &point)
  {
    DEventExt depth_event;
    depth_event.u = event.x;
    depth_event.v = event.y;
    depth_event.x = static_cast<float>(point.x);
    depth_event.y = static_cast<float>(point.y);
    depth_event.depth  = point.depth;
    depth_event.time   = event.t;
    depth_event.parity = event.parity;
    return depth_event;
  }

  DEventExt createDEvent(const Edvs::Event &event, const Point3DF &point)
  {
    DEventExt depth_event;
    depth_event.u = event.x;
    depth_event.v = event.y;
    depth_event.x = point.x;
    depth_event.y = point.y;
    depth_event.depth  = point.depth;
    depth_event.time   = event.t;
    depth_event.parity = event.parity;
    return depth_event;
  }

  DEventExtF createDEventF(const Edvs::Event &event, const Point3DF &point)
  {
    DEventExtF depth_event;
    depth_event.u = event.x;
    depth_event.v = event.y;
    depth_event.x = point.x;
    depth_event.y = point.y;
    depth_event.depth  = point.depth;
    depth_event.time   = event.t;
    depth_event.parity = event.parity;
    return depth_event;
  }

  void createDEvent(const Edvs::Event &event, const Point3DF &point, DEventExt* depth_event)
  {
    depth_event->u = event.x;
    depth_event->v = event.y;
    depth_event->x = point.x;
    depth_event->y = point.y;
    depth_event->depth  = point.depth;
    depth_event->time   = event.t;
    depth_event->parity = event.parity;
  }


  void writeDepthFrameToBinaryFile(const std::string path_to_output, const uint16_t* depth_frame)
  {
    static boost::format fmt("/depth_%05d");
    static int i = 0;
    std::fstream file(path_to_output+(fmt%i).str(), std::ios::out | std::ios::binary);
    file.write((char*) depth_frame, dedvs::kXtionDepthResolutionX * dedvs::kXtionDepthResolutionY * sizeof(uint16_t));
    file.close();
    i++;
  }

  void writeRGBFrameToBinaryFile(const std::string path_to_output, const openni::RGB888Pixel* rgb_frame)
  {
    static boost::format fmt("/rgb_%05d");
    static int i = 0;
    std::fstream file(path_to_output+(fmt%i).str(), std::ios::out | std::ios::binary);
    file.write((char*) rgb_frame, dedvs::kXtionColorResolutionX * dedvs::kXtionColorResolutionY * sizeof(openni::RGB888Pixel));
    file.close();
    i++;
  }

  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEvent>& depth_events)
  {
    std::ofstream file(filename, std::ios::out | std::ios::app);

    for(std::vector<DEvent>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      file  << (*iter).x << "\t"
            << (*iter).y << "\t"
            << (*iter).depth << "\t"
            << (*iter).u << "\t"
            << (*iter).v << "\t"
            << (*iter).time << "\t"
            << static_cast<int>((*iter).parity) << "\n";
    }

    file.close();
  }

  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEventExt>& depth_events)
  {
    std::ofstream file(filename, std::ios::out | std::ios::app);


    for(std::vector<DEventExt>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      file  << (*iter).x << "\t"
            << (*iter).y << "\t"
            << (*iter).depth << "\t"
            << (*iter).u << "\t"
            << (*iter).v << "\t"
            << (*iter).time << "\t"
            << static_cast<int>((*iter).parity) << "\n";
    }

    file.close();
  }

  void writeDEventsToTVSFile(const std::string filename, const std::vector<DEventExtF>& depth_events)
  {
    std::ofstream file(filename, std::ios::out | std::ios::app);
    std::cout << filename << std::endl;
    for(std::vector<DEventExtF>::const_iterator iter = depth_events.cbegin();
        iter != depth_events.cend(); ++iter)
    {
      file  << (*iter).x << "\t"
            << (*iter).y << "\t"
            << (*iter).depth << "\t"
            << (*iter).u << "\t"
            << (*iter).v << "\t"
            << (*iter).time << "\t"
            << static_cast<int>((*iter).parity) << "\n";
    }

    file.close();
  }

  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEvent>* depth_events)
  {
    DEvent temp;
    depth_events->clear();
    depth_events->reserve(100000);
    int count = 0;

    FILE * infile = fopen(filename,"r");
    while(fscanf(infile,"%hu\t%hu\t%hu\t%lu\t%hhu\n",
                 &temp.u,&temp.v,&temp.depth,&temp.time,&temp.parity) != EOF)
    {
      depth_events->push_back(temp);
      ++count;
    }

    fclose(infile);
    return count;
  }

  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEventExt>* depth_events)
  {
    DEventExt temp;
    depth_events->clear();
    depth_events->reserve(100000);
    int count = 0;

    FILE * infile = fopen(filename,"r");
    while(fscanf(infile,"%f\t%f\t%hu\t%hu\t%hu\t%lu\t%hhu\n",
                 &temp.x,&temp.y,&temp.depth,&temp.u,&temp.v,&temp.time,&temp.parity) != EOF)
    {
      depth_events->push_back(temp);
      ++count;
    }

    fclose(infile);
    return count;
  }

  int readDEventsFromFile(const char* filename, std::vector<dedvs::DEventExtF>* depth_events_float)
  {
    FILE * infile = fopen(filename,"r");

    DEventExtF temp;
      depth_events_float->clear();
      depth_events_float->reserve(100000);

    int count = 0;
    while(fscanf(infile,"%f\t%f\t%f\t%f\t%f\t%lu\t%hhu\n",
          &temp.x,&temp.y,&temp.depth,&temp.u,&temp.v,&temp.time,&temp.parity) != EOF)
    {
      depth_events_float->push_back(temp);
      ++count;
    }

    fclose(infile);
    return count;
  }
}