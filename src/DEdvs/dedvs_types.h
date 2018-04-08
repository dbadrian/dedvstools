#ifndef DEDVS_TYPES_H
#define DEDVS_TYPES_H

#include <cstdint>
#include <vector>
#include <chrono>

//EDVSTOOLS
#include "Edvs/Event.hpp"

namespace dedvs {

  typedef std::chrono::high_resolution_clock::time_point ctp; 

  struct StreamInfo {
    int  x_resolution;
    int  y_resolution;
    int  fps;
    bool mirroring_enabled;
  };

  struct DEvent {
    uint16_t u, v;
    uint16_t x, y;
    uint16_t depth;
    uint64_t time;
    uint8_t  parity;
    ctp ct;

    void operator= (const DEvent &b)
    {
      u = b.u;
      v = b.v;
      x = b.x;
      y = b.y;
      depth   = b.depth;
      time    = b.time;
      parity  = b.parity;
      ct = b.ct;
    }

    DEvent() {};
    DEvent(const Edvs::Event &b, const ctp &ct_in)
    {
      u = b.x;
      v = b.y;
      time    = b.t;
      parity  = b.parity;
      ct = ct_in;
    }
  };

  struct DEventExt {
    uint16_t u;      // X-coordinate of the event
    uint16_t v;         // Y-coordinate of the event
    float x;    // X-coordinate of the led
    float y;    // Y-coordinate of the led
    uint16_t depth; // depth of the led
    uint64_t time;  // timestamp of the event
    uint8_t parity; // 0->1 1->0
    ctp ct;

    void operator= (const DEventExt &b)
    {
      u = b.u;
      v = b.v;
      x = b.x;
      y = b.y;
      depth   = b.depth;
      time    = b.time;
      parity  = b.parity;
      ct = b.ct;      
    }
  };

  /* Used to create the normalized FANN calibration material */
  struct DEventExtF {  
    float u;      // X-coordinate of the event
    float v;         // Y-coordinate of the event
    float x;    // X-coordinate of the led
    float y;    // Y-coordinate of the led
    float depth; // depth of the led
    uint64_t time;  // timestamp of the event
    uint8_t parity; // 0->1 1->0
    ctp ct;

    void operator= (const DEventExtF &b)
    {
      u = b.u;
      v = b.v;
      x = b.x;
      y = b.y;
      depth   = b.depth;
      time    = b.time;
      parity  = b.parity;
      ct = b.ct;      
    }
  };

  struct Point3D {
    uint16_t x;
    uint16_t y;
    uint16_t depth;

    void operator= (const Point3D &b)
    {
      x = b.x;
      y = b.y;
      depth = b.depth;
    }

    bool operator== (const Point3D &b) const
    {
      return(x == b.x &&
             y == b.y &&
             depth == b.depth
            );
    }


    Point3D() : x(0), y(0), depth(0) {}
    Point3D(uint16_t in_x, uint16_t in_y, uint16_t in_depth) : x(in_x), y(in_y), depth(in_depth) {}
  };

  struct Point3DF {
    float x;
    float y;
    float depth;

    void operator= (const Point3DF &b)
    {
      x = b.x;
      y = b.y;
      depth = b.depth;
    }

    Point3DF() : x(0.0), y(0.0), depth(0.0) {}
    Point3DF(float in_x, float in_y, float in_depth) : x(in_x), y(in_y), depth(in_depth) {}
  };

}

#endif