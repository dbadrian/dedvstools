#include <iostream>
#include <fstream>
#include <stdint.h>
#include <cmath>

#include "boost/timer/timer.hpp"
//#include "boost/progress.hpp"

#include <Eigen/Dense>

#include "DEdvs/config.h"
#include "DEdvs/dedvs_auxiliary.h"

void createLUT(uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                                                               [dedvs::kXtionDepthResolutionY]
                                                               [dedvs::kDepthLUTEntries]);
void caluculateProjection(float x, float y, float depth, float* u, float* v);


int main(int argc, char *argv[])
{
  boost::timer::auto_cpu_timer t;
  std::cout << "GDCALIB Lookup Table Creator" << std::endl;
  //// Command line arguments
  if(argc != 2)
  {
    std::cout << "Wrong Usage" << std::endl;
    std::cout << "gdcalib_lookup_creator <path_to/dest_lut.bin>" << std::endl;
    return 0;
  }

  static uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                           [dedvs::kXtionDepthResolutionY]
                           [dedvs::kDepthLUTEntries];

  createLUT(depth_lut);

  std::cout << std::endl  << "Writing file...";

  std::fstream file(argv[1], std::ios::out | std::ios::binary);
  file.write((char*) &depth_lut[0][0][0],
              dedvs::kXtionDepthResolutionX * dedvs::kXtionDepthResolutionY * 
              dedvs::kDepthLUTEntries * sizeof(uint16_t));
  file.close();

  std::cout << " Done!" << std::endl;;

  return 0;
}

void createLUT(uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                               [dedvs::kXtionDepthResolutionY]
                               [dedvs::kDepthLUTEntries])
{
  long int u_idx, v_idx, d_idx;
  float u,v;

  for(int x = 0; x < dedvs::kXtionDepthResolutionX; ++x)
  {
    for(int y = 0; y < dedvs::kXtionDepthResolutionY; ++y)
    {
    	for(int d = dedvs::kMinDepth; d < dedvs::kMaxDepth; d += dedvs::kDepthLUTResolution)
      {

        dedvs::projectEDVS(dedvs::unprojectKinect(x,y,d),&u,&v);

	  		u_idx = lround(u);
	  		v_idx = lround(v);
	        //Values should be between [0,127] 
	    	if(u_idx < 0) u_idx = 0;
        if(u_idx >= dedvs::kEdvsResolution_U) u_idx = dedvs::kEdvsResolution_U - 1;
        if(v_idx < 0) v_idx = 0;
        if(v_idx >= dedvs::kEdvsResolution_V) v_idx = dedvs::kEdvsResolution_V - 1;

        d_idx = lround((d-dedvs::kMinDepth)/static_cast<float>(dedvs::kDepthLUTResolution));
        depth_lut[x][y][d_idx] = static_cast<uint16_t>(u_idx + dedvs::kEdvsResolution_U * v_idx);
    	}
    }
   	if(x%16==0)
   		std::cout << 100.0 * x/static_cast<float>(dedvs::kXtionDepthResolutionX) << "%" << std::endl;
  }
}