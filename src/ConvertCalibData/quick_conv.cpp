#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <fstream> 
#include <iostream>
#include <cmath>
#include <string>

#include <boost/lexical_cast.hpp>

#include "DEdvs/config.h"
#include "DEdvs/dedvs_auxiliary.h"

void normalizeDEvents(std::vector<dedvs::DEventExtF>* depth_events_float);

int main(int argc, char *argv[])
{
  if(argc != 2)
  {
    std::cerr << "Wrong Usage" << std::endl;
    std::cerr << "./ConvertCalibData <path_to/calib-data_input>" << std::endl;
    return 0;
  } 

  std::string out_filename = std::string(std::string(argv[1]) +"-(" 
                            + boost::lexical_cast<std::string>(dedvs::kXtionColorResolutionX) +"-"
                            + boost::lexical_cast<std::string>(dedvs::kXtionColorResolutionY) +"-"
                            + boost::lexical_cast<std::string>(dedvs::kMaxDepth)+")-FANN.data");

  std::cout << "Output will be called: " << out_filename << std::endl;

  std::vector<dedvs::DEventExtF> depth_events;

  int count = dedvs::readDEventsFromFile(argv[1], &depth_events);
  std::cout << "# " << count << " Training Points Read!" << std::endl;

  std::cout << "Normalizing events..." << std::endl;
  normalizeDEvents(&depth_events);

  std::cout << "Writing FANN compatible training data..." << std::endl;
  dedvs::writeDEventsToFANNFile(out_filename, depth_events);
  
  std::cout << " Done!" << std::endl;

  return 0;
}

void normalizeDEvents(std::vector<dedvs::DEventExtF>* depth_events_float)
{
  float u,v;
  for(std::vector<dedvs::DEventExtF>::iterator iter = depth_events_float->begin();
      iter != depth_events_float->end(); ++iter)
  {
    (*iter).x = (*iter).x/static_cast<float>(dedvs::kXtionColorResolutionX);
    (*iter).y = (*iter).y/static_cast<float>(dedvs::kXtionColorResolutionY);
    (*iter).depth = (*iter).depth/static_cast<float>(dedvs::kMaxDepth);
    u = (*iter).u;
    v = (*iter).v;
    (*iter).u = v;
    (*iter).v = u;
  }
}