#include <iostream>
#include <fstream>
#include <stdint.h>

#include "fann.h"

#include "DEdvs/config.h"

void createLUT(const char* path_to_fann_net, uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                                                               [dedvs::kXtionDepthResolutionY]
                                                               [dedvs::kDepthLUTEntries]);

int main(int argc, char *argv[])
{
  std::cout << "FANN/EDVS Lookup Table Creator" << std::endl;
  //// Command line arguments
  if(argc != 3)
  {
    std::cout << "Wrong Usage" << std::endl;
    std::cout << "nncalib_lookup_creator <path_to/calib.net> <path_to/dest_lut.bin>" << std::endl;
    return 0;
  }

  static uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                           [dedvs::kXtionDepthResolutionY]
                           [dedvs::kDepthLUTEntries];

  createLUT(argv[1],depth_lut);

  std::cout << std::endl  << "Writing file...";

  std::fstream file(argv[2], std::ios::out | std::ios::binary);
  file.write((char*) &depth_lut[0][0][0],
              dedvs::kXtionDepthResolutionX * dedvs::kXtionDepthResolutionY * 
              dedvs::kDepthLUTEntries * sizeof(uint16_t));
  file.close();

  std::cout << " Done!" << std::endl;;

  return 0;
}

void createLUT(const char* path_to_fann_net, uint16_t depth_lut[dedvs::kXtionDepthResolutionX]
                                                               [dedvs::kXtionDepthResolutionY]
                                                               [dedvs::kDepthLUTEntries])
{
  struct fann *ann = fann_create_from_file(path_to_fann_net);

  // FANN requires special types to work on
  fann_type *xyd_input = (fann_type *) malloc(3 * sizeof(fann_type));
  fann_type *uv_output = (fann_type *) malloc(2 * sizeof(fann_type));
  int u_idx, v_idx, d_idx;

  for(int x = 0; x < dedvs::kXtionDepthResolutionX; ++x)
  {
    for(int y = 0; y < dedvs::kXtionDepthResolutionY; ++y)
    {
      for(int d = dedvs::kMinDepth; d < dedvs::kMaxDepth; d += dedvs::kDepthLUTResolution)
      {
        //Setting up the input data
        xyd_input[0] = static_cast<float>(x) / static_cast<float>(dedvs::kXtionDepthResolutionX);
        xyd_input[1] = static_cast<float>(y) / static_cast<float>(dedvs::kXtionDepthResolutionY);
        xyd_input[2] = static_cast<float>(d) / static_cast<float>(dedvs::kMaxDepth);

        //Reading out the output data
        uv_output = fann_run(ann,xyd_input);
        u_idx = static_cast<int>(lround(uv_output[0]));
        v_idx = static_cast<int>(lround(uv_output[1]));

        // v_idx = static_cast<int>(lround(uv_output[0])); //TO USE OLD ANNs
        // u_idx = static_cast<int>(lround(uv_output[1]));

        //Values should be between [0,127] 
        if(u_idx < 0) u_idx = 0;
        if(u_idx >= dedvs::kEdvsResolution_U) u_idx = dedvs::kEdvsResolution_U - 1;
        if(v_idx < 0) v_idx = 0;
        if(v_idx >= dedvs::kEdvsResolution_V) v_idx = dedvs::kEdvsResolution_V - 1;

        d_idx = lround((d-dedvs::kMinDepth)/static_cast<float>(dedvs::kDepthLUTResolution));
        depth_lut[x][y][d_idx] = u_idx + dedvs::kEdvsResolution_U * v_idx;
      }
    }
    std::cout << 100.0 * x/static_cast<float>(dedvs::kXtionDepthResolutionX) << "%" << std::endl;
  }
  fann_destroy(ann);
}