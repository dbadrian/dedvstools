#include <iostream>
#include <string>

#include "record_calib_data.h"

#include "DEdvs/config.h"
#include "DEdvs/dedvs_auxiliary.h"

int main(int argc, char *argv[])
{
    char time_string[200];
    dedvs::getTimeString(time_string);

    std::string file_path("calib_data" + 
                          std::string(time_string));
    // if(argc == 1)
    // {
      // file_path = std::string("calib_data/calib_data" + std::string(time_string));
    // }
    // else if(argc == 2)
    // {
    //   file_path = std::string(std::string(argv[1])+ "/calib_data" + std::string(time_string));
    // }
    // else
    // {
    //   std::cerr << "Wrong Usage" << std::endl;
    //   std::cerr << "RecordCalibData <optional/path_to_calib_data>" << std::endl;
    //   return 0;
    // }
  
  std::cout << "Calibration Data will be saved at: " << file_path << std::endl;

  if(record_calib_data::recordCalibrationData(file_path) != dedvs::DEDVS_STATUS_OK) 
    return 1;
  else
    return 0;
}





















