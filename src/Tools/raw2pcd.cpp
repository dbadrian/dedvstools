#include <iostream>
#include <thread>
#include <atomic>
//#include <fstream>


#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
// #include "boost/timer/timer.hpp"

#include <Eigen/Dense>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/print.h>

#include "DEdvs/dedvs_auxiliary.h"

namespace bfs =  boost::filesystem;

bool checkPathValidity(bfs::path in_path)
{
  if(bfs::exists(in_path))
  {
    if(!bfs::is_directory(in_path))
    {
      std::cerr << in_path << " does not seem to be a folder!\n";
      return false;
    }
  }
  else 
  {
    std::cerr << "Folder " << in_path << " does not seem to exist!\n";
    return false;      
  }
  // Folder exists
  return true;
}

bool validateFilename(const std::string& s)
{
   static const boost::regex e("depth_\\d{1,15}");
   return boost::regex_match(s, e);
}

pcl::PointCloud<pcl::PointXYZRGBA>
createPCD(bfs::path path_to_depth_frame, bool use_rgb, bfs::path out_path,
          int width, int height, int compression, std::atomic<int>* current_threads)
{
  //Load Depth Frame!
  const uint16_t* depth_frame = new uint16_t[width*height];
  const unsigned char* color_frame = new unsigned char[width*height*3];

  // load depth
  {
    bfs::ifstream ifs(path_to_depth_frame, std::ios::in | std::ios::binary);
    ifs.read((char*) depth_frame, width*height * sizeof(uint16_t));
  }

  // load color
  if(use_rgb) {
    std::string path_to_color_frame = path_to_depth_frame.string();
    boost::replace_all(path_to_color_frame, "depth", "rgb");
    {
      std::ifstream ifs(path_to_color_frame, std::ios::in | std::ios::binary);
      ifs.read((char*) color_frame, width*height * 3);
    }
  }

  // std::cout << path_to_depth_frame << std::endl;
  // std::cout << path_to_color_frame << std::endl;

  // Fill in the cloud data
  pcl::PointCloud<pcl::PointXYZRGBA> pc_depth;
  pc_depth.width    = width;
  pc_depth.height   = height;
  pc_depth.is_dense = false;
  pc_depth.points.resize (pc_depth.width * pc_depth.height);
  const float bad_point = std::numeric_limits<float>::quiet_NaN (); //TODO???!

  //Temporary data
  Eigen::Vector3f temp_point;
  uint16_t depth;
  uint32_t rgb;

  int mult = (width == 640) ? 1 : 2;

  for (int y = 0; y < pc_depth.height; ++y)
  {
    for (int x = 0; x < pc_depth.width; ++x)
    {
      size_t index = x + width * y;
      depth = depth_frame[index];
      if(use_rgb) {
        rgb = (static_cast<uint32_t>(255)<<24)
          + (static_cast<uint32_t>(color_frame[3*index + 2]) << 16)
          + (static_cast<uint32_t>(color_frame[3*index + 1]) << 8)
          + (static_cast<uint32_t>(color_frame[3*index + 0]));
      }
      else {
        rgb = 0;
      }
      auto& p = pc_depth.points[x + width * y];
      p.rgba = rgb;

      if(depth != 0 && !pcl_isnan(depth) && !pcl_isnan(depth))
      {
        temp_point = dedvs::unprojectKinect(mult*x, mult*y, depth);
        p.x = temp_point(0);
        p.y = temp_point(1);
        p.z = temp_point(2);
      }
      else
      {
        p.x = bad_point;
        p.y = bad_point;
        p.z = bad_point;
      }
    }
  }

  //Save Point Cloud File
  std::string out = out_path.native()+path_to_depth_frame.filename().native()+std::string(".pcd");

  switch(compression)
  {
    case 0: pcl::io::savePCDFileASCII(out, pc_depth); break;
    case 1: pcl::io::savePCDFileBinary(out, pc_depth); break;
    case 2: pcl::io::savePCDFileBinaryCompressed(out, pc_depth); break;
  }

  delete[] depth_frame;

  (*current_threads)--;
  return pc_depth;
}

int converter(std::string raw_path, bool use_rgb, std::string out_path, int width, int height, int threads, int compression)
{
  bfs::path raw_p(raw_path);
  bfs::path out_p(out_path);

  //Check if paths actually exist
  if(!checkPathValidity(raw_p) || !checkPathValidity(out_p))
    return 1;

  std::vector<bfs::path> all_files;
  std::vector<bfs::path> raw_files;

  copy(bfs::directory_iterator(raw_path), bfs::directory_iterator(), back_inserter(all_files));

  for (auto iter:all_files)
  {
    if(validateFilename(iter.filename().native()))
      raw_files.push_back(iter);
  }

  sort(raw_files.begin(),raw_files.end());

  std::atomic<int> current_threads(0);

  boost::progress_display prog(raw_files.size());
  //TODO: Only for debug output??
  for(auto iter:raw_files)
  {
    while(current_threads >= threads)
    {
      std::this_thread::sleep_for( std::chrono::milliseconds(20));
    }
    //createPCD(iter, out_path, width, height, compression, stdout_mtx);
    std::thread thread(createPCD, iter, use_rgb, out_path, width, height, compression, &current_threads);
    // std::cout << "Converting " << iter.filename() << " ...\n";
    ++prog;
    thread.detach();
    current_threads++;
  }
  while(current_threads > 0)
  {
    std::this_thread::sleep_for( std::chrono::milliseconds(20));
  }
  std::cout << "\n - Converted all depth frames! -\n\n";
}

int main(int argc, char *argv[])
{
  // boost::timer::auto_cpu_timer t;
  namespace po = boost::program_options;

  int width = 320;
  int height = 240;  
  int threads = 1;
  int compression = 0;
  std::string raw_path = "";
  std::string out_path = "";
  bool use_rgb = false;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("width,w", po::value<int>(&width), "Width of depth frames in Pixel")    
    ("height,h", po::value<int>(&height), "Width of depth frames in Pixel (640/320)")   
    ("threads,t", po::value(&threads), "How many threads shall be spawned")
    ("compression,c", po::value(&compression), "Compression: 0=ASCII 1=Binary 2=Binary Compressed")    
    ("raw_depth_frames_folder,raw", po::value(&raw_path), "Folder/Location of the raw depth frames")
    ("rgb", po::value(&use_rgb), "Wether to use rgb images")
    ("output_folder,out", po::value(&out_path), "Folder of for generated .pcd files")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  return converter(raw_path,use_rgb,out_path,width,height,threads,compression);  
}
