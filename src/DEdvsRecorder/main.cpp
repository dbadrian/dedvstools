#include <iostream>

#include <boost/format.hpp>

#include "DEdvs/config.h"
#include "DEdvs/dedvs.h"
#include "DEdvs/dedvs_impl.h"

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

//OPENCV2
#include "core/core.hpp"
#include "imgproc/imgproc.hpp"
#include "contrib/contrib.hpp"
#include "highgui/highgui.hpp"

#include "DEdvs/opencv_iop.h"

namespace po = boost::program_options;
namespace bfs =  boost::filesystem;

struct prg_args {
  bfs::path fn_lut;
  bfs::path fn_store;
  std::string fn_setname;
  bool enable_rgb;
  bool enable_vis;
  bool dryrun;  
};

bool checkArgs(prg_args args)
{
  if(exists(args.fn_lut))    // does p actually exist?
  {
    if(!is_regular_file(args.fn_lut))
    {
      std::cerr << args.fn_lut << " does not seem to be a correct file.\n";
      return false;
    }
  }
  else
  {
    std::cerr << args.fn_lut << " does not seem to exist\n";
    return false;
  }

  if(!args.dryrun)
  {  
    if(exists(args.fn_store))    // does p actually exist?
    {
      if(!is_directory(args.fn_store))
      {
        std::cerr << args.fn_store << " does not seem to be a correct folder.\n";
        return false;
      }
    }  
    else
    {
      std::cerr << args.fn_store << " does not seem to exist\n";
      return false;
    }
  }

  return true;
}

void recorder(prg_args args)
{
  dedvs::parameters params;
  params.enable_rgb = args.enable_rgb;
  std::string fn_store_str = args.fn_store.native();
  params.enable_record_depth = !fn_store_str.empty();
  params.record_depth_path = fn_store_str + "/depth/";
  params.enable_record_rgb = !fn_store_str.empty();
  params.record_rgb_path = fn_store_str + "/rgb/";
  dedvs::dedvs_impl* impl = dedvs::start(args.fn_lut.native(), params);

  std::vector<dedvs::DEvent> vis_depth_events;  // Events will be buffered for visualisation

  // We only want to write a new frame to the harddrive, if the TS changed
  auto last_depth_frame_ts = std::chrono::high_resolution_clock::now();
  auto last_color_frame_ts = std::chrono::high_resolution_clock::now();


  //////////////////////////// Visualisation Data Allocation ////////////////////////////

  constexpr int SIZE = 128;                   // Size of windows
  constexpr int EVENT_DISPLAY_DURATION = 25;  // Lag till an event is no longer visualised

  //OpenCV Mats for visualisation of Events, Depth Frame, and Depth Mat
  cv::Mat depth_frame_mat, depth_frame_col_mat;
    cv::Mat color_frame;
  cv::Mat depth_map_mat(dedvs::kEdvsResolution_U,dedvs::kEdvsResolution_V,CV_8UC3,cv::Scalar(128,128,128));
  cv::Mat event_frame(dedvs::kEdvsResolution_U,dedvs::kEdvsResolution_V,CV_8UC3,cv::Scalar(128,128,128));
  cv::Mat resized_events_frame(SIZE,SIZE,CV_8UC3,cv::Scalar(128,128,128));
  cv::Mat resized_dmap_frame(SIZE,SIZE,CV_8UC3,cv::Scalar(128,128,128));
  cv::Mat resized_depth_frame(SIZE,SIZE,CV_8UC3,cv::Scalar(128,128,128));
  cv::Mat resized_rgb_frame(SIZE,SIZE,CV_8UC3,cv::Scalar(128,128,128));  

  cv::namedWindow("Events",CV_WINDOW_AUTOSIZE);
  if(args.enable_rgb)
  {
    cv::namedWindow("Color",CV_WINDOW_AUTOSIZE);
  }
  cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("DepthMap",CV_WINDOW_AUTOSIZE);

  ///////////////////////////////////////////////////////////////////////////////////////

  //Ensures that the spawned threads fill in data
  std::this_thread::sleep_for(std::chrono::milliseconds(500));


  std::vector<dedvs::DEvent> buf_depth_events;          // WIll be written to harddrive
  ////////////////////////////////////// Main Loop //////////////////////////////////////
  while(true)
  {    
    // get events
    std::vector<dedvs::pe> vpe = dedvs::get_events(impl); // New point events

    // Better be safe, than sorry
    if(impl->depth_frame == NULL)
      continue;
    if(impl->depth_map.empty())
      continue;
    if(args.enable_rgb && impl->color_frame == NULL)
      continue;

    // Copy over new DEvents
    for(const dedvs::pe& x : vpe) {
      vis_depth_events.push_back(x.devent);
      buf_depth_events.push_back(x.devent);
    }
    std::cout << "Number of depth events: " << vpe.size() << std::endl;

    if(args.enable_vis)
    {
      // visualize depth image
      dedvs::getDepthFrameCVMat(impl->depth_frame, 
                                dedvs::kXtionDepthResolutionX*dedvs::kXtionDepthResolutionY*sizeof(uint16_t), 
                                &depth_frame_mat);

      dedvs::colorizeDepthCVMat(depth_frame_mat, &depth_frame_col_mat);
      cv::resize(depth_frame_col_mat,resized_depth_frame,cv::Size(SIZE*1.333333333,SIZE),0,0,cv::INTER_NEAREST);            
      imshow("Depth", resized_depth_frame);

      if(args.enable_rgb) {
        dedvs::getColorFrameCVMat(impl->color_frame, 
                            dedvs::kXtionColorResolutionX*dedvs::kXtionColorResolutionY*sizeof(uint8_t)*3, 
                            &color_frame);
        cv::resize(color_frame,resized_rgb_frame,cv::Size(SIZE*1.333333333,SIZE),0,0,cv::INTER_NEAREST);      
        imshow("Color", resized_rgb_frame);
      }

      dedvs::colorizeEvents(vis_depth_events, &event_frame);
      cv::resize(event_frame,resized_events_frame,cv::Size(SIZE,SIZE),0,0,cv::INTER_NEAREST);
      imshow("Events", resized_events_frame);

      // visualize depth mapping
      dedvs::getDepthMapCVMat(vis_depth_events, impl->depth_map, &depth_map_mat);
      cv::resize(depth_map_mat, resized_dmap_frame, cv::Size(SIZE,SIZE), 0,0, cv::INTER_NEAREST);
      imshow("DepthMap", resized_dmap_frame);

      // quit on keypress
      if(cv::waitKey(5) >= 0) {
        break;
      }
      // delete events from vis which are too old
      auto ts = std::chrono::high_resolution_clock::now();
      auto it_del = std::lower_bound(vis_depth_events.begin(), vis_depth_events.end(), ts, 
        [](const dedvs::DEvent& a, const dedvs::ctp& t) {
          return a.ct < t - std::chrono::milliseconds(EVENT_DISPLAY_DURATION);
        });
      // std::cout << vis_depth_events.back().ct.time_since_epoch().count() << " vs " << ts.time_since_epoch().count() << std::endl;
      vis_depth_events.erase(vis_depth_events.begin(), it_del);
    }
    else
    {
      vis_depth_events.clear();
    }

    // // write depth image to file
    // if(args.fn_store != "" && !args.dryrun) 
    // {
    //   if(impl->last_depth_frame != last_depth_frame_ts) 
    //   {
    //     //std::cout << "." << std::flush;
    //     dedvs::writeDepthFrameToBinaryFile(args.fn_store.native()+"/depth", impl->depth_frame);
    //     last_depth_frame_ts = impl->last_depth_frame;
    //   }

    //   // write color image to file
    //   if(args.enable_rgb) 
    //   {
    //     if(impl->last_color_frame != last_color_frame_ts)
    //     {
    //       //std::cout << "." << std::flush;
    //       dedvs::writeRGBFrameToBinaryFile(args.fn_store.native()+"/rgb", impl->color_frame);
    //     }
    //     last_color_frame_ts = impl->last_color_frame;
    //   }
    // }
  
    // writing our recorded data to the hd
  }

  dedvs::stop(impl);

  if(args.fn_store != "" && !args.dryrun) 
  {
    writeDEventsToTVSFile(args.fn_store.native() + std::string("/events.tsv"), buf_depth_events);
    std::cout << "Wrote " << buf_depth_events.size() << " events to file..." << std::endl;
  }
}

int main(int argc, char *argv[])
{
  prg_args rec_args;
  rec_args.fn_lut = "";
  rec_args.fn_store = "";
  rec_args.fn_setname = "";
  rec_args.enable_rgb = false;
  rec_args.enable_vis = false;
  rec_args.dryrun = false;  

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("dryrun,dr", po::value( &rec_args.dryrun )->zero_tokens(), "Optional: No data will be saved")
    ("visual,v", po::value( &rec_args.enable_vis )->zero_tokens(), "Optional: Enable visualization")    
    ("rgb,c", po::value( &rec_args.enable_rgb )->zero_tokens(), "Optional: Enable RGB stream")
    ("lut,l", po::value<bfs::path>(&rec_args.fn_lut), "Required: Path to the lookup table (GDCalib!)")    
    ("out_folder,o", po::value<bfs::path>(&rec_args.fn_store), "Required: Output Path (a set folder will be created)")
    ("set_name,s", po::value(&rec_args.fn_setname), "Required: Name of Set")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  //Invalid arguments
  if(!checkArgs(rec_args))
  {
    return 1;
  }

  if(!rec_args.dryrun)
  {
    rec_args.fn_store /= rec_args.fn_setname;
  
    if(!bfs::create_directory(rec_args.fn_store))
    {
      std::cerr << "Could not create directory " << rec_args.fn_store << std::endl;
      return 1;
    }

    if(!bfs::create_directory(rec_args.fn_store / "depth"))
    {
      std::cerr << "Could not create directory " << rec_args.fn_store  / "depth" << std::endl;
      return 1;
    }

    if(rec_args.enable_rgb)
    {
      if(!bfs::create_directory(rec_args.fn_store / "rgb"))
      {
        std::cerr << "Could not create directory " << rec_args.fn_store / "rgb" << std::endl;
        return 1;
      }
    }
  }

  recorder(rec_args);

  return 0;
}

