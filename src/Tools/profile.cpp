#include <iostream>

#include <boost/format.hpp>

#include "DEdvs/config.h"
#include "DEdvs/dedvs.h"
#include "DEdvs/dedvs_impl.h"

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

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

  // We only want to write a new frame to the harddrive, if the TS changed
  auto last_depth_frame_ts = std::chrono::high_resolution_clock::now();
  auto last_color_frame_ts = std::chrono::high_resolution_clock::now();


  //////////////////////////// Visualisation Data Allocation ////////////////////////////

  constexpr int SIZE = 256;                   // Size of windows
  constexpr int EVENT_DISPLAY_DURATION = 25;  // Lag till an event is no longer visualised

  ///////////////////////////////////////////////////////////////////////////////////////

  //Ensures that the spawned threads fill in data
  std::this_thread::sleep_for(std::chrono::milliseconds(500));


  std::vector<dedvs::DEvent> buf_depth_events;          // Will be written to harddrive
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
      buf_depth_events.push_back(x.devent);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(buf_depth_events.size() >= 500000)
      break;
  }

  dedvs::stop(impl);
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