#ifndef INCLUDED_DEDVS_DEDVSIMPL
#define INCLUDED_DEDVS_DEDVSIMPL

#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>

//OpenNI
#include "OpenNI.h"

//EDVSTOOLS
#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

//XTION OPENCV INTEROPERABILITY LIB 
#include "DEdvs/config.h"
#include "DEdvs/dedvs_auxiliary.h"

namespace dedvs
{

	struct record_cache
	{
		size_t size_depth;
		size_t size_rgb;
		std::queue<uint16_t*> v_depth;
		std::queue<uint8_t*> v_rgb;
		std::mutex mtx;

		void add_rgb(const uint8_t* p) {
			uint8_t* ptr = new uint8_t[size_rgb];
			std::copy(p, p + size_rgb, ptr);
			mtx.lock();
			v_rgb.push(ptr);
			mtx.unlock();
		}

		void add_depth(const uint16_t* p) {
			uint16_t* ptr = new uint16_t[size_depth];
			std::copy(p, p + size_depth, ptr);
			mtx.lock();
			v_depth.push(ptr);
			mtx.unlock();
		}

		void write_one(const std::string& path_depth, const std::string& path_rgb) {
			// get frames
			uint16_t* pd = 0;
			uint8_t* pc = 0;
			mtx.lock();
			if(!v_depth.empty()) {
				pd = v_depth.front();
				v_depth.pop();
			}
			if(!v_rgb.empty()) {
				pc = v_rgb.front();
				v_rgb.pop();
			}
			//std::cout << "cache: " << v_depth.size() << " " << v_rgb.size() << std::endl;
			mtx.unlock();
			// write frames and free memory
			if(pd) {
				//std::cout << "Writing depth frame..." << std::endl;
				writeDepthFrameToBinaryFile(path_depth, pd);
				delete[] pd;
			}
			if(pc) {
				//std::cout << "Writing color frame..." << std::endl;
				writeRGBFrameToBinaryFile(path_rgb, (const openni::RGB888Pixel*)pc);
				delete[] pc;
			}
		}

	};

	struct dedvs_impl
	{
		parameters params;

		openni::Device* device;
		openni::VideoStream* vs_depth;
		openni::VideoStream* vs_color;
		Edvs::EventStream* event_stream;

		PrimesenseToEdvsLUT lut;

		std::atomic<bool> thread_running;

		std::mutex depth_frame_mtx;
		std::mutex depth_map_mtx;

		std::vector<dedvs::Point3D> depth_map;

		dedvs::ctp last_depth_frame;
		dedvs::ctp last_color_frame;
		dedvs::ctp ts_init;		

		const uint16_t* depth_frame;
		const openni::RGB888Pixel* color_frame;

		std::mutex events_mtx;
		std::vector<pe> events;

		std::thread read_events_thread;
		std::thread update_depth_map_thread;
		std::thread update_color_frame_thread;
		std::thread record_thread;

		record_cache cache;

	};

}

#endif
