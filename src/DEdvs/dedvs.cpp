#include "dedvs.h"
#include "dedvs_impl.h"

#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <chrono>


#include <boost/format.hpp>

//OpenNI
#include "OpenNI.h"

//EDVSTOOLS
#include "Edvs/Event.hpp"
#include "Edvs/EventStream.hpp"

//XTION OPENCV INTEROPERABILITY LIB 
#include "DEdvs/config.h"
#include "DEdvs/xtion_io.h"
#include "DEdvs/opencv_iop.h"
#include "DEdvs/dedvs_auxiliary.h"


namespace dedvs
{

void recordThread(dedvs_impl* h)
{
	while(h->thread_running)
	{
		h->cache.write_one(h->params.record_depth_path, h->params.record_rgb_path);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void readEventStream(dedvs_impl* h)
{
	std::vector<Edvs::Event> buf_stream_events;
	std::vector<DEvent> depth_events;
	std::chrono::high_resolution_clock::time_point ts;
	std::vector<dedvs::DEvent> local_events;

	dedvs::flushEventsFromStream(h->event_stream);

	while(h->thread_running)
	{
		// read raw events from device
		buf_stream_events.clear();
		buf_stream_events.resize(1024);
		h->event_stream->read(buf_stream_events);
		ts = std::chrono::high_resolution_clock::now();

		// magic ... // TUNE UP if disabled prolly...just for visualization
		dedvs::transformEvents(&buf_stream_events);

		// generate events
		for(const Edvs::Event& e : buf_stream_events) {
			local_events.push_back(dedvs::DEvent(e, ts));
		}

		// try to process events
		h->depth_map_mtx.lock();
		depth_events = augmentEvents(h->depth_map, h->last_depth_frame, h->ts_init, &local_events);
		h->depth_map_mtx.unlock();

		// copy blah
		h->events_mtx.lock();
		for(DEvent e : depth_events) {
			pe q;
			q.devent = e;
			if(kXtionDepthResolutionX == 320 && 
			   kXtionDepthResolutionY == 240)
			{
				e.x *= 2;
				e.y *= 2;
			}
			q.p = unprojectKinect(e);
			q.s = 0.001f * static_cast<float>(e.depth) / kKinectFocalPX;
			h->events.push_back(q);
		}
		
		h->events_mtx.unlock();
	}
}

void updateColorFrame(dedvs_impl* h)
{
	while(h->thread_running)
	{
		dedvs::getColorFrame(h->vs_color, &h->color_frame);
		h->last_color_frame = std::chrono::high_resolution_clock::now();

		// add to cache for recording
		if(h->params.enable_record_rgb) {
			h->cache.add_rgb((uint8_t*)h->color_frame);
		}
	}
}

void updateDepthMap(dedvs_impl* h)
{
	//buffering since we are rather slow to update the lookuptable
	std::vector<dedvs::Point3D> depth_map_new;
	dedvs::ctp ts_start = std::chrono::high_resolution_clock::now();

	while(h->thread_running)
	{
		h->depth_frame_mtx.lock();
		dedvs::getDepthFrame(h->vs_depth, &h->depth_frame);
		h->last_depth_frame = std::chrono::high_resolution_clock::now();
		h->depth_frame_mtx.unlock();

		dedvs::generateDepthMap(h->depth_frame, h->lut, &depth_map_new);
		// std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(h->last_depth_frame - ts_start).count() << std::endl;

		h->depth_map_mtx.lock();
		h->depth_map = depth_map_new;
		h->depth_map_mtx.unlock();

		// add to cache for recording
		if(h->params.enable_record_rgb) {
			h->cache.add_depth(h->depth_frame);
		}
	}
}

dedvs_impl* start(const std::string& fn_lut, const parameters& params)
{
	dedvs_impl* h = new dedvs_impl();
	h->params = params;

	h->cache.size_depth = dedvs::kXtionDepthResolutionX * dedvs::kXtionDepthResolutionY;
	h->cache.size_rgb = dedvs::kXtionColorResolutionX * dedvs::kXtionColorResolutionY * sizeof(openni::RGB888Pixel);

	h->depth_frame = NULL;
	h->color_frame = NULL;

	h->device = new openni::Device();
	h->vs_depth = new openni::VideoStream();


	if(dedvs::intializeOpenNI() != dedvs::DEDVS_STATUS_OK)
		return NULL;

	if(dedvs::openXtionDevice(h->device) != dedvs::DEDVS_STATUS_OK)
		return NULL;

	if(dedvs::createStream(dedvs::DEPTH, h->device, h->vs_depth) != dedvs::DEDVS_STATUS_OK)
		return NULL;

	if(dedvs::startStream(h->vs_depth) != dedvs::DEDVS_STATUS_OK)
		return NULL;

	if(h->params.enable_rgb)
	{
		h->vs_color = new openni::VideoStream();
		if(dedvs::createStream(dedvs::COLOR, h->device, h->vs_color) != dedvs::DEDVS_STATUS_OK)
			return NULL;

		if(dedvs::startStream(h->vs_color) != dedvs::DEDVS_STATUS_OK)
			return NULL;
	}
	else
	{
		h->vs_color = NULL;
	}

  if(dedvs::setImageRegistrationMode(true, h->device) != dedvs::DEDVS_STATUS_OK) return NULL;

	h->event_stream = dedvs::setupEdvs();

	h->thread_running = true;

	h->lut = loadDepthLUT(fn_lut);

	h->ts_init = std::chrono::high_resolution_clock::now();

	h->read_events_thread = std::thread(readEventStream, h);

	h->update_depth_map_thread = std::thread(updateDepthMap, h);

	if(h->params.enable_rgb)
	{
		h->update_color_frame_thread = std::thread(updateColorFrame, h);
	}

	if(h->params.enable_record_depth || h->params.enable_record_rgb)
	{
		h->record_thread = std::thread(recordThread, h);
	}

	return h;
}

std::vector<pe> get_events(dedvs_impl* h)
{
	h->events_mtx.lock();
	std::vector<pe> tmp = h->events;
	h->events.clear();
	h->events_mtx.unlock();
	return tmp;
}

void get_events(dedvs_impl* h, std::vector<pe>* events_out)
{
	h->events_mtx.lock();
	*events_out = h->events;
	h->events.clear();
	h->events_mtx.unlock();
}

void stop(dedvs_impl* h)
{
	h->thread_running = false;

	h->read_events_thread.join();
	h->update_depth_map_thread.join();
	if(h->update_color_frame_thread.joinable())
		h->update_color_frame_thread.join();
	if(h->record_thread.joinable())
		h->record_thread.join();

	//In order: Close and destroy streams, shutdown device, and unload drivers
	dedvs::shutdownVideoStream(h->vs_depth);
	if(h->params.enable_rgb)
	{
		dedvs::shutdownVideoStream(h->vs_color);
		delete h->vs_color;
	}
	dedvs::shutdownDevice(h->device);
	dedvs::shutdownOpenNI();

	// Close EDVS event stream
	h->event_stream->close();

	delete h->vs_depth;
	delete h->device;
	delete h->event_stream;
	delete h;
}

}
