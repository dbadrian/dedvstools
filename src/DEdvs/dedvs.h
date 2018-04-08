#ifndef INCLUDED_DEDVS_DEDVS
#define INCLUDED_DEDVS_DEDVS

#include "DEdvs/config.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>

namespace dedvs
{

	struct dedvs_impl;

	struct parameters
	{
		bool enable_rgb;
		bool enable_record_depth;
		std::string record_depth_path;
		bool enable_record_rgb;
		std::string record_rgb_path;
	};

	struct pe
	{
		dedvs::DEvent devent;
		Eigen::Vector3f p; // 3D position
		float s; // uncertainty
	};

	dedvs_impl* start(const std::string& fn_lut, const parameters& params);

	void get_events(dedvs_impl* h, std::vector<pe>* events_out);
	std::vector<pe> get_events(dedvs_impl* h);

	void stop(dedvs_impl* h);

}

#endif 
