/*
 * Skeletonizer.h
 *
 *  Created on: 2015
 *      Author: simon
 */
#pragma once

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif


#include <toffy/tracers/segmentTracer.hpp> // for Segments


/**
 *  base class for all skeletonizers
 */

class Pruner {
public:
	Pruner();
	virtual ~Pruner();

	/** prune segments - this marks all segments as prunable that end within
	 * the radius of a crossing point. It does not delete segments.
	 */
	virtual void prune(const Segments& in, Segments& out);

	//virtual void configure(const rapidjson::Value& configObject) = 0;

	int slack; //< constant number of pixels added to the threshold (typ. 0);
	double factor; //< multiplication factor that the thickness is multiplied with (typ 1.0)

	bool pruneBorderPoints; //< keep or prune segments that touch the image border
protected:
	//Debuging switch
	bool Debugging;
};
