/*
 * Segments.h
 *
 *  Created on: Dec 13, 2012
 *      Author: simon
 */
#pragma once

#include<vector>
#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif
#include "toffy/graphs/graph.hpp"
#include "toffy/skeletonizers/skeletonizerFactory.hpp"
//#include "rapidjson/document.h"

#include "toffy/tracers/segments.hpp"

/** a collection of skeleton segments (which are connected typically)
 */
class Segments: public std::vector<RasterSegment*> {
 public:
	virtual void points(std::vector<RasterPoint*>& points) const
	{
		for (unsigned int i=0;i<size();i++) {
			(*this)[i]->points(points);
		}
	}


	void addUnique(RasterSegment* seg) {
	    Segments& s=*this;
	    for (size_t i=0;i<s.size();i++) {
		if ( ( s[i]->start == seg->start && s[i]->end == seg->end)
		     || ( s[i]->start == seg->end && s[i]->end == seg->start))
		    return;
	    }
	    push_back(seg);
	}

	virtual bool contains(const RasterPoint* start, const RasterPoint* end) const;
};
