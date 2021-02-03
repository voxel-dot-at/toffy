#include "toffy/tracers/segments.hpp"
#include <iostream>
using namespace std;

bool Segments::contains(const RasterPoint* start, const RasterPoint* end) const
{
    for (unsigned int i=0;i<size();i++) {
	RasterSegment* seg = (*this)[i];
	if ( ( seg->start == start && seg->end == end )
	     || ( seg->start == end && seg->end == start ) )
	    return true;
    }
    return false;
}
