#include <toffy/tracers/pruner.hpp>


#include <opencv2/imgproc/types_c.h>
#if OCV_VERSION_MAJOR >= 3
#include <opencv2/highgui.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include <iostream>

using namespace cv;
using namespace std;
//using namespace rapidjson;

Pruner::Pruner() : slack(0), factor(1.0)
{
	Debugging = true;
}

Pruner::~Pruner() 
{
}

const bool dbg=false;

void Pruner::prune(const Segments& in, Segments& out)
{
    vector<RasterPoint*> pts;
    in.points(pts);

    if (dbg) cout << "PRUNING " << in.size() << endl;    

    for(size_t i=0;i<pts.size();i++) {
        RasterPoint& rp = *pts[i];

        if (rp.type == inLine)
            continue;
        if (dbg) cout << "\trp " << rp << " " << rp.type << " " << rp.thick  << endl;

        if (rp.type == crossingPoint) {
            vector<RasterSegment*>::iterator segIter = rp.segs.begin();
            while (segIter!=rp.segs.end()) {
                RasterSegment& seg = **segIter;
                if (dbg) cout << "\t" << seg << " " << seg.isLeaf() << " " << seg.len <<  endl;
                if (seg.isLeaf() && seg.len <= (rp.thick * factor + slack)) {
                	seg.pruned = true;
			if (dbg) cout << "\t\tpruned " << rp << endl;

                } else {
                	seg.pruned = false;
			out.push_back(*segIter);
		}
                segIter++;
            }
        }
    }
    if (dbg) cout << "PRUNED " << out.size() << endl;    
}
