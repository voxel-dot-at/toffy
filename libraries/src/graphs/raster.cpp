
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "toffy/graphs/raster.hpp"

using namespace cv;
using namespace std;

RasterPoint RasterSegment::unknown(-1, -1);

int RasterPoint::counter = 0;

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
int RasterSegment::RID = 0;

RasterSegment::RasterSegment(RasterPoint& start, RasterPoint& end)
    : id(RID++), start(&start), end(&end), len(0), myGraph(0), pruned(false) {

          // cout << "NEW SEG " << id << " " << start << " " << end << endl;
      };

RasterSegment::RasterSegment(const RasterSegment& o)
    : id(o.id), start(o.start), end(o.end), pts(o.pts), len(o.len)
{
    std::cout << "CPY SEG " << o.id << " " << (string) * this << std::endl;
    // for (int i=0;i<8;i++) next[i] = o.next[i];
};

RasterSegment::~RasterSegment()
{
    for (size_t i = 0; i < pts.size(); i++) delete pts[i];
    pts.clear();
}

void RasterSegment::points(std::vector<cv::Point*>& points) const
{
    points.push_back(start);
    for (size_t i = 0; i < pts.size(); i++) points.push_back(pts[i]);
    points.push_back(end);
}

void RasterSegment::points(std::vector<cv::Point>& points) const
{
    points.push_back(*start);
    for (size_t i = 0; i < pts.size(); i++) points.push_back(*pts[i]);
    points.push_back(*end);
}

void RasterSegment::points(std::vector<RasterPoint*>& points) const
{
    points.push_back(start);

    for (size_t i = 0; i < pts.size(); i++) {
        points.push_back(pts[i]);
    }
    points.push_back(end);
}

void RasterSegment::points(std::vector<RasterPoint*>& points,
                           bool unVisitedOnly, bool addInlinePoints)
{
    if (unVisitedOnly) {
        if (!start->visited) {
            points.push_back(start);
        }
        start->visited = true;
    } else
        points.push_back(start);

    if (addInlinePoints)
        for (size_t i = 0; i < pts.size(); i++) {
            if (unVisitedOnly) {
                if (!pts[i]->visited) points.push_back(pts[i]);
                pts[i]->visited = true;
            } else
                points.push_back(pts[i]);
        }

    if (unVisitedOnly) {
        if (!end->visited) points.push_back(end);
        end->visited = true;
    } else
        points.push_back(end);
}

RasterSegment::operator std::string() const
{
    std::stringstream sb;
    sb << "(" << id << ":";
    if (end)
        sb << *start;
    else
        sb << "(nil)";
    sb << "-" << len << "->";
    if (end)
        sb << *end;
    else
        sb << "(nil)";
    sb << ")";
    return sb.str();
}

std::ostream& operator<<(std::ostream& o, const RasterSegment& seg)
{
    o << (std::string)seg;
    return o;
}

void RasterSegment::extend()
{
    if (len == 0) {
        return;
    }
    RasterPoint* cor = 0;
    if (!pts.empty()) {
        for (size_t i = 0; i < pts.size(); i++) {
            if (pts[i]->type == cornerPoint) cor = pts[i];
        }
    }
    RasterPoint* beg = start;
    if (cor != 0) {
        // work on end -> cor
        int x, y;
        x = end->x;
        y = end->y;
        x -= cor->x;
        y -= cor->y;
        int size = x + y;
        if (size < 0) size = -size;
        int newSize = size + (len >> 3);
        x *= newSize;
        x /= size;
        y *= newSize;
        y /= size;
        end->x = x + cor->x;
        end->y = y + cor->y;
        // the same with bed->cor
        x = beg->x;
        y = beg->y;
        x -= cor->x;
        y -= cor->y;
        size = x + y;
        if (size < 0) size = -size;
        newSize = size + (len >> 3);
        x *= newSize;
        x /= size;
        y *= newSize;
        y /= size;
        beg->x = x + cor->x;
        beg->y = y + cor->y;
    } else {
        int x, y;
        int mx, my;
        mx = end->x + beg->x;
        mx >>= 1;
        my = end->y + beg->y;
        my >>= 1;
        // work on end
        x = end->x - mx;
        y = end->y - my;
        int size = x + y;
        if (size < 0) size = -size;
        int newSize = size + (len >> 2);
        x *= newSize;
        if (size != 0) x /= size;
        y *= newSize;
        if (size != 0) y /= size;
        end->x = x + mx;
        end->y = y + my;
        // work on beg
        x = beg->x - mx;
        y = beg->y - my;
        size = x + y;
        if (size < 0) size = -size;
        newSize = size + (len >> 2);
        x *= newSize;
        if (size != 0) x /= size;
        y *= newSize;
        if (size != 0) y /= size;
        beg->x = x + mx;
        beg->y = y + my;
    }
}

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/

std::ostream& operator<<(std::ostream& o, const RasterPoint& p)
{
    o << "[ " << p.id << ": " << p.x << ", " << p.y << ", " << p.type << ", "
      << p.thick << " ]";
    return o;
}

void paintSegment(Mat& disMax, const RasterSegment& seg, Scalar color,
                  int thickness)
{
    vector<Point*> pts;
    seg.points(pts);

    RasterPoint* p1;
    p1 = seg.start;

    for (size_t i = 0; i < pts.size(); i++) {
        line(disMax, *p1, *pts[i], color, thickness);

        if (((RasterPoint*)(p1))->type == crossingPoint)
            line(disMax, *p1, *p1, Scalar(255, 255, 255));
        else if (((RasterPoint*)(p1))->type == endPoint)
            line(disMax, *p1, *p1, Scalar(128, 255, 255));
        else
            line(disMax, *p1, *p1, Scalar(0, 0, 128));
        p1 = (RasterPoint*)pts[i];
    }
    line(disMax, *p1, *seg.end, color, thickness);

    if (seg.end->type == crossingPoint)
        line(disMax, *seg.end, *seg.end, Scalar(255, 255, 255));
    else if (seg.end->type == endPoint)
        line(disMax, *seg.end, *seg.end, Scalar(128, 255, 255));
    else
        line(disMax, *seg.end, *seg.end, Scalar(0, 0, 128));
}
