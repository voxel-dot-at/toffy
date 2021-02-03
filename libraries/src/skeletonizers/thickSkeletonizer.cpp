#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include "toffy/skeletonizers/thickSkeletonizer.hpp"

using namespace std;
using namespace cv;
//using namespace rapidjson;

ThickSkeletonizer::ThickSkeletonizer() :maxT(255) {
	initActions();
    Debugging = true;
}

ThickSkeletonizer::~ThickSkeletonizer() {
}


void ThickSkeletonizer::skeletonize(const cv::Mat& lines, cv::Mat& skel)
{

	/****************************/
	/* compute region thicknesses
	 ****************************/
    if(Debugging){
        cout << "ThickSkeletonizer : Computing thickness" << endl;
    }
    lines.copyTo(Input);
    computeThickness(lines, thickness);

    if(Debugging){
        cout << "ThickSkeletonizer : skeletonizing" << endl;
    }
    skeleton_1st_pass = lines.clone();
    // switched to erosion.
    Mat max_merged;

    //ErodeSkeletonizer er;
    //er.skeletonize(lines, max_merged);
max_merged = lines;
    skeleton_last_pass = max_merged.clone();

    string t="____s";
    Debugging=true;
    max = max_merged.clone();
    // final iteration?
    int c=0,n;
    do {
    	n = skel_once(max_merged,skel);
        if(Debugging){
            cout << "\t => iter " << c << ": removed " << n << endl;
	    t[0]='0'+c;
	    imshow(t,skel>0);
        }
    	c++;
    	max_merged = skel.clone();
    } while (n);
    if(Debugging){
        cout << "ThickSkeletonizer : merging" << endl;
    }
    mergeMax(thickness, skel);

    skeleton_merged = skel.clone();
    if(Debugging){
        cout << "ThickSkeletonizer : Skeletonizing Finished" << endl;
    }

}

void ThickSkeletonizer::skel(const Mat& in, Mat& skel) {
	// morphol. thinning, based on
	// http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
	Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	Mat lines;

	//dilate(in, lines, MORPH_RECT);

	skel.create(in.rows, in.cols, CV_8UC1);
	cv::Mat temp(in.size(), CV_8UC1);
	skel.setTo(Scalar(0));

	bool done;
	do {
		cv::morphologyEx(lines, temp, cv::MORPH_OPEN, element);
		cv::bitwise_not(temp, temp);
		cv::bitwise_and(lines, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		cv::erode(lines, lines, element);

		double max;
		cv::minMaxLoc(lines, 0, &max);
		done = (max == 0);
	} while (!done);
}


void ThickSkeletonizer::computeThickness(const Mat& in, Mat& thick) {
	// morphol. thinning, based on
	// http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
	Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	Mat lines = in.clone();
	lines.setTo(Scalar(1),in);

	thick.create(lines.rows, lines.cols, CV_8UC1);
	cv::Mat temp(lines.size(), CV_8UC1);
	thick.setTo(Scalar(0));

	bool done;
	do {
		thick += lines;
		cv::erode(lines, lines, element);

		double max;
		cv::minMaxLoc(lines, 0, &max);
		done = (max == 0);
	} while (!done);
}

Skeletonizer *ThickSkeletonizer::Create()
{
    return new ThickSkeletonizer();
}


void ThickSkeletonizer::findLocalMaxima(const Mat& in, Mat& max) {

	//Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	max.create(in.rows, in.cols, CV_8UC1);
	max.setTo(Scalar(0));

	// TODO: surrounding pixels.
	for (int y=1;y<in.rows-1;y++) {
		const unsigned char *last = in.ptr(y-1);
		const unsigned char *cur = in.ptr(y);
		const unsigned char *next = in.ptr(y+1);
		unsigned char *out = max.ptr(y);

		for (int x=1;x<in.cols-1;x++) {
			char max = cur[x];
			// get max:

			// check normal cross first
/*
			if (last[x]>=max) max=last[x];
			if (next[x]>=max) max=next[x];
			if (cur[x+1]>=max) max=cur[x+1];
			if (cur[x-1]>=max) max=cur[x-1];

			if (max && max<=cur[x]) {
				out[x] = 1;
				continue;
			}
			// diagonal cross:
			max = cur[x];
			if (last[x-1]>=max) max=last[x-1];
			if (last[x+1]>=max) max=last[x+1];
			if (next[x-1]>=max) max=next[x-1];
			if (next[x+1]>=max) max=next[x+1];
			if (max && max<=cur[x]) {
				out[x] = 1;
			}
*/
			// 8-neighbours:

			if (last[x-1]>=max) max=last[x-1];
			if (last[x]>=max) max=last[x];
			if (last[x+1]>=max) max=last[x+1];

			if (next[x-1]>=max) max=next[x-1];
			if (next[x]>=max) max=next[x];
			if (next[x+1]>=max) max=next[x+1];

			if (cur[x+1]>=max) max=cur[x+1];
			if (cur[x-1]>=max) max=cur[x-1];

			if (max && max<=cur[x]) {
				out[x] = max;
			}
		}
	}
}

static std::string prt(unsigned char code);

/** merge neighbouring maxima that are still in a region (thickness>0)
 * this bridges saddle points.
 *
 * TODO: this algorithm looks at the whole picture again. It should be
 * sufficient to look at the neighbours of (already known - cf. points vector)
 * endpoints.
 *
 * The logic is as follows:
 * IF a pixel p is not set
 * 	AND thickness[p] >0
 * 	THEN we have a candidate --> for this pixel chose action:
 * 	look at the neighbouring pixels; if we can connect 2 or more unconnected
 * 	segments, insert a pixel, otherwise leave it blank
 */
void ThickSkeletonizer::mergeMax(const Mat& thick, Mat& in) {
	Mat temp = in.clone();
//	if (in.empty()) {
//		temp.create(thick.rows,thick.cols,CV_8UC1);
//	} else {
//		temp= in.clone();
//	}
	Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	int unknown=0;
	for (int y=1;y<in.rows-1;y++) {
		const unsigned char *last = in.ptr(y-1);
		const unsigned char *cur = in.ptr(y);
		const unsigned char *next = in.ptr(y+1);
		unsigned char *out = temp.ptr(y);

		for (int x=1;x<in.cols-1;x++) {

			if (cur[x])
				continue; // only unset pixels

			char t = thick.at<char>(y,x); // and only if we're in the region
			if (! t )
				continue;

			char x1,x2,x3,x4,x5,x6,x7,x8;
			// numbering as in Lam, Lee 1992: x1 is east, counter-clockwise
			x1 = cur [x+1] >0;
			x2 = last[x+1] >0;
			x3 = last[x  ] >0;
			x4 = last[x-1] >0;
			x5 = cur [x-1] >0;
			x6 = next[x-1] >0;
			x7 = next[x  ] >0;
			x8 = next[x+1] >0;

			int idx = 	(x1<<0) | (x2<<1)| (x3<<2)| (x4<<3) |
						(x5<<4) | (x6<<5)| (x7<<6)| (x8<<7);
			int c = x1+x2+x3+x4+x5+x6+x7+x8;

			struct action& a = actions[idx];
			if (a.isKnown){

				if (a.setPixel) {
					out[x] = 1; // set to thickness.
				}
			} else {
				unknown++;
			}
		}
	}
	in = temp;
}

void ThickSkeletonizer::setAction(action actions[], unsigned char code, bool set) {
	actions[code].code = code;
	actions[code].isKnown = true;
	actions[code].setPixel = set;
}

/** rotate 45 deg against clock
 */
static unsigned char rot45(unsigned char code) {
	unsigned char c = (code & 0x80)>0;
	code <<=1;
	code |= c;
	return code;
}


void ThickSkeletonizer::setDiagRotatedActions(action actions[], unsigned char code, bool set) {
	int i;
	for (i=0;i<8;i++) {
		setAction(actions,code, set);
		code = rot45(code);
	}
}
void ThickSkeletonizer::set90DegRotatedActions(action actions[], unsigned char code, bool set) {
	int i;
	for (i=0;i<4;i++) {
		setAction(actions,code, set);
		code = rot45(code);
		code = rot45(code);
	}
}

#define X1 (1<<0)
#define X2 (1<<1)
#define X3 (1<<2)
#define X4 (1<<3)
#define X5 (1<<4)
#define X6 (1<<5)
#define X7 (1<<6)
#define X8 (1<<7)

static std::string prt(unsigned char code) {
	std::string c="";
	if (code&X1) c += "|x1";
	if (code&X2) c += "|x2";
	if (code&X3) c += "|x3";
	if (code&X4) c += "|x4";
	if (code&X5) c += "|x5";
	if (code&X6) c += "|x6";
	if (code&X7) c += "|x7";
	if (code&X8) c += "|x8";
	return c;
}

void ThickSkeletonizer::initActions() {
	/****************************************************/
	/* merge patterns									*/
	/****************************************************/

    //cout << "ACTIONS " << sizeof(actions) << endl;
	memset(actions,0,sizeof(actions));

	///// 0 neighbours
	setAction(actions,0x00,false);
	///// 1 neighbours
	setDiagRotatedActions(actions, X1, false);

	///// 2 neighbours
	setDiagRotatedActions(actions, X1 | X2, false);
	setDiagRotatedActions(actions, X1 | X3, false);

	setDiagRotatedActions(actions, X1 | X5, true);

	setDiagRotatedActions(actions, X1 | X4, true);
	setDiagRotatedActions(actions, X1 | X6, true); // vflip

	///// 3 neighbours
	setDiagRotatedActions(actions,X1 | X2 | X3, false);

	setDiagRotatedActions(actions,X2 | X3 | X5, false);
	setDiagRotatedActions(actions,X4 | X3 | X1, false);

	setDiagRotatedActions(actions,X1 | X2 | X5, true);
	setDiagRotatedActions(actions,X1 | X4 | X5, true);

	setDiagRotatedActions(actions,X1 | X3 | X5, false);
	setDiagRotatedActions(actions,X1 | X7 | X5, false);

	//0 1 0
	//1 X 0
	//0 0 1
	setDiagRotatedActions(actions,X1 | X3 | X6, false);
	setDiagRotatedActions(actions,X3 | X5 | X8, false);

	///// 4 neighbours
	setDiagRotatedActions(actions,0x0f, false);

	setDiagRotatedActions(actions,X1 | X3 | X5 | X7, false);
	setDiagRotatedActions(actions,X1 | X3 | X6 | X7, false);
	setDiagRotatedActions(actions,X8 | X3 | X5 | X7, false);

	setDiagRotatedActions(actions,X1 | X3 | X4 | X6, false);
	setDiagRotatedActions(actions,X1 | X4 | X6 | X7, false);


	setDiagRotatedActions(actions,X1|X2|X3 | X5, false);
	setDiagRotatedActions(actions,X1|X2|X3 | X7, false);

	setDiagRotatedActions(actions,X1|X2|X3 | X6, false);
	setDiagRotatedActions(actions,X1|X2 | X4|X5, false);


	setDiagRotatedActions(actions,X1|X2 |X5 | X6, true);
	setDiagRotatedActions(actions,X1|X8 |X5 | X4, true);

	///// 5 neighbours
	setDiagRotatedActions(actions,0x1f, false);

	setDiagRotatedActions(actions, X2 | X3 | X4 | X6 | X8, false);
	setDiagRotatedActions(actions, X2 | X3 | X4 | X7 | X8, false);
	setDiagRotatedActions(actions, X2 | X3 | X4 | X6 | X7, false);

	setDiagRotatedActions(actions, X2 | X3 | X5 | X7 | X8, false);

	setDiagRotatedActions(actions, X2 | X5 | X6 | X7 | X8, false);

	setDiagRotatedActions(actions, X2 | X3 | X6 | X7 | X8, false);

	///// 6 neighbours
	setDiagRotatedActions(actions,0x3f, false);

	setDiagRotatedActions(actions, X2|X3|X4 |  X6|X7|X8 , false);

	setDiagRotatedActions(actions, X2|X3 |X5|X6|X7|X8 , false);
	setDiagRotatedActions(actions, X3|X4    |X6|X7|X8|X1 , false);

	///// 7 neighbours
	setDiagRotatedActions(actions,0x7f, false);

	///// 8 neighbours
	setAction(actions, 0xff,false);

	/****************************************************/
	/*		Skeletonizer								*/
	/****************************************************/
	/**
	 * if the current px is set, look if we can remove it.
	 */
	memset(skels,0,sizeof(skels));

	setAction(skels,0, true);// 1 single point: keep

	///// 1 neighbour
	setDiagRotatedActions(skels,X1, true);// only 1 neigh: keep

	///// 2 neighbour
	set90DegRotatedActions(skels,X1|X2, true); // J shape - end
	set90DegRotatedActions(skels,X1|X8, true); // L shape - end

	set90DegRotatedActions(skels,X1|X3, false); // L shape - bend
	set90DegRotatedActions(skels,X3|X5, false); // L shape - bend

	setDiagRotatedActions(skels,X1|X5, true); // line; keep

	set90DegRotatedActions(skels,X1|X4, true); // line bend; keep
	set90DegRotatedActions(skels,X1|X6, true); // line bend; keep

	set90DegRotatedActions(skels,X2|X4, true); // line bend; keep

	///// 3 neighbour
	set90DegRotatedActions(skels,X1|X2|X3, false); // edge of block; kill

	set90DegRotatedActions(skels,X2|X3|X4, false); // line ridge

	set90DegRotatedActions(skels,X1|X3|X4, false); // line bend --_
	set90DegRotatedActions(skels,X1|X6|X7, false); // line bend __-

	set90DegRotatedActions(skels,X1|X3|X5, false); // T connector
	set90DegRotatedActions(skels,X1|X3|X7, false); // T connector

	set90DegRotatedActions(skels,X1|X3|X6, true); // would break conn
	set90DegRotatedActions(skels,X1|X7|X4, true); // would break conn

	set90DegRotatedActions(skels,X1|X2 | X4, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X8 | X4, true); // would break connectivity

	set90DegRotatedActions(skels,X1|X8 | X5, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X2 | X5, true); // would break connectivity

	set90DegRotatedActions(skels,X1|X2 | X6, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X8 | X6, true); // would break connectivity

	set90DegRotatedActions(skels,X1|X2 | X7, false); // would break connectivity
	set90DegRotatedActions(skels,X4|X5 | X7, false); // would break connectivity

	set90DegRotatedActions(skels,X2|X8 | X4, true); // would break connectivity
	set90DegRotatedActions(skels,X2|X8 | X5, true); // would break connectivity
	set90DegRotatedActions(skels,X2|X8 | X6, true); // would break connectivity

	///// 4 neighbour
	setAction(skels, X1|X3|X5|X7, false); // diamond
	setAction(skels, X2|X4|X6|X8, true); // X

	set90DegRotatedActions(skels,X1|X2|X3|X4, false); // block edge; shrink
	set90DegRotatedActions(skels,X1|X6|X7|X8, false); // block edge; shrink

	set90DegRotatedActions(skels,X1|X2|X3 |X5, false); // edge of block; shrink
	set90DegRotatedActions(skels,X1|X2|X3 |X7, false); // edge of block; shrink

	set90DegRotatedActions(skels,X1|X2|X3 |X6, true); // edge of block; shrink

	set90DegRotatedActions(skels,X1|X2 |X6 |X7, false); // inner edge of big J; shrink

	set90DegRotatedActions(skels,X1|X2 | X4|X5, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X8 | X5|X6, true); // would break connectivity

	set90DegRotatedActions(skels,X1|X2 | X4 | X6, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X8 | X4 | X6, true); // would break connectivity

	set90DegRotatedActions(skels,X1|X2 | X4 | X7, true); // would break connectivity


	set90DegRotatedActions(skels,X1|X2 | X5|X6, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X8 | X5|X4, true); // would break connectivity


	set90DegRotatedActions(skels,X1|X2|X8 | X4, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X2|X8 | X5, true); // would break connectivity
	set90DegRotatedActions(skels,X1|X2|X8 | X6, true); // would break connectivity


	set90DegRotatedActions(skels,X1 | X3|X4 | X7, true); // would break connectivity
	set90DegRotatedActions(skels,X2|X3 | X5 | X7, true); // would break connectivity

	set90DegRotatedActions(skels,X1 | X3|X4 | X6, true); // would break connectivity
	set90DegRotatedActions(skels,X2|X3 | X5 | X8, true); // would break connectivity

	set90DegRotatedActions(skels,X1 | X3|X4 | X6, true); // would break connectivity

	///// 5 neighbour
	set90DegRotatedActions(skels,X1|X2|X3|X4|X5, false); // in block

	set90DegRotatedActions(skels,X1|X2|X3 |X5|X6, false); //
	set90DegRotatedActions(skels,X1| X3|X4|X5|X6, false); //


	set90DegRotatedActions(skels,X1|X2|X3 |X5|X7, false); //

	set90DegRotatedActions(skels,X1|X2|X4 |X5|X7, false); //

	set90DegRotatedActions(skels,X1|X2|X3 |X6|X7, false); //

	set90DegRotatedActions(skels,X1|X2 |X4 |X6|X7, true); // would break conn.

	set90DegRotatedActions(skels,X1|X2|X8 |X4 |X6, true); // would break conn.

	set90DegRotatedActions(skels,X1|X2|X3|X8 | X5, true); //

	set90DegRotatedActions(skels,X1|X2|X3|X8 | X6, true); //
	set90DegRotatedActions(skels,X1|X2|X3|X4 | X6, true); //

	set90DegRotatedActions(skels,X1|X2 |X4 |X5|X6, false); //

	set90DegRotatedActions(skels,X1|X2|X8 |X4|X5, true); //
	set90DegRotatedActions(skels,X1|X2|X8 |X5|X6, true); //


	set90DegRotatedActions(skels,X8|X1|X2|X3|X4, false); //  BIG L, inner point
	set90DegRotatedActions(skels,X2|X3|X4|X5|X6, false); //  BIG L, inner point

	///// 6 neighbour
	set90DegRotatedActions(skels,X1|X2|X3|X4|X5|X6, false); //  BIG L, inner point

	set90DegRotatedActions(skels,X1|X2|X3|X4|X5 | X7, false); //

	set90DegRotatedActions(skels,X1|X2|X3|X4 |X6|X7, false); //

	set90DegRotatedActions(skels,X1|X2|X3|X4|X8 | X6, true); //

	set90DegRotatedActions(skels,X1|X2|X3 | X5|X6|X7, false); //

	set90DegRotatedActions(skels,X8|X1|X2|X3 |X4|X5, false); //  BIG L, inner point

	set90DegRotatedActions(skels,X8|X1|X2|X3 |X5|X6, false); //  BIG L, inner point

	set90DegRotatedActions(skels,X1|X2|X8 | X4|X5|X6, true); // 2lines parallel connected

	///// 7 neighbour
	set90DegRotatedActions(skels,X1|X2|X3|X4|X5|X6|X7, false); //  BIG L, inner point

	///// 8 neighbour
	setDiagRotatedActions(skels,0xff, true);// inner pixel; next iteration

}

int ThickSkeletonizer::skel_once(const Mat& in, Mat& skel) {
    int x,y;
    skel = in.clone();
    int removed = 0;
    //skel.setTo(Scalar(0));

    for (y=1;y<in.rows-1;y++){
        const uchar* prv = skel.ptr(y-1);
        const uchar* cur = skel.ptr(y);
        const uchar* nxt = skel.ptr(y+1);

        uchar* out = skel.ptr(y);

        for (x=1;x<in.cols-1;x++) {
        	char px = cur[x];

        	if (0 == px)
        		continue;

			char x1,x2,x3,x4,x5,x6,x7,x8;
			// numbering as in Lam, Lee 1992: x1 is east, counter-clockwise
			x1 = cur[x+1] >0;
			x2 = prv[x+1] >0;
			x3 = prv[x  ] >0;
			x4 = prv[x-1] >0;
			x5 = cur[x-1] >0;
			x6 = nxt[x-1] >0;
			x7 = nxt[x  ] >0;
			x8 = nxt[x+1] >0;

			int idx = 	(x1<<0) | (x2<<1)| (x3<<2)| (x4<<3) |
						(x5<<4) | (x6<<5)| (x7<<6)| (x8<<7);
//			int c = x1+x2+x3+x4+x5+x6+x7+x8;

			struct action& a = skels[idx];
			if (a.isKnown){

				if (a.setPixel) {
					out[x] = 1;
				} else {

					out[x] = 0;
					removed++;
				}
			} else {
			}
        }
    }
    return removed;
}

/*
void ThickSkeletonizer::configure(const rapidjson::Value &configObject)
{
    Debugging = configObject["Debug"].GetBool();
}
*/
