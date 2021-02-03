#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include "toffy/skeletonizers/depthSkeletonizer.hpp"

using namespace std;
using namespace cv;
//using namespace rapidjson;


bool N0[256];
bool N1[256];
bool N2[256];
bool N3[256];
bool N4[256];
bool N5[256];
bool N6[256];
bool N7[256];

//Weight for K3M
//128 01 02
// 64 XX 04
// 32 16 08

DepthSkeletonizer::DepthSkeletonizer() {
	thresh = 10;
    init();
}
DepthSkeletonizer::~DepthSkeletonizer()
{
	Borders.release();
}

void DepthSkeletonizer::init(){
    //Lookup Table for borders marking
    unsigned char cA0[] = {3,6,7,12,14,15,24,28,30,31,48,56,60,62,63,96,112,120,124,
                          126,127,129,131,135,143,159,191,192,193,195,199,207,223,224,
                          225,227,231,239,240,241,243,247,248,249,251,252,253,254};
    //Lookup tables for neighbours...:
    unsigned char n_3[] = { 7,14,   28,      56,      112,        131,         193,      224 };
    unsigned char n_4[] = {      15,   30,     60,      120,        135,        195,       225,      240 };
    unsigned char n_5[] = {               31,    62,      124,         143,        199,       227,      241,   248 };
    unsigned char n_6[] = {                        63,       126,        159,        207,       231,      243,   249,    252 };
    unsigned char n_7[] = {                                      127,          191,      223,             239,     247,        251,     253, 254 };

    //Lookup Tables for Phases 2-6
    unsigned char cA1[] = {7,14,28,56,112,131,193,224};
    unsigned char cA2[] = {7,14,15,28,30,56,60,112,120,131,135,193,195,224,225,240};

    unsigned char cA3[] = {7,14,15,28,30,31,56,60,62,112,120,124,131,135,143,193,195,
                          199,224,225,227,240,241,248};
    unsigned char cA4[] = {7,14,15,28,30,31,56,60,62,63,112,120,124,126,131,135,143,159
                          ,193,195,199,207,224,225,227,231,240,241,243,248,249,252};
    unsigned char cA5[] = {7,14,15,28,30,31,56,60,62,63,112,120,124,126,131,135,143,159
                          ,191,193,195,199,207,224,225,227,231,239,240,241,243,248,249
			   ,251,252,253,254};
    unsigned int i =0;

    memset(A0, 0, sizeof(A0));
    memset(A1, 0, sizeof(A1));
    memset(A2, 0, sizeof(A2));
    memset(A3, 0, sizeof(A3));
    memset(A4, 0, sizeof(A4));
    memset(A5, 0, sizeof(A5));

    memset(N0, 0, sizeof(N0));
    memset(N1, 0, sizeof(N1));
    memset(N2, 0, sizeof(N2));
    memset(N3, 0, sizeof(N3));
    memset(N4, 0, sizeof(N4));
    memset(N5, 0, sizeof(N5));
    memset(N6, 0, sizeof(N6));
    memset(N7, 0, sizeof(N7));


    cout << "A0 ";
    while(i < sizeof(cA0)) {
        A0[cA0[i]] = true;
	cout << hex << (int)cA0[i] << " ";
        i++;
    }
    cout << endl;
    for (i=0;i<sizeof(n_3);i++) {
        A1[n_3[i]] = true;
        A2[n_3[i]] = true;
        A3[n_3[i]] = true;
        A4[n_3[i]] = true;
        A5[n_3[i]] = true;
	
        N3[n_3[i]] = true;

    };

    for (i=0;i<sizeof(n_4);i++) {
        A2[n_4[i]] = true;
        A3[n_4[i]] = true;
        A4[n_4[i]] = true;
        A5[n_4[i]] = true;

        N4[n_4[i]] = true;
    };

    for (i=0;i<sizeof(n_5);i++) {
        A3[n_5[i]] = true;
        A4[n_5[i]] = true;
        A5[n_5[i]] = true;

        N5[n_5[i]] = true;
    };

    for (i=0;i<sizeof(n_6);i++) {
        A4[n_6[i]] = true;
        A5[n_6[i]] = true;

        N6[n_6[i]] = true;
    };

    for (i=0;i<sizeof(n_7);i++) {
        A5[n_7[i]] = true;

        N7[n_7[i]] = true;
    };

    return;


    cout << "A0 ";
    while(i < sizeof(cA0)) {
        A0[cA0[i]] = true;
	cout << hex << (int)cA0[i] << " ";
        i++;
    }
    cout << endl;
    i =0;
    while(i < sizeof(cA1)) {
        A1[cA1[i]] = true;
        i++;
    }

    i =0;
    while(i < sizeof(cA2)) {
        A2[cA2[i]] = true;
        i++;
    }

    i =0;
    while(cA3[i] != 248) {
        A3[cA3[i]] = true;
        i++;
    }

    i =0;
    while(i < sizeof(cA4)) {
        A4[cA4[i]] = true;
        i++;
    }

    i =0;
    while(i < sizeof(cA5)) {
        A5[cA5[i]] = true;
        i++;
    }
    Debugging = true;
}


void DepthSkeletonizer::skeletonize(const cv::Mat& lines, cv::Mat& skel)
{
    //Copy the src mat to the dest one and make the border Mat;
    lines.copyTo(Borders);
    lines.copyTo(skel);

    Mat removeMe = lines.clone(); 
    Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool modified;
    string sc="0_0_";
    int i=0;
    Mat mm;
    bool d = true;

    do{
        int fac=1;
        modified = false;
        //PHASE 0 MARK BORDER
        markBorder(skel);
	//if (d) {  resize(Borders,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='B'; imshow(sc,mm>0);imwrite(sc+".png", mm>0); }
	//if (d) {  resize(Borders,mm,Size(),fac,fac); sc[0]='A'; sc[2]='B'; imshow(sc,mm>0); }


	// K3M-original inline-modification approach that leads to artefacts

        //cout<<"phase 1" << endl;
        modified |= phase(skel,&A1[0]);
        //cout<<"phase 2" << endl;
        modified |= phase(skel,&A2[0]);
        //cout<<"phase 3" << endl;
        modified |= phase(skel,&A3[0]);
        //<<"phase 4" << endl;
        modified |= phase(skel,&A4[0]);
        //cout<<"phase 5" << endl;
        modified |= phase(skel,&A5[0]);
#if 0
	// Updated 2-phase approach has no artefacts but that leads to segmented graphs

	removeMe=0;
        cout<<"phase 1" << endl;
        modified |= phase(skel, removeMe, A1);
	skel -= removeMe;
	//imshow("REM 1", removeMe>0);
	//if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='1'; imshow(sc,mm);imwrite(sc+".png", mm>0); }

        cout<<"phase 2" << endl;
        modified |= phase(skel,removeMe, A2);
	skel &= ~removeMe;
	//imshow("REM 2", removeMe>0);
	//if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='2'; imshow(sc,mm);imwrite(sc+".png", mm>0); }

        cout<<"phase 3" << endl;
        modified |= phase(skel,removeMe,A3);
	skel &= ~removeMe;
	//imshow("REM 3", removeMe>0);
	//if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='3'; imshow(sc,mm);imwrite(sc+".png", mm>0); }
	
        cout<<"phase 4" << endl;
        modified |= phase(skel,removeMe,A4);
	skel &= ~removeMe;
	//imshow("REM 4", removeMe>0);
	//if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='4'; imshow(sc,mm);imwrite(sc+".png", mm>0); }

	cout<<"phase 5" << endl;
        modified |= phase(skel,removeMe,A5);
	skel &= ~removeMe;
	//imshow("REM 5", removeMe>0);
	//if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'+i; sc[2]='5'; imshow(sc,mm);imwrite(sc+".png", mm>0); }
	if (d) { resize(skel,mm,Size(),fac,fac); sc[0]='A'; sc[2]='5'; imshow(sc,mm); }

#endif

	i++;
	//waitKey();
    }while(modified);
    if(Debugging){
        cout << "DepthSkeletonizer : Prepare for Tracing"<< endl;
    }

    if(Debugging){
        cout << "DepthSkeletonizer : Skeletonization finished" << endl;
    }
}

/*void DepthSkeletonizer::configure(const rapidjson::Value &configObject)
{
    Debugging = configObject["Debug"].GetBool();
}*/

Skeletonizer *DepthSkeletonizer::Create()
{
    return new DepthSkeletonizer();
}

//static int thresh=40;
static inline bool chk(int thresh, unsigned char ref,unsigned char neigh) { 
	return neigh && abs(ref-neigh) < thresh ; 
}

void DepthSkeletonizer::markBorder(const cv::Mat &mat){
    //Go through the picture
    const unsigned char* cur = 0;
    const unsigned char* prev = 0;
    const unsigned char* next = 0;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get lines ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows )next = mat.ptr<unsigned char>(i+1);

        //Columns
        for(int j = 0; j < mat.cols;j++){
	    unsigned char c=cur[j];

            //is a object pix?
            if(!c){
                Borders.at<uchar>(i,j) = 0;
                continue;
            }

            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
		if((i-1) > 0 && chk(thresh, c, prev[j-1]) ) weight += 128;
                if(chk(thresh, c, cur[j-1])) weight += 64;
                if((i+1) < mat.rows && chk(thresh, c, next[j-1])) weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && chk(thresh, c, prev[j+1]) ) weight += 2;
                if( chk(thresh, c, cur[j+1])) weight += 4;
                if((i+1) < mat.rows && chk(thresh, c, next[j+1])) weight += 8;
            }
            //current column
            if((i-1) > 0 && chk(thresh, c, prev[j])) weight++;
            if((i+1) < mat.rows && chk(thresh, c, next[j])) weight += 16;
            //If weight in A0 then it's a border

            if(A0[weight]){
                Borders.at<uchar>(i,j) = 1;
            }
            else{
                Borders.at<uchar>(i,j) = 0;
            }
        }
    }
}

bool DepthSkeletonizer::phase(cv::Mat mat, bool *lookup){
    //Go through the picture
    const unsigned char* cur = 0;
    const unsigned char* prev = 0;
    const unsigned char* next = 0;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows )next = mat.ptr<unsigned char>(i+1);
        //Columns
        for(int j = 0; j < mat.cols;j++){
		unsigned char c = cur[j];
            //is a border pix?
            if(!Borders.at<uchar>(i,j)){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
		if((i-1) > 0 && chk(thresh, c, prev[j-1]) ) weight += 128;
                if(chk(thresh, c, cur[j-1])) weight += 64;
                if((i+1) < mat.rows && chk(thresh, c, next[j-1])) weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && chk(thresh, c, prev[j+1]) ) weight += 2;
                if( chk(thresh, c, cur[j+1])) weight += 4;
                if((i+1) < mat.rows && chk(thresh, c, next[j+1])) weight += 8;
            }
            //current column
            if((i-1) > 0 && chk(thresh, c, prev[j])) weight++;
            if((i+1) < mat.rows && chk(thresh, c, next[j])) weight += 16;
            //If weight in A0 then it's a border


	    /*
            if((j-1) > 0){
                if((i-1) > 0 && prev[j-1]) weight += 128;
                if(cur[j-1]) weight += 64;
                if((i+1) < mat.rows && next[j-1])weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && prev[j+1]) weight += 2;
                if(cur[j+1]) weight += 4;
                if((i+1) < mat.rows && next[j+1]) weight += 8;
            }

            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight += 16;


	    if (j==25 && i==29) {
		    cout << "neben Eck x=" << dec << j << " y=" << dec << i << " w=" << weight << "  " << lookup[weight] << endl;
	    }
	    */

            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                mat.at<uchar>(i,j) = 0;
            }
        }
    }
    return toReturn;
}


bool DepthSkeletonizer::phase(cv::Mat mat, cv::Mat& removeMe, bool *lookup){
    //Go through the picture
    const unsigned char* cur = 0;
    const unsigned char* prev = 0;
    const unsigned char* next = 0;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows )next = mat.ptr<unsigned char>(i+1);
        //Columns
        for(int j = 0; j < mat.cols;j++){
		unsigned char c = cur[j];
            //is a border pix?
            if(!Borders.at<uchar>(i,j)){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
		if((i-1) > 0 && chk(thresh, c, prev[j-1]) ) weight += 128;
                if(chk(thresh, c, cur[j-1])) weight += 64;
                if((i+1) < mat.rows && chk(thresh, c, next[j-1])) weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && chk(thresh, c, prev[j+1]) ) weight += 2;
                if( chk(thresh, c, cur[j+1])) weight += 4;
                if((i+1) < mat.rows && chk(thresh, c, next[j+1])) weight += 8;
            }
            //current column
            if((i-1) > 0 && chk(thresh, c, prev[j])) weight++;
            if((i+1) < mat.rows && chk(thresh, c, next[j])) weight += 16;
            //If weight in A0 then it's a border

	    if (j==25 && i==29) {
		    cout << "neben Eck x=" << dec << j << " y=" << dec << i << " w=" << weight << "  " << lookup[weight] << endl;
	    }


            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                removeMe.at<uchar>(i,j) = 255;
            } else {
                removeMe.at<uchar>(i,j) = 0;
	    }
        }
    }
    return toReturn;
}
