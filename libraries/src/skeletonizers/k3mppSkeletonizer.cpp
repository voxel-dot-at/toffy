#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include "toffy/skeletonizers/k3mppSkeletonizer.hpp"

const bool trc=false;

using namespace std;
using namespace cv;
//using namespace rapidjson;

//Weight for K3M
//128 01 02
// 64 XX 04
// 32 16 08

K3MPPSkeletonizer::K3MPPSkeletonizer() {
    memset(A0, 0, sizeof(A0));
    memset(A1, 0, sizeof(A1));
    memset(A2, 0, sizeof(A2));
    memset(A3, 0, sizeof(A3));
    memset(A4, 0, sizeof(A4));
    memset(A5, 0, sizeof(A5));
    memset(A6, 0, sizeof(A6));

    init();
}
K3MPPSkeletonizer::~K3MPPSkeletonizer()
{
	Borders.release();
}

void K3MPPSkeletonizer::init(){
    //Lookup Table for borders marking
    unsigned char cA0[] = {3,6,7,12,14,15,24,28,30,31,48,56,60,62,63,96,112,120,124,
			   126,127,129,131,135,143,159,191,192,193,195,199,207,223,224,
                          225,227,231,239,240,241,243,247,248,249,251,252,253,254};
    //Lookup Tables for Phases 2-6
    unsigned char cA1[] = {7,14,28,56,112,131,193,224};
    unsigned char cA2[] = {7,14,15,28,30,56,60,112,120,131,135,193,195,224,225,240};
    unsigned char cA3[] = {7,14,15,28,30,31,56,60,62,112,120,124,131,135,143,193,195,
                          199,224,225,227,240,241,248};
    unsigned char cA4[] = {7,14,15,28,30,31,56,60,62,63,112,120,124,126,131,135,143,159
                          ,193,195,199,207,224,225,227,231,240,241,243,248,249,252};
    unsigned char cA5[] = {7,14,15,28,30,31,56,60,62,63,112,120,124,126,131,135,143,159
                          ,191,193,195,199,207,224,225,227,231,239,240,241,243,248,249
                          ,251,252,254};
    unsigned char cA1px[] = {3, 6, 7, 12, 14, 15, 24, 28, 30, 31, 48, 56,
			     60, 62, 63, 96, 112, 120, 124, 126, 127, 129, 131,
			     135, 143, 159, 191, 192, 193, 195, 199, 207, 223,
			     224, 225, 227, 231, 239, 240, 241, 243, 247, 248,
			     249, 251, 252, 253, 254,
			     // corner points:
			     65, 80, 20, 5,
			     // stair cases
			     1|4|8, 
			     2|4|16,
			     4|16|32,
			     1|32|64,
			     1|4|128,
			     64|16|8, 
			     128|64|16,
			     128|64|16|8,
			     1|2|32|64,
			     2|4|16|32,
			     // t crossings
			     64|16|4,
			     1 | 4 | 64,
			     1 | 4  | 16 | 32,
			     1 | 16 | 64,
			     1 | 16 | 8 | 64,
			     1 | 16 | 64 | 128,
			     1 | 4  |  8 | 128,
    };
    size_t i =0;

    for (i=0;i<sizeof(cA0);i++) {
        A0[cA0[i]] = true;
    }

    for (i=0;i<sizeof(cA1);i++) {
        A1[cA1[i]] = true;
    }

    for (i=0;i<sizeof(cA2);i++) {
        A2[cA2[i]] = true;
    }

    for (i=0;i<sizeof(cA3);i++) {
        A3[cA3[i]] = true;
    }

    for (i=0;i<sizeof(cA4);i++) {
        A4[cA4[i]] = true;
    }

    for (i=0;i<sizeof(cA5);i++) {
        A5[cA5[i]] = true;
    }

    for (i=0;i<sizeof(cA1px);i++) {
        A6[cA1px[i]] = true;
    }

    Debugging = false;
}


static void readyToTrace(cv::Mat &mat)
{
    //Go through the picture
    const unsigned char* cur;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        cur = mat.ptr<unsigned char>(i);
        //Columns
        for(int j = 0; j < mat.cols;j++){
            //is a object pix?
            if(!cur[j]){
                continue;
            }
            mat.at<uchar>(i,j) = 1;
        }
    }
}

void K3MPPSkeletonizer::skeletonize(const cv::Mat& lines, cv::Mat& skel){

    //imshow("k3m PRE LINES", lines );
    //imshow("k3m PRE LINES>>", lines>0 );

    //Copy the src mat to the dest one and make the border Mat;
    lines.copyTo(Borders);
    lines.copyTo(Input);
    Borders.copyTo(skel);
    if(Debugging){
        cout << "K3MPPSkeletonizer : Computing thickness"<< endl;
    }
    computeThickness(Input);
    if(Debugging){
        cout << "K3MPPSkeletonizer : Skeletonizing" <<endl;
    }
    Mat m;
    //imshow("k3m PRE SKEL", skel>0 );

    bool modified;
    int iter=0;
    do{
        modified = false;
        //PHASE 0 MARK BORDER
        markborder(skel);
	if (trc) { resize( Borders != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("border",m );  }
        //cout<<"phase 1" << endl;

        modified |= phase(skel, A1);
	if (trc) { resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase 1",m ); }

        //cout<<"phase 2" << endl;
        modified |= phase(skel ,A2);
	if (trc) { resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase 2",m ); }

        //cout<<"phase 3" << endl;
        modified |= phase(skel ,A3);
	if (trc) { resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase 3",m ); }

        //<<"phase 4" << endl;
        modified |= phase(skel, A4);
	if (trc) { resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase 4",m ); }

        //cout<<"phase 5" << endl;
        modified |= phase(skel, A5);
	if (trc) { resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase 5",m ); }

	//string n("0 k3m_skel");
	//n[0] = 'A'+iter;
	//imshow(n, skel > 0);
	//iter++;

    }while(modified);

    markborder(skel);
#if 0
    Mat out;
    phaseSingle(skel,out, A6);
    out = skel;
#else
    phaseSingle(skel, A6);
#endif
    if (trc) {resize( skel != 0, m, Size(), 4,4, INTER_NEAREST);	imshow("phase X",m );  }
    if (trc) imshow("phase X SKEL",skel ); 

    if(Debugging){
        cout << "K3MPPSkeletonizer : Prepare for Tracing"<< endl;
    }
    readyToTrace(skel);
    
    if(Debugging){
        cout << "K3MPPSkeletonizer : Skeletonization finished" << endl;
    }
}
/*
void K3MPPSkeletonizer::configure(const rapidjson::Value &configObject)
{
    Debugging = configObject["Debug"].GetBool();
}
*/
Skeletonizer *K3MPPSkeletonizer::Create()
{
    return new K3MPPSkeletonizer();
}

void K3MPPSkeletonizer::computeThickness(const Mat& in) {
    distanceTransform(in, thickness, CV_DIST_L2, 3);
    thickness.convertTo(thickness,CV_8UC1);
    //imshow("thickness" , thickness*16);
}

void K3MPPSkeletonizer::markborder(const cv::Mat &mat){
    //Go through the picture
    const unsigned char* cur=0;
    const unsigned char* prev=0;
    const unsigned char* next=0;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get lines ptr
        if((i-1) >= 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows )next = mat.ptr<unsigned char>(i+1);
        //Columns
        for(int j = 0; j < mat.cols;j++){
            //is a object pix?
            if(!cur[j]){
                Borders.at<uchar>(i,j) = 0;
                continue;
            }
            //Calculate the weight
            int weight = 0;
	    
#if 1
            //Previous column
            if((j-1) >= 0){
                if((i-1) >= 0 && prev[j-1]) weight ++;
                if(cur[j-1]) weight ++;
                if((i+1) < mat.rows && next[j-1])weight ++;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) >= 0 && prev[j+1]) weight ++;
                if(cur[j+1]) weight ++;
                if((i+1) < mat.rows && next[j+1])weight ++;
            }
            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight ++;

            if( weight >=1 && weight <= 7){
                Borders.at<uchar>(i,j) = 1;
            }
            else{
                Borders.at<uchar>(i,j) = 0;
            }
#else
            //Previous column
            if((j-1) >= 0){
                if((i-1) >= 0 && prev[j-1]) weight += 128;
                if(cur[j-1]) weight += 64;
                if((i+1) < mat.rows && next[j-1])weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) >= 0 && prev[j+1]) weight += 2;
                if(cur[j+1]) weight += 4;
                if((i+1) < mat.rows && next[j+1])weight += 8;
            }
            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight += 16;
            //If weight in A0 then it's a border


	    if (weight < 255)
	    cout << " ... " << j << "x"<<i<<"\t"<< weight << "\t" << hex << weight << " " << " " <<A0[weight] << dec << endl ; 
            if(A0[weight]){
                Borders.at<uchar>(i,j) = 1;
            }
            else{
                Borders.at<uchar>(i,j) = 0;
            }
#endif
        }
    }
}

bool K3MPPSkeletonizer::phase(cv::Mat mat, bool* lookup)
{
    //Go through the picture
    const unsigned char* cur=0;
    const unsigned char* prev=0;
    const unsigned char* next=0;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows ) next = mat.ptr<unsigned char>(i+1);

        //Columns
        for(int j = 0; j < mat.cols;j++){
            //is a border pix?
            if(!Borders.at<uchar>(i,j)){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
                if((i-1) > 0 && prev[j-1]) weight += 128;
                if(cur[j-1]) weight += 64;
                if((i+1) < mat.rows && next[j-1])weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && prev[j+1]) weight += 2;
                if(cur[j+1]) weight += 4;
                if((i+1) < mat.rows && next[j+1])weight += 8;
            }
            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight += 16;
            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                mat.at<uchar>(i,j) = 0;
	    }
	}
    }
    return toReturn;
}

bool K3MPPSkeletonizer::phase(cv::Mat mat, cv::Mat& out, bool* lookup)
{
    mat.copyTo(out);

    //Go through the picture
    const unsigned char* cur;
    const unsigned char* prev;
    const unsigned char* next;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows ) next = mat.ptr<unsigned char>(i+1);

        //Columns
        for(int j = 0; j < mat.cols;j++){
            //is a border pix?
            if(!Borders.at<uchar>(i,j)){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
                if((i-1) > 0 && prev[j-1]) weight += 128;
                if(cur[j-1]) weight += 64;
                if((i+1) < mat.rows && next[j-1])weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && prev[j+1]) weight += 2;
                if(cur[j+1]) weight += 4;
                if((i+1) < mat.rows && next[j+1])weight += 8;
            }
            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight += 16;
            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                out.at<uchar>(i,j) = 0;
	    }
	}
    }
    return toReturn;
}


bool K3MPPSkeletonizer::phaseSingle(cv::Mat mat, cv::Mat& out, bool* lookup)
{
    mat.copyTo(out);
    out.setTo(0);

    //Go through the picture
    const unsigned char* cur;
    const unsigned char* prev;
    const unsigned char* next;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows ) next = mat.ptr<unsigned char>(i+1);

        //Columns
        for(int j = 0; j < mat.cols;j++){
            if(! cur[j] ){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0){
                if((i-1) > 0 && prev[j-1]) weight += 128;
                if(cur[j-1]) weight += 64;
                if((i+1) < mat.rows && next[j-1])weight += 32;
            }
            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && prev[j+1]) weight += 2;
                if(cur[j+1]) weight += 4;
                if((i+1) < mat.rows && next[j+1])weight += 8;
            }
            //current column
            if((i-1) > 0 && prev[j]) weight++;
            if((i+1) < mat.rows && next[j]) weight += 16;
            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                out.at<uchar>(i,j) = 0;
		cout << " " << i << "x" << j << endl;
	    } else {
                out.at<uchar>(i,j) = 1;
	    }
	}
    }

    return toReturn;
}

bool K3MPPSkeletonizer::phaseSingle(cv::Mat& mat, bool* lookup)
{
    //Go through the picture
    unsigned char* cur = 0;
    const unsigned char* prev = 0;
    const unsigned char* next = 0;
    bool toReturn = false;
    //Rows
    for(int i = 0; i < mat.rows;i++){
        //get line ptr
        if((i-1) > 0) prev = mat.ptr<unsigned char>(i-1);
        cur = mat.ptr<unsigned char>(i);
        if((i+1) < mat.rows ) next = mat.ptr<unsigned char>(i+1);

        //Columns
        for(int j = 0; j < mat.cols;j++){
            if(! cur[j] ){
                continue;
            }
            //Calculate the weight
            int weight = 0;
            //Previous column
            if((j-1) > 0) {
                if((i-1) > 0 && prev[j-1]) weight += 128;
                if(cur[j-1])               weight += 64;
                if((i+1) < mat.rows && next[j-1]) 
                                           weight += 32;
            }
            //current column
            if((i-1) > 0 && prev[j])        weight ++;
            if((i+1) < mat.rows && next[j]) weight += 16;

            //next Column
            if((j+1) < mat.cols){
                if((i-1) > 0 && prev[j+1])  weight += 2;
                if(cur[j+1])                weight += 4;
                if((i+1) < mat.rows && next[j+1])
                                            weight += 8;
            }
            //if weight is in lookup then it's to delete
            if(lookup[weight]){
                toReturn = true;
                //mat.at<uchar>(i,j) = 0;
		cur[j] = 0;
		if (trc) cout << " xx " << i << "x" << j << " " << hex << weight << dec << endl;
	    } else {
		if (trc) cout << " keep " << i << "x" << j << " " << hex << weight << dec << endl;
	    }
	}
    }
    return toReturn;
}
