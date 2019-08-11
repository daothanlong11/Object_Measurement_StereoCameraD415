#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;


/// Global variables
Mat _src;

// Function get corner
vector<Point2f> getCorner(Mat src);

/** @function main */
int main( int argc, char** argv )
{
    /*
    VideoCapture cap;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY; 
    cap.open(deviceID+apiID);
    */

    VideoCapture cap;
    cap.open("objectVideo5.avi");
    
    for(;;)
    {
        cap >> _src;
        if( _src.empty() )
            break;
        
        vector<Point2f> hai = getCorner(_src);
        int sophantu = hai.size();
        if (sophantu == 4)
        {
            circle( _src, Point(hai[0]), 1, Scalar(255, 0, 0), -1);
            circle( _src, Point(hai[1]), 1, Scalar(0, 255, 0), -1);
            circle( _src, Point(hai[2]), 1, Scalar(0, 0, 255), -1);
            circle( _src, Point(hai[3]), 1, Scalar(0, 0, 0), -1);
            for (int i=0; i<4; i++)
            {
                //circle( _src, Point(hai[i]), 1, Scalar(0, 255, 0), -1);
            }
        }
        
        imshow("goc", _src);
        //cout<<"width: "<<_src.cols<<endl;
        //cout<<"height: "<<_src.rows<<endl;
        
        //waitKey(0);    
        char c = (char)waitKey(30);
        if( c == 'q' || c == 'Q' || c == 27 ) break;
            
    }
    return(0);
}


vector<Point2f> getCorner(Mat src)
{
    ////////////////////////////////////////////////////////// 
    #define left_width 0
    #define right_width 640
    #define up_height 0
    #define down_height 480 
    
    #define radius 10
    const int thresh = 3;
    const int block = 7;

    const int lowThreshold = 20;
    const int ratio = 3;
    const int kernel_size = 3;
    
    Mat kernel_thr = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat dstGray = Mat::zeros(Size(src.cols, src.rows), CV_8U);

    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////
    //---------------------------------------------------------
    //------------getNoiseFilter-----------------------------//
    Mat srcGray, adapThreshGray, CannyGray;
    Mat cropGray;
    ///////////////////////////////////////////////////////////
    cvtColor(src, srcGray, CV_BGR2GRAY);
    ///------adaptiveThreshold------------------------------//
    adaptiveThreshold(srcGray, adapThreshGray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, block, thresh);
    morphologyEx(adapThreshGray, adapThreshGray, MORPH_CLOSE, kernel_thr);
    //imshow("adapThresh", adapThreshGray);
    ///-------Canny detector--------------------------------//
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(4,4), Point(1,1));
    blur(srcGray, CannyGray, Size(3,3));
    Canny(CannyGray, CannyGray, lowThreshold, lowThreshold*ratio, kernel_size );
    dilate( CannyGray, CannyGray, element );
    //imshow("Canny", CannyGray);
    ///--------Result: dstGray----------------------------//
    bitwise_and(adapThreshGray, CannyGray, dstGray);
    //---------------------------------------------------//
    Mat ROI(dstGray, Rect(left_width,up_height,(right_width-left_width-1),
                            (down_height-up_height-1)));
    ROI.copyTo(cropGray);
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    //--------------getCannyObject------------------------//
    const uint16_t width = cropGray.cols;
    const uint16_t height = cropGray.rows;
    //imshow("cropGray",cropGray);
    Mat frame = Mat::zeros(Size(width, height), CV_8U);
    Mat frame1 = frame.clone();
    Mat frame2 = frame.clone();
    ///////////////////////////////////////////////////////////
    for(uint16_t i=0; i<width; i++)
    {
        uint16_t index1;
        uint16_t index2 = 0;
        for(uint16_t j=0; j<height; j++)
        {
            if (cropGray.at<uchar>(j,i) == 255)
            {
                frame1.at<uchar>(j,i) = 255;
                index1 = j+1;
                break;
            }
        }
        for(uint16_t j=index1; j<height; j++)
        {
            if (cropGray.at<uchar>(j,i) == 255)
            {
                frame1.at<uchar>(j,i) = 255;
                frame1.at<uchar>(index2,i) = 0;
                index2 = j;
            }
        }
    }
    for(uint16_t i=0; i<height; i++)
    {
        uint16_t index1;
        uint16_t index2 = 0;
        for(uint16_t j=0; j<width; j++)
        {
            if (cropGray.at<uchar>(i,j) == 255)
            {
                frame2.at<uchar>(i,j) = 255;
                index1 = j+1;
                break;
            }
        }
        for(uint16_t j=index1; j<width; j++)
        {
            if (cropGray.at<uchar>(i,j) == 255)
            {
                frame2.at<uchar>(i,j) = 255;
                frame2.at<uchar>(i,index2) = 0;
                index2 = j;
            }
        }
    }
    //---------Result: frame--------------------------------------//
    bitwise_or(frame1,frame2,frame);
    //imshow("frame", frame);
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    //-----------------getCorners---------------------------------//
    vector<Point2f> PointCorner;

    ///////////////////////////////////////////////////////////////
    vector<Vec4i> lines;
    int egdeSet[40][4] = {0};
    int fourEdge[4][4] = {0};
    int fourPoint[4][2] = {0};
    float ablines[4][2] = {0.0f};

    HoughLinesP(frame, lines, 1, CV_PI/180, 14, 10, 700 );
    int numberLines = lines.size();
    uint8_t indexNext;
    float lengthMax = 0;
    for (uint8_t i = 0; i < numberLines; i++)
    {
        Vec4i l = lines[i];
        float length = (float)((l[0]-l[2])*(l[0]-l[2])+(l[1]-l[3])*(l[1]-l[3]));
        if (length > lengthMax)
        {
            lengthMax = length;
            indexNext = i;
        }
        egdeSet[i][0] = l[0];
        egdeSet[i][1] = l[1];
        egdeSet[i][2] = l[2];
        egdeSet[i][3] = l[3];
    }
    for (uint8_t index = 0; index<4; index++)
    {
        bool flag_indexNext = false;
        float a = (float)(egdeSet[indexNext][1]-egdeSet[indexNext][3])/(float)(egdeSet[indexNext][0]-egdeSet[indexNext][2]);
        float b = (float)egdeSet[indexNext][1] - a*(float)egdeSet[indexNext][0];
        int vectorU1 = egdeSet[indexNext][0]-egdeSet[indexNext][2];
        int vectorU2 = egdeSet[indexNext][1]-egdeSet[indexNext][3];
        float length = 0.0f; 
        for (int8_t i = 0; i < numberLines; i++)
        {
            float distance1 = (a*(float)egdeSet[i][0]-(float)egdeSet[i][1]+b)*(a*(float)egdeSet[i][0]
                                -(float)egdeSet[i][1]+b)/(a*a+b*b);
            float distance2 = (a*(float)(egdeSet[i][2])-(float)(egdeSet[i][3])+b)*(a*(float)(egdeSet[i][2])
                                -(float)(egdeSet[i][3])+b)/(a*a+b*b);
            float distanceSum = distance1+distance2;
            if (distanceSum < 0.02f)
            {
                float lengthnow = (float)(egdeSet[i][0]-egdeSet[i][2])*(float)(egdeSet[i][0]-egdeSet[i][2])
                                    +(float)(egdeSet[i][1]-egdeSet[i][3])*(float)(egdeSet[i][1]-egdeSet[i][3]);
                if (lengthnow > length)
                {
                    length = lengthnow;
                    fourEdge[index][0] = egdeSet[i][0];
                    fourEdge[index][1] = egdeSet[i][1];
                    fourEdge[index][2] = egdeSet[i][2];
                    fourEdge[index][3] = egdeSet[i][3];
                }
                numberLines--;
                egdeSet[i][0] = egdeSet[numberLines][0];
                egdeSet[i][1] = egdeSet[numberLines][1];
                egdeSet[i][2] = egdeSet[numberLines][2];
                egdeSet[i][3] = egdeSet[numberLines][3];
                i--;
            }
            else if (flag_indexNext == false)
            {
                int vectorV1 = egdeSet[i][0]-egdeSet[i][2];
                int vectorV2 = egdeSet[i][1]-egdeSet[i][3];
                float angle = (float)((vectorU1*vectorV1+vectorU2*vectorV2)
                        *(vectorU1*vectorV1+vectorU2*vectorV2))
                        /(float)((vectorU1*vectorU1+vectorU2*vectorU2)
                        *(vectorV1*vectorV1+vectorV2*vectorV2));
                if(angle <= 0.0013f)
                {
                    flag_indexNext = true;
                    indexNext = i;
                }
            }
        }
        if ((flag_indexNext==false) && (index<3))
        {
            PointCorner.push_back(Point(0.0f,0.0f));
            return PointCorner;
        }
    }
    if(numberLines>0)
    {
        PointCorner.push_back(Point(0.0f,0.0f));
        return PointCorner;   
    }
    ///////////////////////////////////////////////////////////////
    for (uint8_t i=0; i<4; i++)
    {
        ablines[i][0] = (float)(fourEdge[i][1]-fourEdge[i][3])/(float)(fourEdge[i][0]-fourEdge[i][2]);
        ablines[i][1] = (float)fourEdge[i][1] - ablines[i][0]*(float)fourEdge[i][0];
    }
    float temp1[1][2] = {{ablines[1][0], ablines[1][1]}};
    ablines[1][0] = ablines[2][0];
    ablines[1][1] = ablines[2][1];
    ablines[2][0] = temp1[0][0];
    ablines[2][1] = temp1[0][1];
    for (uint8_t i=0; i<2; i++)
    {
        float coordinatesX1 = ((ablines[2][1]-ablines[i][1])/(ablines[i][0]-ablines[2][0]));
        fourPoint[2*i][0] = (int16_t)coordinatesX1; 
        fourPoint[2*i][1] = (int16_t)(ablines[2][0]*coordinatesX1+ablines[2][1]);
        float coordinatesX2 = ((ablines[3][1]-ablines[i][1])/(ablines[i][0]-ablines[3][0]));
        fourPoint[2*i+1][0] = (int16_t)coordinatesX2;
        fourPoint[2*i+1][1] = (int16_t)(ablines[3][0]*coordinatesX2+ablines[3][1]);
    }
    int temp2[1][2] = {{fourPoint[2][0], fourPoint[2][1]}};
    fourPoint[2][0] = fourPoint[3][0];
    fourPoint[2][1] = fourPoint[3][1];
    fourPoint[3][0] = temp2[0][0];
    fourPoint[3][1] = temp2[0][1];
    for (uint8_t index=0; index<4; index++)
    {
        if (frame.at<uchar>(fourPoint[index][1],fourPoint[index][0])==0)
        {
            uint8_t r = 1;
            int16_t pre_x = fourPoint[index][0];
            int16_t pre_y = fourPoint[index][1];
            do
            {
                int8_t a = 0,b = 0;
                int16_t x = pre_x-r;
                int16_t y = pre_y-r;
                for (uint8_t i = 0; i < 8*r; i++)
                {
                    if (i<=2*r)
                    {
                        a = i;
                    }
                    else if (i<=4*r)
                    {
                        b = i-2*r;
                    }
                    else if (i<=6*r)
                    {
                        a = 6*r-i;
                    }
                    else
                    {
                        b = 8*r-i;
                    }
                    if(frame.at<uchar>(y+b,x+a) == 255)
                    {
                        fourPoint[index][0] = x+a;
                        fourPoint[index][1] = y+b;
                        r = radius-1;
                        break;
                    }
                }
                r++; 
            } while (r < radius);
        }
    }
    //------------The Final Results Find PointCorner-------------//
    ///////////////////////////////////////////////////////////////
    for (uint8_t i=0; i<4; i++)
    {
        fourPoint[i][0] += left_width;
        fourPoint[i][1] += up_height;
        PointCorner.push_back(Point2i(fourPoint[i][0],fourPoint[i][1]));
    }
    return PointCorner;
}