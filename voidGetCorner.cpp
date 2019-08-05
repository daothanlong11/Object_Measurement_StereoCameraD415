#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;


/// Global variables
Mat src_h, src_gray_h;
const char* source_window = " Detect Corner";
const char* thresh_window = "Thresh gray";
RNG rng(12345);

// Function get corner
vector<Point2f> getCorner(Mat src_gray);

/** @function main */
int main( int argc, char** argv )
{
    /// Load source image and convert it to gray
    VideoCapture cap;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY; 
    cap.open(deviceID+apiID);

    namedWindow(source_window, CV_WINDOW_AUTOSIZE);
    namedWindow(thresh_window, CV_WINDOW_AUTOSIZE);


    /// Create Trackbar to set the number of corners
    
    for(;;)
    {
        cap >> src_h;
        if( src_h.empty() )
            break;
        cvtColor( src_h, src_gray_h, CV_BGR2GRAY );

        vector<Point2f> hai = getCorner(src_gray_h);
        for (int i=0; i<4; i++)
        {
            circle( src_h, Point(hai[i]), 2, Scalar(0, 0, 255), -1);
        }
        imshow( source_window, src_h);
        char c = (char)waitKey(30);
        if( c == 'q' || c == 'Q' || c == 27 )
            break;
    }
    return(0);
}



vector<Point2f> getCorner(Mat src_gray)
{
    // Tham so khoi tao ban dau
    vector<Point2f> corners;
    int maxCorners = 7;
    const int  thresh_index = 11, block_index = 4;
    const double qualityLevel = 0.01;
    const double minDistance = 10;
    const int blockSize = 3;
    const bool useHarrisDetector = false;
    const double k = 0.04;

    // khai bao cac goc A, B, C, D mat tren cua vat the
    vector<Point2f> PointCorner;
    static Point2f pointA;
    static Point2f pointB;
    static Point2f pointC;
    static Point2f pointD;

    static double distanceC;
    static double distanceD;
    static float d = 0;
    Mat thresh_gray;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));

    // Tien xu ly anh
    adaptiveThreshold(src_gray, thresh_gray, 255, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV , (2*block_index+1), thresh_index);
    morphologyEx(thresh_gray, thresh_gray, MORPH_CLOSE, kernel);
    // Ham tim cac goc
    if( maxCorners < 1 ) { maxCorners = 1; }
    goodFeaturesToTrack( src_gray,
                        corners,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k );
    
    // Ve diem
    for (int i=0; i<corners.size(); i++)
    {
        circle( src_h, Point(corners[i]), 2, Scalar(255, 0, 0), -1);
    }

    // Tim chinh xac 4 goc
    float matrixPoint1[maxCorners][2];
    float matrixPoint2[maxCorners-1][2];

    for(int i=0; i< maxCorners; i++)
    {
        matrixPoint1[i][0] = corners[i].x;
        matrixPoint1[i][1] = corners[i].y;
    }
    
    // Tim diem A
    int index1 = 0;
    if(thresh_gray.at<uchar>(pointA.y,pointA.x) != 255)
    {
        pointA.x = 1000;
        pointA.y = 1000;
    }
    for(int i=0; i < maxCorners; i++)
    {
        if(pointA.x > matrixPoint1[i][0])
        {
            pointA.x = matrixPoint1[i][0];
            pointA.y = matrixPoint1[i][1];
        }
        else
        {
            matrixPoint2[index1][0] =  matrixPoint1[i][0];
            matrixPoint2[index1][1] =  matrixPoint1[i][1];
            index1++;
        }
    }
    
    // Tim diem C
    int index2 = 0;
    if(thresh_gray.at<uchar>(pointC.y,pointC.x) != 255)
    {
        distanceC = 0;
    }
    for (int i = 0; i < index1; i++)
    {
        float distance = (pointA.x - matrixPoint2[i][0])*(pointA.x - matrixPoint2[i][0]) + (pointA.y - matrixPoint2[i][1])*(pointA.y - matrixPoint2[i][1]);
        if (distanceC < distance)
        {
            pointC.x = matrixPoint2[i][0];
            pointC.y = matrixPoint2[i][1];
            distanceC = distance;
        }
        else
        {
            matrixPoint1[index2][0] =  matrixPoint2[i][0];
            matrixPoint1[index2][1] =  matrixPoint2[i][1];
            index2++;
        }
    }

    // Tim diem B
    if(thresh_gray.at<uchar>(pointB.y,pointB.x) != 255)
    {
        d = 0;
    }
    float a = (pointA.y-pointC.y)/(pointA.x-pointC.x);
    float b = pointA.y - a*pointA.x;
    index1 = 0;
    for(int i=0; i<index2; i++)
    {
        float d_temp = (a*matrixPoint1[i][0]-matrixPoint1[i][1]+b)*(a*matrixPoint1[i][0]-matrixPoint1[i][1]+b)/(a*a+b*b);
        if(d < d_temp)
        {
            pointB.x = matrixPoint1[i][0];
            pointB.y = matrixPoint1[i][1];
            d = d_temp;
        }
        else
        {
            matrixPoint2[index1][0] =  matrixPoint1[i][0];
            matrixPoint2[index1][1] =  matrixPoint1[i][1];
            index1++;
        }
    }

    // Tim diem D
    if(thresh_gray.at<uchar>(pointD.y,pointD.x) != 255)
    {
        distanceD = 0;
    }
    index2 = 0;
    for (int i = 0; i < index1; i++)
    {
        float distance = (pointB.x - matrixPoint2[i][0])*(pointB.x - matrixPoint2[i][0]) + (pointB.y - matrixPoint2[i][1])*(pointB.y - matrixPoint2[i][1]);
        if (distanceD < distance)
        {
            pointD.x = matrixPoint2[i][0];
            pointD.y = matrixPoint2[i][1];
            distanceD = distance;
        }
    }

    PointCorner.push_back(pointA);
    PointCorner.push_back(pointB);
    PointCorner.push_back(pointC);
    PointCorner.push_back(pointD);
    
    imshow(thresh_window, thresh_gray);
    return PointCorner;
}

