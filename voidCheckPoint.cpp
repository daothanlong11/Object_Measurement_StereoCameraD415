#include "opencv2/opencv.hpp"

////////////////////////////////////////////////////////////
vector<Point2f> checkPoint(vector<Point2f> PointCheck, Mat frameDis)
{

    #define radius 10
    const uint16_t width = frameDis.cols;
    const uint16_t height = frameDis.rows;
    float fourPoint[4][2];

    Mat frameGray = Mat::zeros(Size(width, height), CV_8U);
    const Point2f pts = (const cv::Point2f*) Mat(PointCheck).data;
    polylines(frameGray, &pts, 4, 1, true, 255);

    for(uint8_t i=0; i<4; i++)
    {
        fourPoint[i][0] = PointCheck[i].x;
        fourPoint[i][1] = PointCheck[i].y;
    }

    for (uint8_t index=0; index<4; index++)
    {
        if (frameDis.get_distance(fourPoint[i][0], fourPoint[i][1]) == 0)
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
                    if((frameDis.get_distance(fourPoint[i][0], fourPoint[i][1]) > 0.5f) 
                                    && (frameGray.at<uchar>(y+b,x+a) == 255))
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
    PointCheck.clear();
    for (uint8_t i=0; i<4; i++)
    {
        PointCheck.push_back(Point2i(fourPoint[i][0],fourPoint[i][1]));
    }
    return PointCheck;
}
