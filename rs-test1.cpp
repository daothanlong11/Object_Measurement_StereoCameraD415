// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

using namespace cv;
using namespace std;


vector<Point2f> distanceframe(rs2::depth_frame depth, int width, int height);
////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) try
{
    int w, h;
    rs2::align align_to(RS2_STREAM_DEPTH);
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::hole_filling_filter hole;
    hole.set_option(RS2_OPTION_HOLES_FILL,1);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 0);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 0);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 0);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);


    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    
    //VideoWriter video("rs-depVideo2.avi",CV_FOURCC('M','J','P','G'),10, Size(640,480)); 

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frameset data1 = data;
        data1 = data1.apply_filter(hole);
        data1 = data1.apply_filter(align_to);

        //rs2::frame imageColor = data.get_color_frame();
        //rs2::frame ir_frame = data.first(RS2_STREAM_INFRARED);
        
        rs2::frame dep_frame = data1.get_color_frame();
        rs2::depth_frame depth = data1.get_depth_frame();
        

        // Query frame size (width and height)
        w = depth.as<rs2::video_frame>().get_width();
        h = depth.as<rs2::video_frame>().get_height();
        
        //cout<<"width: "<<w<<endl;
        //cout<<"height: "<<h<<endl;
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image1(Size(w, h), CV_8UC3, (void*)imageColor.get_data(), Mat::AUTO_STEP);
        //Mat image2(Size(w, h), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);
        Mat image3(Size(w, h), CV_8UC3, (void*)dep_frame.get_data(), Mat::AUTO_STEP);

        
        vector<Point2f> hai = distanceframe(depth, w, h);
        for(int i = 0; i < 4; i++ )
        line(image3, hai[i], hai[(i+1)%4], Scalar(0, 255, 0), 1, LINE_AA);
        circle(image3, Point2f(w/2, h/2), 3, Scalar(0, 0, 255), -1);
        circle(image3, Point2f(w-40, 20), 3, Scalar(0, 0, 255), -1);
        //Mat colorhai;
        //cvtColor(hai, colorhai, CV_GRAY2BGR);
        //video.write(colorhai);

        //mau.create(image3.size(), image3.type());
        //mau = Scalar::all(0);
        //image3.copyTo(mau, hai);
        // Update the window with new data
        imshow(window_name, image3);
        //imshow("frame depth", hai);
        //imshow("mau", mau);
        //waitKey(0);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

vector<Point2f> distanceframe(rs2::depth_frame depth, int width, int height)
{
    int64 t0 = cv::getTickCount();

    float vicinityDistance = 0.012f;
    float setVicinityDistance[6] = {0.001f, 0.003f, 0.011f, 0.0168f, 0.0275f, 0.032f};
    
    Mat frameDepth = Mat::zeros(Size(width, height), CV_8U);
    float objectHeight;

    //float distanceCenter = depth.get_distance(width/2, height/2);
    float maxDistance = depth.get_distance(width-40, 10);
    float objectDistance = depth.get_distance(width/2, height/2);
    float sumMaxDistance = 0.0f;
    float sumObjectDistance = 0.0f;
    uint sumObjectPoint = 0;
    uint sumMaxPoint = 0;
    
    cout<<"object distance: "<<objectDistance<<endl;
    cout<<"max distance: "<<maxDistance<<endl;

    for (uint16_t x = 0; x < width; x++)
    {
        for (uint16_t y = 0; y < height; y++)
        {
            float distance = depth.get_distance(x,y);
            if (abs(distance - objectDistance) <= vicinityDistance)
            {
                frameDepth.at<uchar>(y,x) = 255;
                sumObjectDistance += distance;
                sumObjectPoint++;
            }
            else if (abs(distance - maxDistance) <= vicinityDistance)
            {
                sumMaxDistance += distance;
                sumMaxPoint++;
            }
        }
    }

    //-------------Result Object Height--------------------//
    objectHeight = (sumMaxDistance/(float)sumMaxPoint - sumObjectDistance/(float)sumObjectPoint)*100;
    //cout<<"chieu cao vat the: "<<objectHeight<<endl;
    objectDistance = sumObjectDistance/(float)sumObjectPoint;
    /////////////////////////////////////////////////////////
    vector<Point2f> fourCorner;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat imageContours = frameDepth.clone();
    /// Find contours
    findContours( imageContours, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    //cout<<"number contours: "<<contours.size()<<endl;
    
    int indexmaxContours[2] = {0, 0};
    for (uint8_t i=0; i<contours.size(); i++)
    {
        if (contours[i].size() > indexmaxContours[1])
        {
            indexmaxContours[0] = i;
            indexmaxContours[1] = contours[i].size();
        }
    }

    Point2f fourPoint[4];
    RotatedRect box = minAreaRect(contours[indexmaxContours[0]]);
    box.points(fourPoint);

    for (uint8_t i = 0; i < 4; i++)
    {
        fourCorner.push_back(fourPoint[i]);
    }
    fourCorner.push_back(Point2f(objectHeight, objectDistance));
    imshow("frame depth", frameDepth);
    //////////////////////////////////////////////////////////////////
    int64 t1 = cv::getTickCount();
    double secs = (t1-t0)/cv::getTickFrequency();
    cout<<"thoi gian: "<<secs<<endl;
    //cout<<"khoang cach: "<<sumDistan<<endl;
    return fourCorner;
}