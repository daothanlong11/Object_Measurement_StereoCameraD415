//include realsense2 library

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
//#include "/home/l/librealsense/examples/example.hpp"

//include opencv library
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//include  some library for data-structure
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std;
using namespace cv;

using pixel = pair<float,float>;
pixel u,v,u1,v1;


float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v);
void checkPoint1(const rs2::depth_frame& frame,pixel u,pixel v);
vector<Point2f> checkPoint2(vector<Point2f> PointCheck, const rs2::depth_frame& frameDis);

int const low_thresh = 50;
int const high_thresh = 250;
int kernel_size = 3;
Mat blur_img,gray,canny_img;
Mat img_processing(Mat img)
{
    cvtColor(img,gray,COLOR_BGR2GRAY);
    //GaussianBlur(gray,blur_img,Size(3,3),0.1);
    //Canny(blur_img,canny_img,low_thresh,high_thresh,kernel_size);
    return gray;
}



void CallBackFunc(int event,int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        u1.first = x;
        u1.second = y;
        float x1 = x / 1280.0 ;
        float y1 = y / 720.0;
        u.first = x1 * 1280.0 ;
        u.second = y1 * 720.0;
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //cout<<"u first: "<<u.first<<endl;
        //cout<<"u second: "<<u.second<<endl;
    }

    if (event == EVENT_RBUTTONDOWN)
    {
        v1.first = x;
        v1.second = y;
        float x2 = x / 1280.0;
        float y2 = y / 720.0;
        v.first = x2 * 1280.0;
        v.second = y2 * 720.0;
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //cout<<"v first: "<<v.first<<endl;
        //cout<<"v second: "<<v.second<<endl;
    }
    
}

int main(int argc, char * argv[]) try
{   
    const auto window_name = "img";
    namedWindow(window_name,WINDOW_AUTOSIZE);
    //window app(640,360,"IMG");
    
    //rs2::colorizer color_map;
    
    rs2::colorizer color_map;
    // Use black to white color map
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    // Decimation filter reduces the amount of data (while preserving best samples)
    rs2::decimation_filter dec;
    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter spat;
    // Enable hole-filling
    // Hole filling is an agressive heuristic and it gets the depth wrong many times
    // However, this demo is not built to handle holes
    // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    // Define temporal filter
    rs2::temporal_filter temp;
    // Spatially align all streams to depth viewport
    // We do this because:
    //   a. Usually depth has wider FOV, and we only really need depth for this demo
    //   b. We don't want to introduce new holes
    rs2::align align_to(RS2_STREAM_DEPTH);


    rs2::config cfg;
    
    cfg.enable_stream(RS2_STREAM_DEPTH); 
    cfg.enable_stream(RS2_STREAM_COLOR,RS2_FORMAT_BGR8);
    
    rs2::pipeline p;
    auto profile = p.start(cfg);
    auto sensor = profile.get_device().first<rs2::depth_sensor>();

    //set the device to high accuraccy preset of the d400
    if (sensor && sensor.is<rs2::depth_stereo_sensor>())
    {
        sensor.set_option(RS2_OPTION_VISUAL_PRESET,RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    }

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2::frame_queue postprocessed_frames;
    std::atomic_bool alive{ true };

    std::thread video_processing_thread([&]() {
        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset data;
            if (p.poll_for_frames(&data))
            {
                // First make the frames spatially aligned
                data = data.apply_filter(align_to);

                // Decimation will reduce the resultion of the depth image,
                // closing small holes and speeding-up the algorithm
                //data = data.apply_filter(dec);

                // To make sure far-away objects are filtered y
                // we try to switch to disparity domain
                data = data.apply_filter(depth2disparity);

                // Apply spatial filtering
                //data = data.apply_filter(spat);

                // Apply temporal filtering
                data = data.apply_filter(temp);

                // If we are in disparity domain, switch back to depth
                data = data.apply_filter(disparity2depth);

                //// Apply color map for visualization of depth
                data = data.apply_filter(color_map);
                // Send resulting frames for visualization in the main thread
                postprocessed_frames.enqueue(data);
            }
        }
    });


    rs2::frameset current_frameset;
    while(waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {   
        postprocessed_frames.poll_for_frame(&current_frameset);

        if (current_frameset)
        {
        auto fr = current_frameset.get_color_frame();
        auto depth = current_frameset.get_depth_frame();
        auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

        const int w = fr.as<rs2::video_frame>().get_width();
        const int h = fr.as<rs2::video_frame>().get_height();
        
        Mat image(Size(w,h),CV_8UC3,(void*)fr.get_data(),Mat::AUTO_STEP);
        //app.show(data);
        //imshow(window_name,image);


        //processing image
        //Mat final_img = img_processing(image);
        //Mat final_img = image;
        setMouseCallback(window_name,CallBackFunc,NULL);
        //cout<<"u first: "<<u.first<<endl;
        //cout<<"u second: "<<u.second<<endl;
        //cout<<"v first: "<<v.first<<endl;
        //cout<<"v second: "<<v.second<<endl;
        /*
        vector<Point2f> diem,diem1;
        diem.push_back(Point2f(720,146));
        diem.push_back(Point2f(835,130));
        diem.push_back(Point2f(853,306));
        diem.push_back(Point2f(740,323));
        
        circle(image,Point2f(720,146),5,Scalar(0,0,255),CV_FILLED,2,0);
        circle(image,Point2f(835,130),5,Scalar(0,0,255),CV_FILLED,2,0);
        circle(image,Point2f(740,323),5,Scalar(0,0,255),CV_FILLED,2,0);
        circle(image,Point2f(853,306),5,Scalar(0,0,255),CV_FILLED,2,0);
        diem1 = checkPoint2(diem,depth);
        
        circle(image,diem1[0],3,Scalar(0,255,0),CV_FILLED,2,0);
        circle(image,diem1[1],3,Scalar(0,255,0),CV_FILLED,2,0);
        circle(image,diem1[2],3,Scalar(255,0,0),CV_FILLED,2,0);
        circle(image,diem1[3],3,Scalar(255,0,0),CV_FILLED,2,0);
        cout<<"check point"<<endl;
        //vector<Point2f> pts = checkPoint();
        //circle(final_img,Point(u1.first,u1.second),5,Scalar(0,0,255),CV_FILLED,3,0);
        //circle(final_img,Point(v1.first,v1.second),5,Scalar(0,0,255),CV_FILLED,3,0);
        //cout<<"dist1: "<<depth.get_distance(720,146)<<endl;
        //cout<<"dist2: "<<depth.get_distance(935,130)<<endl;
        //cout<<"dist3: "<<depth.get_distance(740,323)<<endl;
        //cout<<"dist4: "<<depth.get_distance(835,306)<<endl;
        cout<<"cout 2"<<endl;
        cout<<"dist1.2: "<<depth.get_distance(diem1[0].x,diem1[0].y)<<endl;
        cout<<"dist2.2: "<<depth.get_distance(diem1[1].x,diem1[1].y)<<endl;
        cout<<"dist3.2: "<<depth.get_distance(diem1[2].x,diem1[2].y)<<endl;
        cout<<"dist4.2: "<<depth.get_distance(diem1[3].x,diem1[3].y)<<endl;
        */
        float wid = depth.get_width();
        float hei = depth.get_height();
        //cout<<"wid: "<<wid<<endl;
        //cout<<"hei: "<<hei<<endl;
        //cout<<"width: "<<w<<endl;
        //cout<<"height: "<<h<<endl;



       

        circle(image,Point(u1.first,u1.second),5,Scalar(255,0,0),CV_FILLED,3,0);
        circle(image,Point(v1.first,v1.second),5,Scalar(255,0,0),CV_FILLED,3,0);
        
        
        float air_dist = dist_3d(depth, u, v);

        cout<<"distance between 2 point is: "<<air_dist<<endl;
        imshow(window_name,image);
        pixel u,v;
        }
    }
    alive = false;
    video_processing_thread.join();
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

float dist_3d(const rs2::depth_frame& frame,pixel u,pixel v)
{
    float upixel[2];
    float upoint[3];

    float vpixel[2];
    float vpoint[3];

    upixel[0] = u.first;
    upixel[1] = u.second;
    vpixel[0] = v.first;
    vpixel[1] = v.second;

    auto udist = frame.get_distance(upixel[0],upixel[1]);
    auto vdist = frame.get_distance(vpixel[0],vpixel[1]);

    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    rs2_deproject_pixel_to_point(upoint,&intr,upixel,udist);
    rs2_deproject_pixel_to_point(vpoint,&intr,vpixel,vdist);

    float distance = sqrt(pow(upoint[0] - vpoint[0], 2) +
                pow(upoint[1] - vpoint[1], 2) +
                pow(upoint[2] - vpoint[2], 2));

    return distance;
}

void checkPoint1(const rs2::depth_frame& frame,pixel u,pixel v)
{
     
    //////////////////// p1 ./////////////////
    if(frame.get_distance(u.first,u.second)<0.00001)
    {
        vector<Point2f> vertifyPoints;
        pixel goc;
        goc.first = u.first;
        goc.second = u.second; 
        int radius = 1;
        int phantu = 0;
        //int kernel = 2*radius+1;
        while(phantu==0)
        {
            goc.first = goc.first+radius;
            goc.second = goc.second+radius;
            for(int i=0;i<=radius;i++)
            {
                float dis1 = frame.get_distance(goc.first,goc.second-i);
                float dis2 = frame.get_distance(goc.first-i,goc.second);
                if(dis1 > 0.01){vertifyPoints.push_back(Point2f(goc.first,goc.second-i));}
                if(dis2 > 0.01){vertifyPoints.push_back(Point2f(goc.first-i,goc.second));}
            }
            radius++;
            phantu = vertifyPoints.size();
        }
        float chieu_x,chieu_y;
        for(int i=0;i<phantu;i++)
        {
            
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
        }
        u1.first = chieu_x/phantu;
        u1.second = chieu_y/phantu;
    }

    ////////////////// p2 ////////////////
    if(frame.get_distance(v.first,v.second)<0.00001)
    {
        vector<Point2f> vertifyPoints;
        pixel goc;
        goc.first = v.first;
        goc.second = v.second; 
        int radius = 1;
        int phantu = 0;
        //int kernel = 2*radius+1;
        while(phantu==0)
        {
            goc.first = goc.first-radius;
            goc.second = goc.second+radius;
            for(int i=0;i<=radius;i++)
            {
                float dis1 = frame.get_distance(goc.first,goc.second-i);
                float dis2 = frame.get_distance(goc.first+i,goc.second);
                if(dis1 > 0.01){vertifyPoints.push_back(Point2f(goc.first,goc.second-i));}
                if(dis2 > 0.01){vertifyPoints.push_back(Point2f(goc.first+i,goc.second));}
            }
            radius++;
            phantu = vertifyPoints.size();
        }
        float chieu_x,chieu_y;
        for(int i=0;i<phantu;i++)
        {
            
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
           
            
        }
        v1.first = chieu_x/phantu;
        v1.second = chieu_y/phantu;
    } 
   
}





////////////////////////////////////////////////////////////
vector<Point2f> checkPoint2(vector<Point2f> PointCheck, const rs2::depth_frame& frameDis)
{
    vector<Point2f> pointCorner;
     cout<<"dist1: "<<frameDis.get_distance(720,146)<<endl;
        cout<<"dist2: "<<frameDis.get_distance(835,130)<<endl;
        cout<<"dist3: "<<frameDis.get_distance(740,323)<<endl;
        cout<<"dist4: "<<frameDis.get_distance(853,306)<<endl;
    #define radius 20
    const uint16_t width = 1280;
    const uint16_t height = 720;
    float fourPoint[4][2];

    Mat frameGray = Mat::zeros(Size(width, height), CV_8U);
    Point pt[4];
    cout<<"pointcheck"<<PointCheck[1].x<<endl;
    for(uint8_t i=0; i<4; i++)
    {
        fourPoint[i][0] = PointCheck[i].x;
        fourPoint[i][1] = PointCheck[i].y;
        pt[i] = PointCheck[i];
    }
    fillConvexPoly(frameGray, pt, 4, 255);
    imshow("frameGray", frameGray);
    for (uint8_t index=0; index<4; index++)
    {
        if (frameDis.get_distance(fourPoint[index][0], fourPoint[index][1]) < 0.01f)
        {
            uint8_t r = 1;
            cout<<"here"<<endl;
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
                    if((frameDis.get_distance(x+a, y+b) > 0.01f)
                                    && (frameGray.at<uchar>(y+b,x+a) == 255))
                    {
                        fourPoint[index][0] = x+a;
                        fourPoint[index][1] = y+b;
                        r = radius-1;
                        cout<<"here 1"<<endl;
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
        pointCorner.push_back(Point2i(fourPoint[i][0],fourPoint[i][1]));
    }
    return pointCorner;
}