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


int const low_thresh = 50;
int const high_thresh = 250;
int kernel_size = 3;
Mat frame,gray,canny_img,blur_img,threshold_output;

bool comparePointx(Point i1, Point i2) 
{ 
    return (i1.x < i2.x); 
} 
bool comparePointy(Point i1, Point i2) 
{ 
    return (i1.y < i2.y); 
} 


vector<Point2f> img_processing(Mat img)
{
    cvtColor(img,gray,COLOR_BGR2GRAY);
    blur(gray,blur_img,Size(3,3));

    vector<Point2f> corners;
    double qualityLevel = 0.01;
    int minDistance = MAX(minDistance,1);
    //Mat copy = frame.clone();
    //Canny(gray,canny_img,thresh1,200);
    threshold(blur_img,threshold_output,0,255,THRESH_BINARY);

    goodFeaturesToTrack(gray,corners,20,qualityLevel,5);
        
    for (size_t i =0;i<corners.size();i++)
    {
        circle(img,corners[i],3,Scalar(0,0,125),-1);
    }
    
    //find good corner
    vector<Point2f> corners_sort;
    corners_sort = corners;
    vector<Point2f> corners_sort_x;
    vector<Point2f> corners_sort_y;
    vector<Point2f> contours;
    sort(corners.begin(),corners.end(),comparePointx);
    corners_sort_x = corners;
    sort(corners.begin(),corners.end(),comparePointy);
    corners_sort_y = corners;
    
    contours.push_back(corners_sort_x[0]);
    contours.push_back(corners_sort_x[corners_sort_x.size()-1]);
    contours.push_back(corners_sort_y[0]);
    contours.push_back(corners_sort_y[corners_sort_y.size()-1]);
    
    line(img,contours[0],contours[2],Scalar(0,125,0),2);
    line(img,contours[2],contours[1],Scalar(0,125,0),2);
    line(img,contours[1],contours[3],Scalar(0,125,0),2);
    line(img,contours[3],contours[0],Scalar(0,125,0),2);

    return contours;
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
    spat.set_option(RS2_OPTION_HOLES_FILL,5); // 5 = fill all the zero pixels
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
        sensor.set_option(RS2_OPTION_VISUAL_PRESET,RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY);
    }

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2::frame_queue postprocessed_frames;
    rs2::frame_queue postprocessed_frames1;
    std::atomic_bool alive{ true };

    std::thread video_processing_thread([&]() {
        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset data;
            if (p.poll_for_frames(&data))
            {   
                data = data.apply_filter(color_map);
                postprocessed_frames1.enqueue(data);
                // First make the frames spatially aligned
                data = data.apply_filter(align_to);

                // Decimation will reduce the resultion of the depth image,
                // closing small holes and speeding-up the algorithm
                data = data.apply_filter(dec);

                // To make sure far-away objects are filtered y
                // we try to switch to disparity domain
                data = data.apply_filter(depth2disparity);

                // Apply spatial filtering
                data = data.apply_filter(spat);

                // Apply temporal filtering
                data = data.apply_filter(temp);

                // If we are in disparity domain, switch back to depth
                data = data.apply_filter(disparity2depth);

                //// Apply color map for visualization of depth
                // Send resulting frames for visualization in the main thread
                postprocessed_frames.enqueue(data);
                
                
            }
        }
    });


    rs2::frameset current_frameset,current_frameset1;
    while(waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {   
        postprocessed_frames.poll_for_frame(&current_frameset);
        postprocessed_frames1.poll_for_frame(&current_frameset1);

        if (current_frameset)
        {
        auto fr1 = current_frameset1.get_color_frame();
        //auto fr = current_frameset.get_color_frame();
        auto depth = current_frameset.get_depth_frame();
        auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

        const int w = fr1.as<rs2::video_frame>().get_width();
        const int h = fr1.as<rs2::video_frame>().get_height();
        
        Mat image(Size(w,h),CV_8UC3,(void*)fr1.get_data(),Mat::AUTO_STEP);
        //app.show(data);
        //imshow(window_name,image);


        //processing image
        vector<Point2f> contours = img_processing(image);
        Mat final_img = image;
        u.first = contours[0].x;
        u.second = contours[0].y;
        v.first = contours[3].x;
        v.second = contours[3].y;
        //setMouseCallback(window_name,CallBackFunc,NULL);
        cout<<"u first: "<<u.first<<endl;
        cout<<"u second: "<<u.second<<endl;
        cout<<"v first: "<<v.first<<endl;
        cout<<"v second: "<<v.second<<endl;

        float wid = depth.get_width();
        float hei = depth.get_height();
        cout<<"wid: "<<wid<<endl;
        cout<<"hei: "<<hei<<endl;
        cout<<"width: "<<w<<endl;
        cout<<"height: "<<h<<endl;
        
        
        float air_dist = dist_3d(depth, u, v);

        cout<<"distance between 2 point is: "<<air_dist<<endl;
        imshow(window_name,final_img);
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

    upixel[0] = (u.first/1920)*640;
    upixel[1] = (u.second/1080)*360;
    vpixel[0] = (v.first/1920)*640;
    vpixel[1] = (v.second/1080)*360;

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