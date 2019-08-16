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

#define left_width 500
#define right_width 1500
#define up_height 200
#define down_height 1000

using pixel = pair<float,float>;
pixel u,v,g,k,u1,v1;

struct values
{
    vector<Point2f> cornerpoints;
    Mat white_frame;
};


float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v);
values getCorner(Mat src);
pixel points1,points2,points3,points4;
void checkPoint(const rs2::depth_frame& frame,pixel u,pixel v,pixel g,pixel k);


/* 
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
*/


void CallBackFunc(int event,int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        u1.first = x;
        u1.second = y;
        float x1 = x / 1920.0 ;
        float y1 = y / 1080.0;
        u.first = x1 * 640.0 ;
        u.second = y1 * 360.0;
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //cout<<"u first: "<<u.first<<endl;
        //cout<<"u second: "<<u.second<<endl;
    }

    if (event == EVENT_RBUTTONDOWN)
    {
        v1.first = x;
        v1.second = y;
        float x2 = x / 1920.0;
        float y2 = y / 1080.0;
        v.first = x2 * 640.0;
        v.second = y2 * 360.0;
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //cout<<"v first: "<<v.first<<endl;
        //cout<<"v second: "<<v.second<<endl;
    }
}



  
// Compares two intervals according to staring times. 
bool compareInterval(Point2f i1, Point2f i2) 
{ 
    return (i1.x < i2.x); 
} 

int main(int argc, char * argv[]) try
{   
   
    const auto window_name = "window";
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
    //spat.set_option(RS2_OPTION_HOLES_FILL,5); // 5 = fill all the zero pixels
    // Define temporal filter
    rs2::temporal_filter temp;
    // Spatially align all streams to depth viewport
    // We do this because:
    //   a. Usually depth has wider FOV, and we only really need depth for this demo
    //   b. We don't want to introduce new holes
    rs2::align align_to(RS2_STREAM_DEPTH);

    rs2::pipeline p;
    rs2::config cfg;
    
    cfg.enable_stream(RS2_STREAM_DEPTH); 
    cfg.enable_stream(RS2_STREAM_COLOR,RS2_FORMAT_BGR8);
    
    
    auto profile = p.start(cfg);
    auto sensor = profile.get_device().first<rs2::depth_sensor>();

    //set the device to high accuraccy preset of the d400
    if (sensor && sensor.is<rs2::depth_stereo_sensor>())
    {
        sensor.set_option(RS2_OPTION_VISUAL_PRESET,RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
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
        //auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

        const int w = fr1.as<rs2::video_frame>().get_width();
        const int h = fr1.as<rs2::video_frame>().get_height();
        float wid = depth.get_width();
        float hei = depth.get_height();
        
        Mat image(Size(w,h),CV_8UC3,(void*)fr1.get_data(),Mat::AUTO_STEP);
        Mat image1;
        cout<<"w: "<<w<<endl;
        cout<<"h: "<<h<<endl;
        cout<<"wid: "<<wid<<endl;
        cout<<"hei: "<<hei<<endl;
        setMouseCallback(window_name,CallBackFunc,NULL);
        circle(image,Point(u1.first,u1.second),5,Scalar(255,0,0),CV_FILLED,3,0);
        circle(image,Point(v1.first,v1.second),5,Scalar(255,0,0),CV_FILLED,3,0);
        /*
        values hai = getCorner(image);
        vector<Point2f> Corner = hai.cornerpoints;
        Mat white_frame = hai.white_frame;
        cout<<"not dump"<<endl;
        int sophantu = Corner.size();
        
        if (sophantu == 4)
        {
            Point2f p1,p2,p3,p4;
            //circle( image, Point(hai[0]), 3, Scalar(255, 0, 0), -1);
            //circle( image, Point(hai[1]), 3, Scalar(0, 255, 0), -1);
            //circle( image, Point(hai[2]), 3, Scalar(0, 0, 255), -1);
            //circle( image, Point(hai[3]), 3, Scalar(0, 0, 0), -1);
            for (int i=0; i<4; i++)
            {
                circle( image, Point(Corner[i]), 10, Scalar(0, 255, 0), -1);
            }
            ////////////////////// sort ///////////////////
            
            vector<Point2f> Corner1 = Corner;
            sort(Corner1.begin(),Corner1.end(),compareInterval);
            if(Corner1[0].y>Corner1[1].y)
            {
                p1 = Corner1[1];
                p4 = Corner1[0];
            }
            else
            {
                p4 = Corner1[1];
                p1 = Corner1[0];
            }
            if(Corner1[2].x>Corner1[3].x)
            {
                p2 = Corner1[3];
                p3 = Corner1[2];
            }
            else
            {
                p3 = Corner1[3];
                p2 = Corner1[2];
            }
            ////////////////////////////////////////////////////////

     
            u.first = p1.x;
            u.second = p1.y;
            v.first = p2.x;
            v.second = p2.y;
            g.first = p3.x;
            g.second = p3.y;
            k.first = p4.x;
            k.second = p4.y;
            checkPoint(depth,u,v,g,k);
            circle( image, Point(u.first,u.second), 3, Scalar(0, 255, 0), -1);
            circle( image, Point(v.first,v.second), 3, Scalar(0, 255, 0), -1);
            circle( image, Point(g.first,g.second), 3, Scalar(0, 255, 0), -1);
            circle( image, Point(k.first,k.second), 3, Scalar(0, 255, 0), -1);

            

            float air_dist1 = dist_3d(depth, u,v);
            float air_dist2 = dist_3d(depth, v, g);
            float air_dist3 = dist_3d(depth,g, k);
            cout<<"new distance"<<endl;
            cout<<"distance between 2 point 1 is: "<<air_dist1<<endl;
            cout<<"distance between 2 point 2 is: "<<air_dist2<<endl;
            cout<<"distance between 2 point 3 is: "<<air_dist3<<endl;
            
           //cout<<"point 0: "<<points[0].x<<" | "<<points[0].y<<endl;
           //cout<<"point 1: "<<points[1].x<<" | "<<points[1].y<<endl;
        
        }
        */
        //resize(image,image1,Size(640,360),0.5,0.5,INTER_LINEAR);
        imshow(window_name, image);
        pixel u,v;
        //waitKey(0);
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
void checkPoint(const rs2::depth_frame& frame,pixel u,pixel v,pixel g,pixel k)
{
     
    vector<Point2f> sendPoints;  
    u.first -= left_width;
    u.second -= up_height;
    v.first -= left_width;
    v.second -= up_height;
    g.first -= left_width;
    g.second -= up_height;
    k.first -= left_width;
    k.second -= up_height;

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
        for(int i=0;i<phantu;i++)
        {
            float chieu_x,chieu_y;
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
            u.first = chieu_x/phantu;
            u.second = chieu_y/phantu;
        }
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
        for(int i=0;i<phantu;i++)
        {
            float chieu_x,chieu_y;
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
            v.first = chieu_x/phantu;
            v.second = chieu_y/phantu;
            
        }
    } 


    ////////////////////// p3 //////////////////
    if(frame.get_distance(g.first,g.second)<0.00001)
    {
        vector<Point2f> vertifyPoints;
        pixel goc;
        goc.first = g.first;
        goc.second = g.second; 
        int radius = 1;
        int phantu = 0;
        //int kernel = 2*radius+1;
        while(phantu==0)
        {
            goc.first = goc.first-radius;
            goc.second = goc.second-radius;
            for(int i=0;i<=radius;i++)
            {
                float dis1 = frame.get_distance(goc.first,goc.second+i);
                float dis2 = frame.get_distance(goc.first+i,goc.second);
                if(dis1 > 0.01){vertifyPoints.push_back(Point2f(goc.first,goc.second+i));}
                if(dis2 > 0.01){vertifyPoints.push_back(Point2f(goc.first+i,goc.second));}
            }
            radius++;
            phantu = vertifyPoints.size();
        }
        for(int i=0;i<phantu;i++)
        {
            float chieu_x,chieu_y;
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
            g.first = chieu_x/phantu;
            g.second = chieu_y/phantu;
          
        }
    } 

    ///////////////// p4 //////////////////
    if(frame.get_distance(k.first,k.second)<0.00001)
    {
        vector<Point2f> vertifyPoints;
        pixel goc;
        goc.first = k.first;
        goc.second = k.second; 
        int radius = 1;
        int phantu = 0;
        //int kernel = 2*radius+1;
        while(phantu==0)
        {
            goc.first = goc.first+radius;
            goc.second = goc.second-radius;
            for(int i=0;i<=radius;i++)
            {
                float dis1 = frame.get_distance(goc.first,goc.second+i);
                float dis2 = frame.get_distance(goc.first-i,goc.second);
                if(dis1 > 0.01){vertifyPoints.push_back(Point2f(goc.first,goc.second+i));}
                if(dis2 > 0.01){vertifyPoints.push_back(Point2f(goc.first-i,goc.second));}
            }
            radius++;
            phantu = vertifyPoints.size();
        }
        for(int i=0;i<phantu;i++)
        {
            float chieu_x,chieu_y;
            chieu_x += vertifyPoints[i].x;
            chieu_y += vertifyPoints[i].y;
            k.first = chieu_x/phantu;
            k.second = chieu_y/phantu;
        }
    } 
   
}





values getCorner(Mat src)
{
    cout<<"mark 1"<<endl;
    Mat img = src.clone();

    ////////////////////////////////////////////////////////// 
    
    
    #define radius 30
    const int thresh = 3;
    const int block = 7;

    const int lowThreshold = 20;
    const int ratio = 3;
    const int kernel_size = 3;
    cout<<"mark 2"<<endl;
    Mat kernel_thr = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat dstGray = Mat::zeros(Size(src.cols, src.rows), CV_8U);
    cout<<"mark 3"<<endl;
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
    cout<<"mark 4"<<endl;
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
    cout<<"mark 5"<<endl;
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
    cout<<"mark 6"<<endl;
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
    cout<<"mark 7"<<endl;
    //---------Result: frame--------------------------------------//
    bitwise_or(frame1,frame2,frame);
    imshow("frame", frame);
    cout<<"mark 8"<<endl;
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    //-----------------getCorners---------------------------------//
    vector<Point2f> PointCorner;
    values value = {PointCorner,frame};
    ///////////////////////////////////////////////////////////////
    vector<Vec4i> lines;
    int egdeSet[40][4] = {0};
    int fourEdge[4][4] = {0};
    int fourPoint[4][2] = {0};
    float ablines[4][2] = {0.0f};

    HoughLinesP(frame, lines, 1, CV_PI/180, 14, 10, 1500 );
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
    cout<<"mark 9"<<endl;
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
            return value;
        }
    }
    cout<<"mark 10"<<endl;
    if(numberLines>0)
    {
        PointCorner.push_back(Point(0.0f,0.0f));
        cout<<"wrong in here 1"<<endl;
        return value;   
    }
    cout<<"out return"<<endl;
    ///////////////////////////////////////////////////////////////
    for (uint8_t i=0; i<4; i++)
    {
        ablines[i][0] = (float)(fourEdge[i][1]-fourEdge[i][3])/(float)(fourEdge[i][0]-fourEdge[i][2]);
        cout<<"can loop"<<endl;
        ablines[i][1] = (float)fourEdge[i][1] - ablines[i][0]*(float)fourEdge[i][0];
        cout<<"wrong in here 2"<<endl;
    }
    float temp1[1][2] = {{ablines[1][0], ablines[1][1]}};
    cout<<"wrong in here 3"<<endl;
    ablines[1][0] = ablines[2][0];
    cout<<"wrong in here 4"<<endl;
    ablines[1][1] = ablines[2][1];
    cout<<"wrong in here 5"<<endl; 
    ablines[2][0] = temp1[0][0];
    ablines[2][1] = temp1[0][1];
    cout<<"mark 11"<<endl;
    for (uint8_t i=0; i<2; i++)
    {
        float coordinatesX1 = ((ablines[2][1]-ablines[i][1])/(ablines[i][0]-ablines[2][0]));
        fourPoint[2*i][0] = (int16_t)coordinatesX1; 
        fourPoint[2*i][1] = (int16_t)(ablines[2][0]*coordinatesX1+ablines[2][1]);
        float coordinatesX2 = ((ablines[3][1]-ablines[i][1])/(ablines[i][0]-ablines[3][0]));
        fourPoint[2*i+1][0] = (int16_t)coordinatesX2;
        fourPoint[2*i+1][1] = (int16_t)(ablines[3][0]*coordinatesX2+ablines[3][1]);
    }
    cout<<"mark 13"<<endl;
    //////////////////////////////////////////////////////////////
    for (int i=0; i<4; i++)
            {
                circle( img, Point(fourPoint[i][0]+left_width,fourPoint[i][1]+up_height), 2, Scalar(0, 0, 255), -1);
            }
            //imshow("img",img);
    //////////////////////////////////////////////////////////////
    int temp2[1][2] = {{fourPoint[2][0], fourPoint[2][1]}};
    fourPoint[2][0] = fourPoint[3][0];
    fourPoint[2][1] = fourPoint[3][1];
    fourPoint[3][0] = temp2[0][0];
    fourPoint[3][1] = temp2[0][1];
    cout<<"mark 14"<<endl;
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

    cout<<"mark 15"<<endl;

    //------------The Final Results Find PointCorner-------------//
    ///////////////////////////////////////////////////////////////
    for (uint8_t i=0; i<4; i++)
    {                
        circle( img, Point(fourPoint[i][0]+left_width,fourPoint[i][1]+up_height), 2, Scalar(255, 0, 0), -1);
            
            
        fourPoint[i][0] += left_width;
        fourPoint[i][1] += up_height;
        PointCorner.push_back(Point2i(fourPoint[i][0],fourPoint[i][1]));
    }
    imshow("img",img);
    
    return value;
}