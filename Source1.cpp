
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#include <sys/stat.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
//#include "/home/l/librealsense/examples/example.hpp"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
//include opencv library
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

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
#include <string>
#include <Windows.h>

using namespace std;
using namespace cv;


const char* window_trackbar = "Trackbar";


////////////////////////////////////////////////////////////////////

using pixel = pair<float, float>;
pixel u, v, u1, v1, z, z1;


float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v, int w, int h, int wid, int hei);
vector<Point2f> getCorner(Mat src);
vector<Point2f> checkPoint(vector<Point2f> PointCheck, const rs2::depth_frame&  frameDis);
//void checkPoint1(const rs2::depth_frame& frame, pixel u, pixel v);
//vector<Point2f> checkPoint2(vector<Point2f> PointCheck, const rs2::depth_frame& frameDis);
vector<Point2f> points;
Mat img, img1;
int m = 0;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		if (m == 0)
		{
			u1.first = x;
			u1.second = y;

			u.first = u1.first;
			u.second = u1.second;
			cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			//cout<<"u first: "<<u.first<<endl;
			//cout<<"u second: "<<u.second<<endl

		}
		if (m == 1)
		{
			v1.first = x;
			v1.second = y;

			v.first = v1.first;
			v.second = v1.second;
			cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			//cout<<"v first: "<<v.first<<endl;
			//cout<<"v second: "<<v.second<<endl;

		}
		if (m == 2)
		{
			z1.first = x;
			z1.second = y;

			z.first = z1.first;
			z.second = z1.second;
			cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			//cout<<"v first: "<<v.first<<endl;
			//cout<<"v second: "<<v.second<<endl;

		}
		m++;
	}

}

bool FileExists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

string filename1 = "C:/Users/daoth/source/repos/Project5/control/start/start";
string filename2 = "C:/Users/daoth/source/repos/Project5/control/close/close";
string filename3 = "C:/Users/daoth/source/repos/Project6/Project6/number.txt";
string filename4 = filename1;
string filename5 = filename2;
string filename6 = "C:/Users/daoth/source/repos/Project5/control/state/state";
string filename7 = filename6;
string filename8 = "C:/Users/daoth/source/repos/Project5/control/manual/manual";
string filename9 = "C:/Users/daoth/source/repos/Project5/control/auto/auto";
string filename10 = filename8;
string filename11 = filename9;


int stt2;
int main(int argc, char * argv[]) try
{


	int n = 0;
	int n1 = 0;
	int n2 = 0;
	int n3 = 0;
	string stt1;

	ifstream file2("number.txt");
	if (file2.is_open())
	{
		getline(file2, stt1);
		file2.close();
		stt2 = stoi(stt1);
	}

	const char x = '/';
	const char y = '\\';
	replace(filename4.begin(), filename4.end(), x, y);
	replace(filename5.begin(), filename5.end(), x, y);
	replace(filename7.begin(), filename7.end(), x, y);
	replace(filename10.begin(), filename10.end(), x, y);
	replace(filename11.begin(), filename11.end(), x, y);
	for (int i = 0; i < (stt2 + 1); i++)
	{
		string remove_path1 = filename4 + to_string(i) + ".txt";
		string remove_path2 = filename5 + to_string(i) + ".txt";
		string remove_path3 = filename7 + to_string(i) + ".txt";
		string remove_path4 = filename10 + to_string(i) + ".txt";
		string remove_path5 = filename11 + to_string(i) + ".txt";

		const char * str1 = remove_path1.c_str();
		const char * str2 = remove_path2.c_str();
		const char * str3 = remove_path3.c_str();
		const char * str4 = remove_path4.c_str();
		const char * str5 = remove_path5.c_str();

		remove(str1);
		remove(str2);
		remove(str3);
		remove(str4);
		remove(str5);
	}


	//window app(640,360,"IMG");

	//rs2::colorizer color_map;
	rs2::hole_filling_filter hole_filter;
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

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
	rs2::align align_to(RS2_STREAM_COLOR);

	rs2::pipeline p;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);



	auto profile = p.start(cfg);
	auto sensor = profile.get_device().first<rs2::depth_sensor>();

	//set the device to high accuraccy preset of the d400
	if (sensor && sensor.is<rs2::depth_stereo_sensor>())
	{
		sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_CUSTOM);
	}

	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	rs2::frame_queue postprocessed_frames;

	std::atomic_bool alive{ true };
	int w;
	int h;
	std::thread video_processing_thread([&]() {
		while (alive)
		{
			// Fetch frames from the pipeline and send them for processing
			rs2::frameset data ;
			if (p.poll_for_frames(&data))
			{
				//data1 = data.apply_filter(color_map);

				// First make the frames spatially aligned
				


				// Decimation will reduce the resultion of the depth image,
				// closing small holes and speeding-up the algorithm
				data = data.apply_filter(dec);

				// To make sure far-away objects are filtered y
				// we try to switch to disparity domain
				data = data.apply_filter(depth2disparity);

				

				// Apply spatial filtering
				//data = data.apply_filter(spat);

				// Apply temporal filtering
				data = data.apply_filter(temp);

				// If we are in disparity domain, switch back to depth
				data = data.apply_filter(disparity2depth);
				data = data.apply_filter(align_to);
				//Apply hole filtering
				data = data.apply_filter(hole_filter);

				//// Apply color map for visualization of depth
				data = data.apply_filter(color_map);
				// Send resulting frames for visualization in the main thread
				auto fr = data.get_color_frame();
				//auto colorized_depth = data.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);
				w = fr.as<rs2::video_frame>().get_width();
				h = fr.as<rs2::video_frame>().get_height();
				//rs2::frame fr1 = data1.get_color_frame();
				Mat image(Size(w, h), CV_8UC3, (void*)fr.get_data(), Mat::AUTO_STEP);
				//Mat image1(Size(w, h), CV_8UC3, (void*)fr1.get_data(), Mat::AUTO_STEP);
				img = image;

				//img1 = image1;
				circle(image, Point(u1.first, u1.second), 2, Scalar(255, 0, 0), CV_FILLED, 3, 0);
				circle(image, Point(v1.first, v1.second), 2, Scalar(0, 255, 0), CV_FILLED, 3, 0);
				circle(image, Point(z1.first, z1.second), 2, Scalar(0, 0, 255), CV_FILLED, 3, 0);
				//imshow(window_name, img);
				postprocessed_frames.enqueue(data);
			}
		}
	});


	rs2::frameset current_frameset;
	while (true)
	{

		string start_path = filename1 + to_string(n) + ".txt";
		//string close_path = filename2 + to_string(n) + ".txt";


		while (FileExists(start_path) == 0)
		{
			cout << "stop!!!" << endl;
			Sleep(5);
		}

		while (FileExists(start_path) == 1)
		{


			string close_path = filename2 + to_string(n) + ".txt";
			postprocessed_frames.poll_for_frame(&current_frameset);
			//int s = 0;
			//cout << "new frame" << endl;
			if (current_frameset)
			{

				//////////////////////////////// MANUAL MODE ///////////////////////////////////
				string manual_path = filename8 + to_string(n2) + ".txt";
				string auto_path = filename9 + to_string(n3) + ".txt";
				cout << "start!!!" << endl;

				while (FileExists(manual_path) == 1)
				{

					const auto window_name = "img";
					namedWindow(window_name, WINDOW_AUTOSIZE);

					setMouseCallback(window_name, CallBackFunc, NULL);
					if (m == 3) { m = 0; }
					imshow(window_name, img);

					auto depth = current_frameset.get_depth_frame();
					int wid = depth.get_width();
					int hei = depth.get_height();

					string state_path = filename6 + to_string(n1) + ".txt";

					while (FileExists(state_path) == 1)
					{
						Sleep(700);
						//cout << "Sleep" << endl;
						n1++;
						state_path = filename6 + to_string(n1) + ".txt";

					}
					//cout << "w: " << w << endl;
					//cout << "h: " << h << endl;
					//cout << "wid: " << wid << endl;
					//cout << "hei: " << hei << endl;

					float air_dist1 = dist_3d(depth, u, v, w, h, wid, hei);
					float air_dist2 = dist_3d(depth, v, z, w, h, wid, hei);

					cout << "distance between 2 point u and v is: " << air_dist1 << endl;
					cout << "distance between 2 point v and z is: " << air_dist2 << endl;
					//float air_dist3 = air_dist1 * 100;
					//float air_dist4 = air_dist2 * 100;
					string dist1 = to_string((air_dist1)*100);
					string dist2 = to_string((air_dist2)*100);
					string a = "C:\\Users\\daoth\\source\\repos\\Project6\\Project6\\distance1.txt";
					ofstream myfile1(a);
					if (myfile1.is_open())
					{
						myfile1 << dist1;
						//cout << "ghi xong" << endl;
						myfile1.close();
					}


					ofstream myfile2("C:\\Users\\daoth\\source\\repos\\Project6\\Project6\\distance2.txt");
					if (myfile2.is_open())
					{
						myfile2 << dist2;
						myfile2.close();
					}


					//s++;

					pixel u1, v1;
					if (FileExists(close_path) == 1)
					{
						n++;
						string number;
						if (n < n1) { number = to_string(n1); }
						else { number = to_string(n); }
						ofstream myfile1("number.txt");
						if (myfile1.is_open())
						{
							myfile1 << number;
							myfile1.close();
						}
						destroyAllWindows();
						n2++;



						goto loop;

					}
					char c = (char)waitKey(1);
					if (c == 27)
						break;

					Sleep(5);
				}

				///////////////////////////// AUTO MODE /////////////////////////////////////

				while (FileExists(auto_path) == 1)
				{

					auto depth = current_frameset.get_depth_frame();
					int wid = depth.get_width();
					int hei = depth.get_height();
					
					/////////////////// add detect code for corner detection in here /////////////////

								


					///////////////////////////////////////////////////////////////////////////////////
					float air_dist1 = dist_3d(depth, u, v, w, h, wid, hei);
					float air_dist2 = dist_3d(depth, v, z, w, h, wid, hei);

					cout << "distance between 2 point u and v is: " << air_dist1 << endl;
					cout << "distance between 2 point v and z is: " << air_dist2 << endl;
					//float air_dist3 = air_dist1 * 100;
					//float air_dist4 = air_dist2 * 100;
					string dist1 = to_string((air_dist1) * 100);
					string dist2 = to_string((air_dist2) * 100);
					string a = "C:\\Users\\daoth\\source\\repos\\Project6\\Project6\\distance1.txt";
					ofstream myfile1(a);
					if (myfile1.is_open())
					{
						myfile1 << dist1;
						//cout << "ghi xong" << endl;
						myfile1.close();
					}


					ofstream myfile2("C:\\Users\\daoth\\source\\repos\\Project6\\Project6\\distance2.txt");
					if (myfile2.is_open())
					{
						myfile2 << dist2;
						myfile2.close();
					}


					//s++;

					pixel u1, v1;


					Sleep(5);
					if (FileExists(close_path) == 1)
					{
						n++;
						string number;
						if (n < n1) { number = to_string(n1); }
						else { number = to_string(n); }
						ofstream myfile1("number.txt");
						if (myfile1.is_open())
						{
							myfile1 << number;
							myfile1.close();
						}
						//destroyAllWindows();
						n3++;
						break;

					}

				}
			loop:
				Sleep(5);
				///////////////////////////////////////////////////////////////////

			}

		}


	}

	alive = false;
	video_processing_thread.join();

	return EXIT_SUCCESS;
	destroyAllWindows();
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

float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v, int w, int h, int wid, int hei)
{
	float upixel[2];
	float upoint[3];

	float vpixel[2];
	float vpoint[3];

	upixel[0] = (u.first/w)*wid;
	upixel[1] = (u.second/h)*hei;
	vpixel[0] = (v.first/w)*wid;
	vpixel[1] = (v.second/h)*hei;

	auto udist = frame.get_distance(upixel[0], upixel[1]);
	auto vdist = frame.get_distance(vpixel[0], vpixel[1]);

	rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
	rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdist);

	float distance = sqrt(pow(upoint[0] - vpoint[0], 2) +
		pow(upoint[1] - vpoint[1], 2) +
		pow(upoint[2] - vpoint[2], 2));

	return distance;
}
