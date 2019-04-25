#include <iostream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

int main()
{
	try
	{
		int width = 640;
		int height = 480;
		int fps = 30;

		rs2::pipeline p;
		rs2::config cfg;
		rs2::frameset frames;

		//namedWindow("Display", WINDOW_AUTOSIZE);

		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

		//left IR Image
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
		/*
		auto pipe_profile = cfg.resolve(p);
		auto dev = pipe_profile.get_device();
		auto depth_sensor = dev.first<rs2::depth_sensor>();

		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			//depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
		}*/

		p.start(cfg);

		while (true)
		{
			rs2::frameset frames = p.wait_for_frames();

			rs2::frame ir_frame_left = frames.get_infrared_frame(1);
			rs2::depth_frame depth_frame = frames.get_depth_frame();
			
			Mat image_ir(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
			Mat image_ir_copy;
			image_ir.copyTo(image_ir_copy);

			vector<int> ids;
			vector<vector<Point2f> > corners;
			cv::aruco::detectMarkers(image_ir, dictionary, corners, ids);
			if (ids.size() > 0)
				cv::aruco::drawDetectedMarkers(image_ir_copy, corners, ids);

			Mat image_depth(Size(width, height), CV_16UC1, (void*)depth_frame.get_data());
			image_depth.convertTo(image_depth, CV_8U, 255 / 10000.0, 0.0);

			imshow("IR", image_ir_copy);
			imshow("Depth", image_depth);

			char key = (char)cv::waitKey(10);
			if (key == 27)
				break;
		}

		return EXIT_SUCCESS;
	}
	catch (const rs2::error& e)
	{
		cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}