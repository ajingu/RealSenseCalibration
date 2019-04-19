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

		bool should_take_image;

		rs2::context ctx;
		rs2::pipeline p(ctx);
		rs2::config cfg;
		rs2::frameset frames;

		namedWindow("Display", WINDOW_AUTOSIZE);

		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);

		//left IR Image
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		auto pipe_profile = cfg.resolve(p);
		auto dev = pipe_profile.get_device();
		auto depth_sensor = dev.first<rs2::depth_sensor>();
		
		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
		}

		p.start(cfg);

		while (true)
		{
			rs2::frameset frames = p.wait_for_frames();

			rs2::frame ir_frame_left = frames.get_infrared_frame(1);

			Mat image(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
			Mat image_copy;
			image.copyTo(image_copy);

			vector<int> ids;
			vector<vector<Point2f>> corners;
			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			if (ids.size() >= 2)
			{
				should_take_image = true;
				cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
			}
			else
			{
				should_take_image = false;
			}
				

			imshow("Display", image_copy);

			char key = (char)cv::waitKey(10);
			if (key == 27) //'esc'
			{
				break;
			}
			else if (key == 115) //'s'
			{
				if (should_take_image)
				{
					string file_name = "../Common/Image/Camera/" + string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) + ".png";
					imwrite(file_name, image);
					cout << "Saved File Path: " << file_name << endl;
				}
			}
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