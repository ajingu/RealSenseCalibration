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

		rs2::context ctx;
		rs2::pipeline p(ctx);
		rs2::config cfg;
		rs2::frameset frames;

		namedWindow("Display", WINDOW_AUTOSIZE);

		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

		//left IR Image
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		auto pipe_profile = cfg.resolve(p);
		auto depth_sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
		
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
			if (ids.size() > 0)
				cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

			imshow("Display", image_copy);

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