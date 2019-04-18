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
		rs2::pipeline p;
		rs2::config cfg;
		rs2::frameset frames;

		namedWindow("Display", WINDOW_AUTOSIZE);

		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

		cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
		p.start(cfg);

		while (true)
		{
			rs2::frameset frames = p.wait_for_frames();

			rs2::frame color_frame = frames.get_color_frame();
			
			Mat image(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
			Mat image_copy;
			image.copyTo(image_copy);

			vector<int> ids;
			vector<vector<Point2f> > corners;
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