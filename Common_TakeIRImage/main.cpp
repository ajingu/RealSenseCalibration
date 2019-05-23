#include <iostream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;
using namespace rs2;

int main()
{
	try
	{
		int width = 640;
		int height = 480;
		int fps = 30;

		bool should_take_image;

		context ctx;
		pipeline p(ctx);
		config cfg;
		frameset frames;


		Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);

		//left IR Image
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		
		auto pipe_profile = cfg.resolve(p);
		auto dev = pipe_profile.get_device();

		p.start(cfg);

		while (true)
		{
			frameset frames = p.wait_for_frames();

			frame ir_frame_left = frames.get_infrared_frame(1);

			Mat image_ir(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
			Mat image_ir_copy;
			image_ir.copyTo(image_ir_copy);

			vector<int> ids;
			vector<vector<Point2f>> corners;
			aruco::detectMarkers(image_ir, dictionary, corners, ids);
			if (ids.size() >= 4)
			{
				should_take_image = true;
				aruco::drawDetectedMarkers(image_ir_copy, corners, ids);
			}
			else
			{
				should_take_image = false;
			}
				

			imshow("IR", image_ir_copy);

			char key = (char)waitKey(10);
			if (key == 27) //'esc'
			{
				break;
			}
			else if (key == 115) //'s'
			{
				if (should_take_image)
				{
					string sn = string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

					string ir_image_file_name = "../Common/Image/IR/" + sn + ".png";
					imwrite(ir_image_file_name, image_ir);
					cout << "Saved IR Image Path: " << ir_image_file_name << endl;
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
		cerr << e.what() << endl;
		return EXIT_FAILURE;
	}
}