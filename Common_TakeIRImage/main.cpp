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


		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);

		//left IR Image
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
		
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
			rs2::depth_frame depth_frame = frames.get_depth_frame();

			Mat image_ir(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
			Mat image_ir_copy;
			image_ir.copyTo(image_ir_copy);

			Mat image_depth(Size(width, height), CV_16UC1, (void*)depth_frame.get_data());
			Mat image_depth_copy;
			image_depth.convertTo(image_depth_copy, CV_8U, 255 / 10000.0, 0.0);

			vector<int> ids;
			vector<vector<Point2f>> corners;
			cv::aruco::detectMarkers(image_ir, dictionary, corners, ids);
			if (ids.size() >= 4)
			{
				should_take_image = true;
				cv::aruco::drawDetectedMarkers(image_ir_copy, corners, ids);
			}
			else
			{
				should_take_image = false;
			}
				

			imshow("IR", image_ir_copy);
			imshow("Depth", image_depth_copy);

			char key = (char)cv::waitKey(10);
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

					string depth_image_file_name = "../Common/Image/Depth/" + sn + ".png";
					imwrite(depth_image_file_name, image_depth_copy);
					cout << "Saved Depth Image Path: " << depth_image_file_name << endl;

					string depth_matrix_file_name = "../Common/DepthMatrix/" + sn + ".xml";
					cv::FileStorage fs(depth_matrix_file_name,cv::FileStorage::WRITE);
					if (!fs.isOpened()) 
					{
						std::cout << "File can not be opened." << std::endl;
						return -1;
					}
					fs << "DepthMatrix" << image_depth;
					cout << "Saved Depth Matrix Path: " << depth_matrix_file_name << endl;
					fs.release();
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