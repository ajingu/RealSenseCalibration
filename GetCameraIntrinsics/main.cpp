#include <iostream>
#include <sstream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	int width = 640;
	int height = 480;
	int fps = 30;
	string intrinsics_root_path = "../Common/Calibration/Intrinsics/";

	rs2::context ctx;

	for (auto&& dev : ctx.query_devices())
	{
		rs2::pipeline pipe(ctx);
		rs2::config cfg;
		string serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		cfg.enable_device(serial_number);
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		auto m_pipeline_profile = pipe.start(cfg);
		auto ir_left_stream = m_pipeline_profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
		auto intrinsics = ir_left_stream.get_intrinsics();
		
		cv::FileStorage fs(intrinsics_root_path + serial_number + ".xml",
			cv::FileStorage::WRITE);
		if (!fs.isOpened()) {
		    cerr << "File can not be opened." << std::endl;
			return -1;
		}

		Mat intrinsics_mat = Mat::zeros(3, 3, CV_64FC1);
		intrinsics_mat.at<double>(0, 0) = intrinsics.fx;
		intrinsics_mat.at<double>(1, 1) = intrinsics.fy;
		intrinsics_mat.at<double>(0, 2) = intrinsics.ppx;
		intrinsics_mat.at<double>(1, 2) = intrinsics.ppy;
		intrinsics_mat.at<double>(2, 2) = 1;

		fs << "intrinsics" << intrinsics_mat;

		fs.release();

		cv::FileStorage fs2(intrinsics_root_path + serial_number + ".xml", cv::FileStorage::READ);
		if (!fs2.isOpened()) {
			std::cout << "File can not be opened." << std::endl;
			return -1;
		}

		cout << "SN: " << serial_number << endl;
		Mat m;
		fs2["intrinsics"] >> m;
		cout << "intrinsics: " << endl << m  << endl;
		cout << endl;

		fs2.release();
	}

	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}