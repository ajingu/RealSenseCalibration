#include <iostream>
#include <filesystem>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::experimental::filesystem;
using namespace cv;
using namespace rs2;

void saveIntrinsics(rs2_intrinsics& intrinsics, string serial_number)
{
	FileStorage fs("../Common/Calibration/Intrinsics/" + serial_number + ".xml",
		FileStorage::WRITE);
	if (!fs.isOpened()) {
		cerr << "unable to open intrinsics file." << std::endl;
		system("PAUSE");
		exit(-1);
	}

	Mat intrinsics_mat = Mat::zeros(3, 3, CV_64FC1);
	intrinsics_mat.at<double>(0, 0) = intrinsics.fx;
	intrinsics_mat.at<double>(1, 1) = intrinsics.fy;
	intrinsics_mat.at<double>(0, 2) = intrinsics.ppx;
	intrinsics_mat.at<double>(1, 2) = intrinsics.ppy;
	intrinsics_mat.at<double>(2, 2) = 1;

	fs << "intrinsics" << intrinsics_mat;

	auto dist_coeffs = intrinsics.coeffs;
	Mat dist_coeffs_mat = Mat::zeros(5, 1, CV_64FC1);
	dist_coeffs_mat.at<double>(0, 0) = dist_coeffs[0];
	dist_coeffs_mat.at<double>(1, 0) = dist_coeffs[1];
	dist_coeffs_mat.at<double>(2, 0) = dist_coeffs[2];
	dist_coeffs_mat.at<double>(3, 0) = dist_coeffs[3];
	dist_coeffs_mat.at<double>(4, 0) = dist_coeffs[4];
	fs << "distCoeffs" << dist_coeffs_mat;

	fs.release();
}

int main()
{
	try
	{
		int width = 640;
		int height = 480;
		int fps = 30;

		int time_id = 0;
		int count_id = 0;


		context ctx;
		vector<pipeline> pipelines;
		vector<string> serial_numbers;
		vector<Mat> images;

		for (auto&& dev : ctx.query_devices())
		{
			pipeline pipe(ctx);
			config cfg;
			cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps); //left IR Image

			auto depth_sensor = dev.first<rs2::depth_sensor>();

			if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
			{
				depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
			}

			auto pipeline_profile = pipe.start(cfg);
			pipelines.emplace_back(pipe);

			string sn = string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			serial_numbers.emplace_back(sn);

			rs2_intrinsics intrinsics = pipeline_profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
			saveIntrinsics(intrinsics, sn);

			cout << count_id << " " << sn << endl;
			count_id++;
		}

		cout << endl;


		while (true)
		{
			images.clear();

			for (int pipeline_idx = 0; pipeline_idx < pipelines.size(); pipeline_idx++)
			{
				frameset frames = pipelines[pipeline_idx].wait_for_frames();

				frame ir_frame_left = frames.get_infrared_frame(1);

				Mat image_ir(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());

				images.emplace_back(image_ir);

				imshow(serial_numbers[pipeline_idx], image_ir);
			}


			char key = (char)waitKey(10);
			if (key == 27) //'esc'
			{
				break;
			}
			else if (key == 115) //'s'
			{
				for (int pipeline_idx = 0; pipeline_idx < pipelines.size(); pipeline_idx++)
				{
					//string directory_path = "../Common/Image/IR/main/" + to_string(time_id);
					string directory_path = "../Common/Image/IR/hongo/" + to_string(time_id);
					if (!exists(directory_path))
					{
						create_directory(directory_path);
					}

					string ir_image_file_name = directory_path + (string)"/" + serial_numbers[pipeline_idx] + ".png";
					imwrite(ir_image_file_name, images[pipeline_idx]);
				}

				cout << "Photos are saved(TIME ID: " << time_id << ")" << endl;

				time_id++;
			}
		}

		return EXIT_SUCCESS;
	}
	catch (const rs2::error& e)
	{
		cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
		system("PAUSE");
		return EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
		system("PAUSE");
		return EXIT_FAILURE;
	}
}