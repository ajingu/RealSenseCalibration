#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "bundle_adjustmenter.cpp"

#define MARKER_SIDE 0.032

using namespace std;
using namespace cv;
using namespace ceres;

int main(int argc, char** argv)
{
	//get intrinsics
	string serial_numbers[2] = { "819612072493", "825312072048" };
	map<string, Mat> camera_intrinsics_map;
	map<string, Mat> dist_coeffs_map;
	map<string, Mat> images;

	for_each(begin(serial_numbers), end(serial_numbers), [&camera_intrinsics_map, &dist_coeffs_map, &images](string sn)
	{
		string file_name = "../Common/Image/IR/" + sn + ".png";
		Mat image = imread(file_name);
		images[sn] = image;

		string intrinsics_file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(intrinsics_file_name, FileStorage::READ);
		if (!fs.isOpened())
		{
			cout << "File can not be opened." << endl;
			return -1;
		}

		Mat A, dist_coeffs;
		fs["intrinsics"] >> A;
		fs["distCoeffs"] >> dist_coeffs;
		camera_intrinsics_map[sn] = A;
		dist_coeffs_map[sn] = dist_coeffs;

		cout << "Serial Number: " << sn << endl;
		cout << "Intrinsics: " << endl << A << endl << endl;

		fs.release();
	});


	//Bundle Adjustment
	
	google::InitGoogleLogging(argv[0]);
	
	BALProblem bal_problem;
	if (!bal_problem.loadFile("../Common/Correspondence/test2/correspondence_test.txt"))
	{
		cerr << "ERROR: unable to open file " << "\n";
		return 1;
	}
	
	const double* const observations = bal_problem.observations();

	Problem problem;

	/*
	for (int i = 0; i < bal_problem.num_observations(); i++)
	{
		cout << "camera_transform";
		double* a = bal_problem.mutable_camera_transform_from_base_camera(i);
		for (int j = 0; j < 6; j++)
		{
			cout << " " << a[j];
		}
		cout << endl;

		cout << "base_marker";
		double* b = bal_problem.mutable_base_marker_transform_from_base_camera(i);
		for (int j = 0; j < 6; j++)
		{
			cout << " " << b[j];
		}
		cout << endl;

		cout << "marker";
		double* c = bal_problem.mutable_marker_transform_from_base_marker(i);
		for (int j = 0; j < 6; j++)
		{
			cout << " " << c[j];
		}
		cout << endl;
	}*/
	/* parameter‚Í‚¿‚á‚ñ‚Æ“Ç‚Ýž‚Ü‚ê‚Ä‚é
	cout << "parameters: ";
	for (int i = 0; i < bal_problem.num_parameters(); i++)
	{
		cout << " " << bal_problem.parameters()[i];
	}
	cout << endl;*/
	
	for (int i = 0; i < bal_problem.num_observations(); i++)
	{
		int camera_idx = bal_problem.camera_idx(i);
		
		CostFunction* cost_function =
			ReprojectionError::create(
				observations + 8 * i,
				MARKER_SIDE,
				camera_intrinsics_map[serial_numbers[camera_idx]],
				dist_coeffs_map[serial_numbers[camera_idx]]);
		
		problem.AddResidualBlock(cost_function,
			NULL,
			bal_problem.mutable_camera_transform_from_base_camera(i),
			bal_problem.mutable_base_marker_transform_from_base_camera(i),
			bal_problem.mutable_marker_transform_from_base_marker(i));
	}
	
	Solver::Options options;
	options.linear_solver_type = DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;
	
	//Reprojection Check
	double* camera_transform_ptr = bal_problem.mutable_camera_transform_from_base_camera(1);
	Mat camera_rvec = (Mat_<double>(3, 1) << camera_transform_ptr[0], camera_transform_ptr[1], camera_transform_ptr[2]);
	Mat camera_tvec = (Mat_<double>(3, 1) << camera_transform_ptr[3], camera_transform_ptr[4], camera_transform_ptr[5]);
	Mat camera_rot;
	Rodrigues(camera_rvec, camera_rot);

	cout << "R:" << endl;
	cout << camera_rot << endl;
	cout << "t:" << endl;
	cout << camera_tvec << endl;
	
	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}