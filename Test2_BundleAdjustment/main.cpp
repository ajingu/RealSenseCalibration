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

	int num_base_camera_observation = bal_problem.num_base_camera_observations();
	//cout << "NUM_BASE: " << num_base_camera_observation << endl;
	for (int i = num_base_camera_observation; i < bal_problem.num_observations(); i++)
	{
		int camera_idx = bal_problem.camera_idx(i);
		
		CostFunction* target_camera_cost_function =
			TargetCameraReprojectionError::create(
				observations + 8 * i,
				MARKER_SIDE,
				camera_intrinsics_map[serial_numbers[camera_idx]],
				dist_coeffs_map[serial_numbers[camera_idx]]);

		
		
		problem.AddResidualBlock(target_camera_cost_function,
			NULL,
			bal_problem.mutable_camera_transform_from_base_camera(i),
			bal_problem.mutable_base_marker_transform_from_base_camera(i),
			bal_problem.mutable_marker_transform_from_base_marker(i));
		
		for (int j = 0; j < num_base_camera_observation; j++)
		{
			CostFunction* base_camera_cost_function =
				BaseCameraReprojectionError::create(
					observations + 8 * j,
					MARKER_SIDE,
					camera_intrinsics_map[serial_numbers[0]],
					dist_coeffs_map[serial_numbers[0]]);

			problem.AddResidualBlock(base_camera_cost_function,
				NULL,
				bal_problem.mutable_base_marker_transform_from_base_camera(j),
				bal_problem.mutable_marker_transform_from_base_marker(j));
		}
	}
	
	Solver::Options options;
	options.linear_solver_type = DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;
	
	//Reprojection Check
	FileStorage fs("../Common/Correspondence/test2/Camera_Transform.xml", FileStorage::WRITE);
	if (!fs.isOpened()) {
		cerr << "File can not be opened." << endl;
		return -1;
	}

	for (int i = 0; i < 2; i++)
	{
		double* camera_transform_ptr = bal_problem.mutable_camera_transform_from_base_camera(i*4);
		Mat camera_rvec = (Mat_<double>(3, 1) << camera_transform_ptr[0], camera_transform_ptr[1], camera_transform_ptr[2]);
		Mat camera_tvec = (Mat_<double>(3, 1) << camera_transform_ptr[3], camera_transform_ptr[4], camera_transform_ptr[5]);
		Mat camera_rot;
		Rodrigues(camera_rvec, camera_rot);

		cout << "Camera " << i << endl;
		cout << "R:" << endl;
		cout << camera_rot.t() << endl;
		cout << "t:" << endl;
		cout << -camera_rot.t() * camera_tvec << endl;

		fs << "R" + to_string(i) << camera_rvec;
		fs << "t" + to_string(i) << camera_tvec;
	}

	fs.release();

	vector<Point3d> object_points;
	bal_problem.getPoint3dCoordinates(object_points);
	
	ofstream fout;
	fout.open("../Common/Correspondence/test2/point3d.txt");
	fout << object_points.size() << endl;
	for (int i = 0; i < object_points.size(); i++)
	{
		Point3d point = object_points[i];
		fout << point.x << " " << point.y << " " << point.z << endl;
	}
	fout.close();

	
	
	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}