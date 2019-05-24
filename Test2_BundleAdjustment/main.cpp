#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "bundle_adjustmenter.cpp"

#define MARKER_SIDE 0.048

using namespace std;
using namespace cv;
using namespace ceres;

int main(int argc, char** argv)
{
	//get intrinsics
	string serial_numbers[4] = { "821312061029", "816612062327", "821212062536", "821212061326" };
	map<string, Mat> camera_intrinsics_map;
	map<string, Mat> dist_coeffs_map;
	map<string, Mat> images;

	for_each(begin(serial_numbers), end(serial_numbers), [&camera_intrinsics_map, &dist_coeffs_map, &images](string sn)
	{
		string file_name = "../Common/Image/IR/" + sn + ".png";
		Mat image = imread(file_name);
		if (image.empty())
		{
			cout << file_name << endl;
			cerr << "Image is empty." << endl;
			system("PAUSE");
			exit(-1);
		}
		images[sn] = image;

		string intrinsics_file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(intrinsics_file_name, FileStorage::READ);
		if (!fs.isOpened())
		{
			cout << "unable to open intrinsics file" << endl;
			system("PAUSE");
			exit(1);
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
		cerr << "unable to open correspondence file " << "\n";
		system("PAUSE");
		exit(1);
	}
	
	const double* const observations = bal_problem.observations();
	Problem problem;

	for (int i = 0; i < bal_problem.num_observations(); i++)
	{
		int camera_idx = bal_problem.camera_idx(i);
		
		if (camera_idx == 0)
		{
			CostFunction* base_camera_cost_function =
				BaseCameraReprojectionError::create(
					observations + 8 * i,
					MARKER_SIDE,
					camera_intrinsics_map[serial_numbers[0]],
					dist_coeffs_map[serial_numbers[0]]);

			problem.AddResidualBlock(base_camera_cost_function,
				NULL,
				bal_problem.mutable_base_marker_transform_from_base_camera(i),
				bal_problem.mutable_marker_transform_from_base_marker(i));
		}
		else
		{
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
		cerr << "unable to open Camera_Transform.xml" << endl;
		system("PAUSE");
		exit(1);
	}

	for (int i = 0; i < bal_problem.num_cameras(); i++)
	{
		double* camera_transform_ptr = bal_problem.camera_parameters(i);
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
	fout << object_points.size() << " " << bal_problem.num_times() << " " << bal_problem.num_cameras() << endl;
	for (int time_idx = 0; time_idx < bal_problem.num_times(); time_idx++)
	{
		fout << time_idx;
		for (int camera_idx = 0; camera_idx < bal_problem.num_cameras(); camera_idx++)
		{
			fout << " " << bal_problem.num_observations_per_time_camera(time_idx, camera_idx);
		}
		fout << endl;
	}
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