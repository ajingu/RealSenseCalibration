#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "bundle_adjustmenter.cpp"


using namespace std;
using namespace cv;
using namespace ceres;


int main(int argc, char** argv)
{
	//get intrinsics
	string serial_numbers[2] = { "819612072493", "825312072048" };
	map<string, Mat> camera_matrix_map;
	map<string, Mat> dist_coeffs_map;
	map<string, Mat> images;

	for_each(begin(serial_numbers), end(serial_numbers), [&camera_matrix_map, &dist_coeffs_map, &images](string sn)
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
		camera_matrix_map[sn] = A;
		dist_coeffs_map[sn] = dist_coeffs;

		cout << "Serial Number: " << sn << endl;
		cout << "Intrinsics: " << endl << A << endl << endl;

		fs.release();
	});

	
	//Bundle Adjustment
	google::InitGoogleLogging(argv[0]);

	BALProblem bal_problem;
	if (!bal_problem.LoadFile("../Common/Correspondence/two_cam_data.txt")) 
	{
		cerr << "ERROR: unable to open file " << "\n";
		return 1;
	}
	
	const double* observations = bal_problem.observations();
	
	Problem problem;

	for (int i = 0; i < bal_problem.num_observations(); i++)
	{
		CostFunction* cost_function =
			ReprojectionError::Create(
				observations[2 * i + 0],
				observations[2 * i + 1],
				camera_matrix_map[serial_numbers[1]],
				dist_coeffs_map[serial_numbers[1]]);

		problem.AddResidualBlock(cost_function,
			NULL,
			bal_problem.mutable_camera_for_observation(i),
			bal_problem.mutable_point_for_observation(i));
	}
	
	Solver::Options options;
	options.linear_solver_type = DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;
	

	vector<Point2d> image_points(bal_problem.num_observations()),reprojected_points;
	vector<Point3d> object_points(bal_problem.num_observations());
	for (int i = 0; i < bal_problem.num_observations(); i++)
	{
		Point2d image_point(observations[2 * i + 0], observations[2 * i + 1]);
		image_points.emplace_back(image_point);

		double* object_point_ptr = bal_problem.mutable_point_for_observation(i);
		Point3d object_point(object_point_ptr[0], object_point_ptr[1], object_point_ptr[2]);
		object_points.emplace_back(object_point);
	}

	double* camera_vec_ptr = bal_problem.mutable_cameras();
	Mat camera_rvec = (Mat_<double>(3, 1) << camera_vec_ptr[0], camera_vec_ptr[1], camera_vec_ptr[2]);
	Mat camera_tvec = (Mat_<double>(3, 1) << camera_vec_ptr[3], camera_vec_ptr[4], camera_vec_ptr[5]);
	
	projectPoints(object_points, camera_rvec, camera_tvec, camera_matrix_map[serial_numbers[1]], dist_coeffs_map[serial_numbers[1]], reprojected_points);

	Mat camera_rot;
	Rodrigues(camera_rvec, camera_rot);
	camera_rot = camera_rot.t();
	camera_tvec = -camera_rot * camera_tvec;
	cout << "R: " << endl << camera_rot << endl;
	cout << "t: " << endl << camera_tvec << endl;

	
	Mat reprojection_image = images[serial_numbers[1]];
	for (int i = 0; i < reprojected_points.size(); i++)
	{
		drawMarker(reprojection_image, image_points[i], Scalar(255, 0, 0), MARKER_CROSS, 10, 2);
		drawMarker(reprojection_image, reprojected_points[i], Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
	}

	putText(reprojection_image, "BLUE : Marker Points (t+1)", cv::Point{ 10,20 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
	putText(reprojection_image, "GREEN : Reprojected Points (t -> t+1)", cv::Point{ 10,45 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

	imshow("Reprojection", reprojection_image);
	
	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}