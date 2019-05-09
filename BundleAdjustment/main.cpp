#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "bundle_adjustmenter.cpp"


#define MARKER_SIDE 0.032

using namespace std;
using namespace cv;
using namespace ceres;


vector<Point3f> getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec)
{
	double half_side = side / 2;


	// compute rot_mat
	Mat rot_mat;
	Rodrigues(rvec, rot_mat);

	// transpose of rot_mat for easy columns extraction
	Mat rot_mat_t = rot_mat.t();

	// the two E-O and F-O vectors
	double * tmp = rot_mat_t.ptr<double>(0);
	Point3f camWorldE(tmp[0] * half_side,
		tmp[1] * half_side,
		tmp[2] * half_side);

	tmp = rot_mat_t.ptr<double>(1);
	Point3f camWorldF(tmp[0] * half_side,
		tmp[1] * half_side,
		tmp[2] * half_side);

	// convert tvec to point
	Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

	// return vector:
	vector<Point3f> ret(4, tvec_3f);

	ret[0] += -camWorldE + camWorldF; //top left
	ret[1] += camWorldE + camWorldF; //top right
	ret[2] += camWorldE - camWorldF; //bottom right
	ret[3] += -camWorldE - camWorldF; //bottom left

	return ret;
}

int main(int argc, char** argv)
{
	//get intrinsics
	string serial_numbers[2] = { "819612072493", "825312072048" };
	map<string, Mat> camera_matrix_map;
	map<string, Mat> dist_coeffs_map;

	for_each(begin(serial_numbers), end(serial_numbers), [&camera_matrix_map, &dist_coeffs_map](string sn)
	{
		string file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(file_name, FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cout << "File can not be opened." << std::endl;
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


	//get marker points
	Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	vector<Point3f> object_points;
	vector<Point2f> image_points;

	map<string, Mat> images;


	for_each(begin(serial_numbers), end(serial_numbers), [&dictionary, &object_points, &image_points, &serial_numbers, &camera_matrix_map, &dist_coeffs_map, &images](string sn)
	{
		string file_name = "../Common/Image/IR/" + sn + ".png";
		Mat image = imread(file_name);
		images[sn] = image;
		Mat image_copy;
		image.copyTo(image_copy);
		vector<int> ids;
		vector<vector<Point2f>> corners;

		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		if (ids.size() > 0)
		{
			aruco::drawDetectedMarkers(image_copy, corners, ids);

			Mat camera_matrix = camera_matrix_map[sn];
			Mat dist_coeffs = dist_coeffs_map[sn];
			vector<Vec3d> rvecs, tvecs;
			aruco::estimatePoseSingleMarkers(
				corners, MARKER_SIDE,
				camera_matrix, dist_coeffs,
				rvecs, tvecs);


			for (int i = 0; i < ids.size(); i++)
			{
				aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.01);
			}
			imshow(sn, image_copy);

			//sort ids
			vector<int> indices(ids.size());
			iota(indices.begin(), indices.end(), 0);

			sort(indices.begin(), indices.end(), [&ids](int i1, int i2)
			{
				return ids[i1] < ids[i2];
			});


			for (int i = 0; i < ids.size(); i++)
			{
				if (sn == serial_numbers[0])
				{
					vector<Point3f> corners3D = getCornersInCameraWorld(MARKER_SIDE, rvecs[indices[i]], tvecs[indices[i]]);
					object_points.insert(object_points.end(), corners3D.begin(), corners3D.end());
				}
				else if (sn == serial_numbers[1])
				{
					for (int j = 0; j < 4; j++)
					{
						image_points.push_back(corners[indices[i]][j]);
					}
				}
			}
		}
	});

	Mat camera_rvec, camera_rot, camera_tvec;

	solvePnP(
		object_points, image_points,
		camera_matrix_map[serial_numbers[1]], dist_coeffs_map[serial_numbers[1]],
		camera_rvec, camera_tvec);

	vector<Point2f> reprojected_points;
	projectPoints(object_points, camera_rvec, camera_tvec, camera_matrix_map[serial_numbers[1]], dist_coeffs_map[serial_numbers[1]], reprojected_points);

	Rodrigues(camera_rvec, camera_rot);

	//camera(t+1) transform on camera(t) transform
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

	//Bundle Adjustment
	google::InitGoogleLogging(argv[0]);

	Problem problem;
	for (int i = 0; i < image_points.size(); i++)
	{
		//add residual blocks
	}

	Solver::Options options;
	options.linear_solver_type = DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;

	system("PAUSE");

	return 0;
}