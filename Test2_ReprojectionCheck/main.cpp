#pragma warning(disable: 4996)

#include <algorithm>
#include <iostream>
#include <numeric>
#include <fstream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define MARKER_SIDE 0.032

using namespace std;
using namespace cv;

int main()
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




	//get marker points
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	//vector<Point3d> object_points;
	vector<Point2f> image_points;

	map<string, Mat> images;


	for_each(begin(serial_numbers), end(serial_numbers), [&dictionary, /*&object_points,*/ &image_points, &serial_numbers, &camera_matrix_map, &dist_coeffs_map, &images](string sn)
	{
		string file_name = "../Common/Image/IR/" + sn + ".png";
		Mat image = imread(file_name);
		images[sn] = image;
		Mat image_copy;
		image.copyTo(image_copy);
		vector<int> ids;
		vector<vector<Point2f>> corners;

		aruco::detectMarkers(image, dictionary, corners, ids);
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
				if (sn == serial_numbers[1])
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

	FileStorage fs2("../Common/Correspondence/test2/Camera_Transform.xml", FileStorage::READ);
	if (!fs2.isOpened())
	{
		cerr << "File can not be opened." << endl;
		return -1;
	}

	fs2["R1"] >> camera_rvec;
	fs2["t1"] >> camera_tvec;

	fs2.release();

	vector<Point3d> object_points;
	FILE* fptr = fopen("../Common/Correspondence/test2/point3d.txt", "r");
	if (fptr == NULL)
	{
		cerr << "ERROR: unable to open file " << "\n";
		exit(1);
	};

	int num_points;
	fscanf(fptr, "%d", &num_points);

	for (int i = 0; i < num_points; i++)
	{
		Point3d point;
		fscanf(fptr, "%lf", &point.x);
		fscanf(fptr, "%lf", &point.y);
		fscanf(fptr, "%lf", &point.z);
		object_points.emplace_back(point);
	}
	fclose(fptr);


	vector<Point2d> reprojected_points;
	projectPoints(object_points, camera_rvec, camera_tvec, camera_matrix_map[serial_numbers[1]], dist_coeffs_map[serial_numbers[1]], reprojected_points);

	//visualization
	Mat reprojection_image = images[serial_numbers[1]];
	for (int i = 0; i < reprojected_points.size(); i++)
	{
		drawMarker(reprojection_image, image_points[i], Scalar(255, 0, 0), MARKER_CROSS, 10, 2);
		drawMarker(reprojection_image, reprojected_points[i], Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
	}

	putText(reprojection_image, "BLUE : Marker Points (t+1)", Point{ 10,20 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
	putText(reprojection_image, "GREEN : Reprojected Points (t -> t+1)", Point{ 10,45 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

	imshow("Reprojection", reprojection_image);


	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}