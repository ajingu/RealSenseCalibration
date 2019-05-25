#pragma warning(disable: 4996)

#include <algorithm>
#include <iostream>
#include <numeric>
#include <fstream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define MARKER_SIDE 0.015
#define TIMES 6
#define CAMERAS 4

using namespace std;
using namespace cv;

vector<Point3d> getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec)
{
	double half_side = side / 2;

	// compute rot_mat
	Mat rot_mat;
	Rodrigues(rvec, rot_mat);

	// transpose of rot_mat for easy columns extraction
	Mat rot_mat_t = rot_mat.t();

	// the two E-O and F-O vectors
	double * tmp = rot_mat_t.ptr<double>(0);
	Point3d camWorldE(tmp[0] * half_side,
		tmp[1] * half_side,
		tmp[2] * half_side);

	tmp = rot_mat_t.ptr<double>(1);
	Point3d camWorldF(tmp[0] * half_side,
		tmp[1] * half_side,
		tmp[2] * half_side);

	// convert tvec to point
	Point3d tvec_3d(tvec[0], tvec[1], tvec[2]);

	// return vector:
	vector<Point3d> ret(4, tvec_3d);

	ret[0] += -camWorldE + camWorldF; //top left
	ret[1] += camWorldE + camWorldF; //top right
	ret[2] += camWorldE - camWorldF; //bottom right
	ret[3] += -camWorldE - camWorldF; //bottom left

	return ret;
}

int main()
{
	//get intrinsics
	string serial_numbers[4] = { "821312061029", "816612062327", "821212062536", "821212061326" };
	map<string, Mat> camera_matrix_map;
	map<string, Mat> dist_coeffs_map;

	for_each(begin(serial_numbers), end(serial_numbers), [&camera_matrix_map, &dist_coeffs_map](string sn)
	{
		string file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(file_name, FileStorage::READ);
		if (!fs.isOpened())
		{
			cout << "unable to open intrinsics file." << endl;
			system("PAUSE");
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
	vector<Point2f> image_points[TIMES][CAMERAS];

	map<string, Mat> images[TIMES];

	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			string sn = serial_numbers[camera_idx];
			string file_name = "../Common/Image/IR/hongo/" + to_string(time_idx) + "/" + sn + ".png";
			Mat image = imread(file_name);
			if (image.empty())
			{
				cout << file_name << endl;
				cerr << "Image is empty." << endl;
				system("PAUSE");
				exit(-1);
			}
			images[time_idx][sn] = image;
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
					for (int j = 0; j < 4; j++)
					{
						image_points[time_idx][camera_idx].push_back(corners[indices[i]][j]);
					}
				}
			}
		}
	}

	
	
	FileStorage fs2("../Common/Correspondence/hongo/Camera_Transform.xml", FileStorage::READ);
	if (!fs2.isOpened())
	{
		cerr << "unable to open Camera_Transform.xml" << endl;
		system("PAUSE");
		exit(1);
	}

	Mat camera_rvec, camera_rot, camera_tvec;
	vector<Point3d> object_points;

	FILE* fptr = fopen("../Common/Correspondence/hongo/point3d.txt", "r");
	if (fptr == NULL)
	{
		cerr << "unable to open point3d.txt" << endl;
		system("PAUSE");
		exit(1);
	};

	int num_points_all, num_times, num_cameras;
	fscanf(fptr, "%d", &num_points_all);
	fscanf(fptr, "%d", &num_times);
	fscanf(fptr, "%d", &num_cameras);
	int** num_points_per_time_camera = new int*[num_times];
	for (int time_idx = 0; time_idx < num_times; time_idx++)
	{
		num_points_per_time_camera[time_idx] = new int[num_cameras];
	}
	for (int time_idx = 0; time_idx < num_times; time_idx++)
	{
		int tmp;
		fscanf(fptr, "%d", &tmp);

		for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++)
		{
			fscanf(fptr, "%d", &num_points_per_time_camera[time_idx][camera_idx]);
		}
	}


	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			object_points.clear();

			for (int i = 0; i < num_points_per_time_camera[time_idx][camera_idx]; i++)
			{
				Point3d point;
				fscanf(fptr, "%lf", &point.x);
				fscanf(fptr, "%lf", &point.y);
				fscanf(fptr, "%lf", &point.z);
				object_points.emplace_back(point);
			}
			
			fs2["R" + to_string(camera_idx)] >> camera_rvec;
			fs2["t" + to_string(camera_idx)] >> camera_tvec;

			vector<Point2d> reprojected_points;
			projectPoints(object_points, camera_rvec, camera_tvec, camera_matrix_map[serial_numbers[camera_idx]], dist_coeffs_map[serial_numbers[camera_idx]], reprojected_points);
			
			//visualization
			Mat reprojection_image = images[time_idx][serial_numbers[camera_idx]];
			for (int i = 0; i < reprojected_points.size(); i++)
			{
				drawMarker(reprojection_image, image_points[time_idx][camera_idx][i], Scalar(255, 0, 0), MARKER_CROSS, 10, 2);
				drawMarker(reprojection_image, reprojected_points[i], Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
			}
			
			putText(reprojection_image, "BLUE : Marker Points (t+1)", Point{ 10,20 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
			putText(reprojection_image, "GREEN : Reprojected Points (t -> t+1)", Point{ 10,45 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

			imshow("Reprojection " + serial_numbers[camera_idx] + " " + to_string(time_idx), reprojection_image);
		}
	}
	fclose(fptr);
	fs2.release();

	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}