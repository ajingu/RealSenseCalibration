#pragma warning(disable: 4996)

#include <iostream>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

#define MARKER_SIDE 0.032
#define TIMES 1
#define CAMERAS 2
#define MARKERS 4
#define BASE_MARKER_ID 2

struct Transform 
{
	Vec3d rvec, tvec;
};

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

void getIntrinsics(string* serial_numbers, map<string, Mat>& camera_intrinsics_map, map<string, Mat>& dist_coeffs_map)
{
	for(int i=0; i<CAMERAS; i++)
	{
		string sn = serial_numbers[i];
		string file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(file_name, FileStorage::READ);
		if (!fs.isOpened())
		{
			cerr << "File can not be opened." << endl;
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
	}
}

void getMarkerGeometry(const char* file_path, map<int, Transform>& marker_transforms)
{
	FILE* fptr = fopen(file_path, "r");
	if (fptr == NULL)
	{
		cerr << "ERROR: unable to open file " << "\n";
		exit(1);
	};

	int num_markers;
	fscanf(fptr, "%d", &num_markers);

	int marker_id;
	double rvec_x, rvec_y, rvec_z, tvec_x,tvec_y, tvec_z;
	for (int i = 0; i < num_markers; i++)
	{
		Transform transform;

		fscanf(fptr, "%d", &marker_id);
		fscanf(fptr, "%lf", &rvec_x);
		fscanf(fptr, "%lf", &rvec_y);
		fscanf(fptr, "%lf", &rvec_z);
		Vec3d rvec(rvec_x, rvec_y, rvec_z);
		transform.rvec = rvec;
		
		fscanf(fptr, "%lf", &tvec_x);
		fscanf(fptr, "%lf", &tvec_y);
		fscanf(fptr, "%lf", &tvec_z);
		Vec3d tvec(tvec_x, tvec_y, tvec_z);
		transform.tvec = tvec;

		marker_transforms[marker_id] = transform;
	}

	fclose(fptr);
}

int main()
{
	string serial_numbers[CAMERAS] = { "819612072493", "825312072048" };
	int marker_ids[MARKERS] = { 2, 4, 8, 23 };
	
	//intrinsics
	map<string, Mat> camera_intrinsics_map;
	map<string, Mat> dist_coeffs_map;
	getIntrinsics(serial_numbers, camera_intrinsics_map, dist_coeffs_map);

	//geometry
	const char* geometry_file_path = "../Common/Correspondence/test2/geometry_test.txt";
	map<int, Transform> marker_transforms_from_base;
	getMarkerGeometry(geometry_file_path, marker_transforms_from_base);

	//get marker points
	Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	map<string, Mat> images[TIMES];
	map<int, Transform> marker_transforms_from_camera;
	Vec3d base_rvec, base_tvec;
	vector<Point3d> object_points[CAMERAS];
	vector<Point2f> image_points[CAMERAS];

	for (int time_id = 0; time_id < TIMES; time_id++)
	{
		for (int sn_id = 0; sn_id < CAMERAS; sn_id++)
		{
			string sn = serial_numbers[sn_id];

			string file_name = "../Common/Image/IR/" + time_id + (string)"/" + sn + ".png";
			Mat image = imread(file_name);
			images[time_id][sn] = image;
			vector<int> ids;
			vector<vector<Point2f>> corners;

			aruco::detectMarkers(image, dictionary, corners, ids);
			if (ids.size() == 0) continue;
			
			Mat camera_matrix = camera_intrinsics_map[sn];
			Mat dist_coeffs = dist_coeffs_map[sn];
			vector<Vec3d> rvecs, tvecs;
			aruco::estimatePoseSingleMarkers( corners, MARKER_SIDE, camera_matrix, dist_coeffs, rvecs, tvecs);
			Mat image_copy;
			image.copyTo(image_copy);
			aruco::drawDetectedMarkers(image_copy, corners, ids);
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

			//Define Marker Transform from Camera 
			if (sn_id == 0)
			{
				int tmp_marker_id = ids[indices[0]];
				Vec3d tmp_rvec_from_camera = rvecs[indices[0]];
				Vec3d tmp_tvec_from_camera = tvecs[indices[0]];

				if (tmp_marker_id == BASE_MARKER_ID)
				{
					base_rvec = tmp_rvec_from_camera;
					base_tvec = tmp_tvec_from_camera;
				}
				else
				{
					Transform tmp_marker_transform_from_base = marker_transforms_from_base[tmp_marker_id];
					Vec3d tmp_rvec_from_base = tmp_marker_transform_from_base.rvec;
					Vec3d tmp_tvec_from_base = tmp_marker_transform_from_base.tvec;

					Mat tmp_rot_from_camera,tmp_rot_from_base, tmp_t_mat;
					Rodrigues(tmp_rvec_from_camera, tmp_rot_from_camera);
					Rodrigues(tmp_rvec_from_base, tmp_rot_from_base);

					Rodrigues(tmp_rot_from_base.t()*tmp_rot_from_camera, base_rvec);
					tmp_t_mat = tmp_rot_from_base.t()*Mat(tmp_tvec_from_camera) - tmp_tvec_from_base;
					base_tvec = tmp_t_mat;
				}
				
				//Define All Marker Transform
				marker_transforms_from_camera[marker_ids[0]] = Transform{ base_rvec, base_tvec };
				for (int i = 1; i < MARKERS; i++)
				{
					Transform marker_transform_from_base = marker_transforms_from_base[marker_ids[i]];
					Vec3d rvec_from_base = marker_transform_from_base.rvec;
					Vec3d tvec_from_base = marker_transform_from_base.tvec;

					Mat rot_from_base, base_rot_from_camera, t_mat;
				    Vec3d rvec_from_camera, tvec_from_camera;
					Rodrigues(rvec_from_base, rot_from_base);
					Rodrigues(base_rvec, base_rot_from_camera);

					Rodrigues(rot_from_base*base_rot_from_camera, rvec_from_camera);
					t_mat = rot_from_base * Mat(base_tvec) + Mat(tvec_from_base);
					tvec_from_camera = t_mat;
					marker_transforms_from_camera[marker_ids[i]] = Transform{ rvec_from_camera, tvec_from_camera };
				}
			}
			
			for (int i = 0; i < ids.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					image_points[sn_id].push_back(corners[indices[i]][j]);
				}
				cout << "Time: " << time_id << ", SN_ID: " << sn_id << ", MARKER_ID: " << ids[indices[i]] << endl;
				Transform marker_transform = marker_transforms_from_camera[ids[indices[i]]];
				vector<Point3d> corners3D = getCornersInCameraWorld(MARKER_SIDE, marker_transform.rvec, marker_transform.tvec);
				object_points[sn_id].insert(object_points[sn_id].end(), corners3D.begin(), corners3D.end());
			}
		}
	}

	//Calculate Default Camera Transform from Base Camera
	vector<Vec3d> camera_rvecs(CAMERAS), camera_tvecs(CAMERAS);
	Rodrigues(Mat::eye(3, 3, CV_64FC1), camera_rvecs[0]);
	camera_tvecs[0] = Vec3d(0, 0, 0);

	for (int i = 1; i < CAMERAS; i++)
	{
		solvePnP(
			object_points[i], image_points[i],
			camera_intrinsics_map[serial_numbers[i]], dist_coeffs_map[serial_numbers[i]],
			camera_rvecs[i], camera_tvecs[i]);

		Mat camera_rot;
		Rodrigues(camera_rvecs[i], camera_rot);
		cout << "R" << endl << camera_rot << endl;
		cout << "T" << endl << camera_tvecs[i] << endl;
	}
	
	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}