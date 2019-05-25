#pragma warning(disable: 4996)

#include <iostream>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

#define MARKER_SIDE 0.015
#define TIMES 6
#define CAMERAS 4
#define MARKERS 11
#define BASE_MARKER_ID 0

struct Observation
{
	int marker_id;
	vector<Point2f> points;
};

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
			cerr << "unable to open intrinsics file." << endl;
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
	}
}

void getMarkerGeometry(const char* file_path, map<int, Transform>& marker_transforms)
{
	FILE* fptr = fopen(file_path, "r");
	if (fptr == NULL)
	{
		cerr << "ERROR: unable to open geometry file " << "\n";
		system("PAUSE");
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
	string serial_numbers[CAMERAS] = { "821312061029", "816612062327", "821212062536", "821212061326" };
	int marker_ids[MARKERS] = { 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 23 };
	map<int, int> marker_idx_map;
	for (int i = 0; i < MARKERS; i++)
	{
		marker_idx_map[marker_ids[i]] = i;
	}
	
	//intrinsics
	map<string, Mat> camera_intrinsics_map;
	map<string, Mat> dist_coeffs_map;
	getIntrinsics(serial_numbers, camera_intrinsics_map, dist_coeffs_map);

	//geometry
	const char* geometry_file_path = "../Common/Correspondence/hongo/marker_geometry.txt";
	map<int, Transform> marker_transforms_from_base;
	getMarkerGeometry(geometry_file_path, marker_transforms_from_base);

	//get marker points
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	map<string, Mat> images[TIMES];
	map<int, Transform> marker_transforms_from_camera;
	Vec3d base_rvecs[TIMES], base_tvecs[TIMES];
	vector<Point3d> object_points[CAMERAS];
	vector<Point2f> image_points[CAMERAS];
	vector<Observation> observations[TIMES][CAMERAS];

	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			string sn = serial_numbers[camera_idx];

			string file_name = "../Common/Image/IR/hongo/" + to_string(time_idx) + (string)"/" + sn + ".png";
			Mat image = imread(file_name);
			if (image.empty())
			{
				cout << file_name << endl;
				cerr << "Image is empty." << endl;
				system("PAUSE");
				exit(-1);
			}
			images[time_idx][sn] = image;
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
			/*
			for (int i = 0; i < ids.size(); i++)
			{
				aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.01);
			}
			imshow(sn + (string)" " + to_string(time_idx), image_copy);*/
	        
			//sort ids
			vector<int> indices(ids.size());
			iota(indices.begin(), indices.end(), 0);
			sort(indices.begin(), indices.end(), [&ids](int i1, int i2)
			{
				return ids[i1] < ids[i2];
			});

			//Define Marker Transform from Camera 
			if (camera_idx == 0)
			{
				int tmp_marker_id = ids[indices[0]];
				Vec3d tmp_rvec_from_camera = rvecs[indices[0]];
				Vec3d tmp_tvec_from_camera = tvecs[indices[0]];

				if (tmp_marker_id == BASE_MARKER_ID)
				{
					base_rvecs[time_idx] = tmp_rvec_from_camera;
					base_tvecs[time_idx] = tmp_tvec_from_camera;

					cout << "BASE_RVEC: " << base_rvecs[time_idx] << " BASE_TVEC: " << base_tvecs[time_idx] << endl;
				}
				else
				{
					Transform tmp_marker_transform_from_base = marker_transforms_from_base[tmp_marker_id];
					Vec3d tmp_rvec_from_base = tmp_marker_transform_from_base.rvec;
					Vec3d tmp_tvec_from_base = tmp_marker_transform_from_base.tvec;

					Mat tmp_rot_from_camera,tmp_rot_from_base, tmp_t_mat;
					Rodrigues(tmp_rvec_from_camera, tmp_rot_from_camera);
					Rodrigues(tmp_rvec_from_base, tmp_rot_from_base);

					Rodrigues(tmp_rot_from_camera * tmp_rot_from_base.t(), base_rvecs[time_idx]);
					tmp_t_mat = tmp_rot_from_camera * tmp_rot_from_base.t() * Mat(-tmp_tvec_from_base) + Mat(tmp_tvec_from_camera);
					base_tvecs[time_idx] = tmp_t_mat;

					cout << "MARKER " << BASE_MARKER_ID << "(BASE) RVEC: " << base_rvecs[time_idx] << " TVEC: " << base_tvecs[time_idx] << endl;
				}
				
				//Define All Marker Transform
				marker_transforms_from_camera[marker_ids[0]] = Transform{ base_rvecs[time_idx], base_tvecs[time_idx] };
				for (int i = 1; i < MARKERS; i++)
				{
					Transform marker_transform_from_base = marker_transforms_from_base[marker_ids[i]];
					Vec3d rvec_from_base = marker_transform_from_base.rvec;
					Vec3d tvec_from_base = marker_transform_from_base.tvec;
					
					Mat rot_from_base, base_rot_from_camera, t_mat;
				    Vec3d rvec_from_camera, tvec_from_camera;
					Rodrigues(rvec_from_base, rot_from_base);
					Rodrigues(base_rvecs[time_idx], base_rot_from_camera);

					Rodrigues(base_rot_from_camera * rot_from_base, rvec_from_camera);
					t_mat = base_rot_from_camera * Mat(tvec_from_base) + Mat(base_tvecs[time_idx]);
					tvec_from_camera = t_mat;
					marker_transforms_from_camera[marker_ids[i]] = Transform{ rvec_from_camera, tvec_from_camera };

					cout << "MARKER " << marker_ids[i] << " RVEC: " << rvec_from_camera << " TVEC: " << tvec_from_camera << endl;
				}
			}

			cout << endl;

			for (int i = 0; i < ids.size(); i++)
			{
				Observation observation;
				observation.marker_id = ids[indices[i]];

				for (int j = 0; j < 4; j++)
				{
					image_points[camera_idx].emplace_back(corners[indices[i]][j]);
					observation.points.emplace_back(corners[indices[i]][j]);
				}

				observations[time_idx][camera_idx].emplace_back(observation);

				Transform marker_transform = marker_transforms_from_camera[ids[indices[i]]];
				vector<Point3d> corners3D = getCornersInCameraWorld(MARKER_SIDE, marker_transform.rvec, marker_transform.tvec);
				object_points[camera_idx].insert(object_points[camera_idx].end(), corners3D.begin(), corners3D.end());

				cout << "Time: " << time_idx << ", CAMERA: " << camera_idx << ", MARKER_ID: " << ids[indices[i]] << endl;
			}
		}
	}

	//Calculate Default Camera Transform from Base Camera
	vector<Vec3d> camera_rvecs(CAMERAS), camera_tvecs(CAMERAS);
	Rodrigues(Mat::eye(3, 3, CV_64FC1), camera_rvecs[0]);
	camera_tvecs[0] = Vec3d(0, 0, 0);

	for (int i = 1; i < CAMERAS; i++)
	{
		if (image_points[i].size() < 4)
		{
			cerr << "The correspondence points are too few." << endl ;
			system("PAUSE");
			exit(-1);
		}

		solvePnP(
			object_points[i], image_points[i],
			camera_intrinsics_map[serial_numbers[i]], dist_coeffs_map[serial_numbers[i]],
			camera_rvecs[i], camera_tvecs[i]);

		Mat camera_rot;
		Rodrigues(camera_rvecs[i], camera_rot);
		cout << "R" << to_string(i) << endl << camera_rot << endl;
		cout << "T" << to_string(i) << endl << camera_tvecs[i] << endl;

		
	}

	//output
	ofstream fout;
	fout.open("../Common/Correspondence/hongo/correspondence.txt");
	fout << TIMES << " " << CAMERAS  << " " << MARKERS;
	
	int observations_num = 0;

	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			observations_num += observations[time_idx][camera_idx].size();
		}
	}
	fout << " " << observations_num << endl;

	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		fout << time_idx;

		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			fout << " " << observations[time_idx][camera_idx].size();
		}

		fout << endl;
	}
	
	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			for (int observation_idx = 0; observation_idx < observations[time_idx][camera_idx].size(); observation_idx++)
			{
				Observation observation = observations[time_idx][camera_idx][observation_idx];
				fout << time_idx << " " << camera_idx << " " << marker_idx_map[observation.marker_id];

				for (int i = 0; i < 4; i++)
				{
					Point2f point = observation.points[i];
					fout << " " << point.x << " " << point.y;
				}

				fout << endl;
			}
		}
	}
	for (int i = 0; i < CAMERAS; i++)
	{
		Vec3d camera_rvec = camera_rvecs[i];
		Vec3d camera_tvec = camera_tvecs[i];

		fout << camera_rvec[0] << " " << camera_rvec[1] << " " << camera_rvec[2] << " ";
		fout << camera_tvec[0] << " " << camera_tvec[1] << " " << camera_tvec[2] << endl;
	}
	for (int i = 0; i < TIMES; i++)
	{
		Vec3d base_rvec = base_rvecs[i];
		Vec3d base_tvec = base_tvecs[i];

		fout << base_rvec[0] << " " << base_rvec[1] << " " << base_rvec[2] << " ";
		fout << base_tvec[0] << " " << base_tvec[1] << " " << base_tvec[2] << endl;
	}
	for (int i = 0; i < MARKERS; i++)
	{
		Vec3d rvec_from_base =  marker_transforms_from_base[marker_ids[i]].rvec;
		Vec3d tvec_from_base =  marker_transforms_from_base[marker_ids[i]].tvec;

		fout << rvec_from_base[0] << " " << rvec_from_base[1] << " " << rvec_from_base[2] << " ";
		fout << tvec_from_base[0] << " " << tvec_from_base[1] << " " << tvec_from_base[2] << endl;
	}

	fout.close();


	//Reprojection Check
	for (int time_idx = 0; time_idx < TIMES; time_idx++)
	{
		for (int camera_idx = 0; camera_idx < CAMERAS; camera_idx++)
		{
			int last_offset = 0; 
			for (int i = 0; i < time_idx; i++)
			{
				last_offset += observations[i][camera_idx].size() * 4;
			}
			int current_offset = observations[time_idx][camera_idx].size()*4;
			
			vector<Point3d> object_points_per_time(observations[time_idx][camera_idx].size()*4);
			copy(object_points[camera_idx].begin()+last_offset, object_points[camera_idx].begin()+last_offset+current_offset, object_points_per_time.begin());

			vector<Point2d> image_points_per_time(observations[time_idx][camera_idx].size()*4);
			copy(image_points[camera_idx].begin()+last_offset, image_points[camera_idx].begin()+last_offset+current_offset, image_points_per_time.begin());

			vector<Point2d> reprojected_points;
			for (int i = 0; i < reprojected_points.size(); i++)
			{
				Point2d point = reprojected_points[i];
				cout << i << " x: " << point.x << " y: " << point.y << endl;
			}
			projectPoints(object_points_per_time, camera_rvecs[camera_idx], camera_tvecs[camera_idx], camera_intrinsics_map[serial_numbers[camera_idx]], dist_coeffs_map[serial_numbers[camera_idx]], reprojected_points);

			Mat reprojection_image = images[time_idx][serial_numbers[camera_idx]];
			for (int point_index = 0; point_index < reprojected_points.size(); point_index++)
			{
				drawMarker(reprojection_image, image_points_per_time[point_index], Scalar(255, 0, 0), MARKER_CROSS, 10, 2);
				drawMarker(reprojection_image, reprojected_points[point_index], Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
			}

			putText(reprojection_image, "BLUE : Marker Points (t+1)", Point{ 10,20 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
			putText(reprojection_image, "GREEN : Reprojected Points (t -> t+1)", Point{ 10,45 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

			imshow("Reprojection " + serial_numbers[camera_idx] + " " + to_string(time_idx), reprojection_image);
		}
	}
	
	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}