#include "reprojection_check.h"

namespace RSCalibration
{
	void ReprojectionCheck::Reproject(vector<map<string, Mat>>& images, vector<vector<vector<Point2f>>>& image_points_per_time, map<string, Mat>& camera_intrinsics_map, map<string, Mat>& dist_coeffs_map)
	{
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

		FileStorage fs2("../Common/Correspondence/hongo/Camera_Transform.xml", FileStorage::READ);
		if (!fs2.isOpened())
		{
			cerr << "unable to open Camera_Transform.xml" << endl;
			system("PAUSE");
			exit(1);
		}

		Mat camera_rvec, camera_rot, camera_tvec;
		vector<Point3d> object_points;

		//Reduce Images
		num_times = num_times > 3 ? 3 : num_times;

		for (int time_idx = 0; time_idx < num_times; time_idx++)
		{
			for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++)
			{
				object_points.clear();

				if (num_points_per_time_camera[time_idx][camera_idx] == 0) continue;

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
				projectPoints(object_points, camera_rvec, camera_tvec, camera_intrinsics_map[SERIAL_NUMBERS[camera_idx]], dist_coeffs_map[SERIAL_NUMBERS[camera_idx]], reprojected_points);

				//visualization
				Mat reprojection_image = images[time_idx][SERIAL_NUMBERS[camera_idx]];
				for (int i = 0; i < reprojected_points.size(); i++)
				{
					drawMarker(reprojection_image, image_points_per_time[time_idx][camera_idx][i], Scalar(255, 0, 0), MARKER_CROSS, 10, 2);
					drawMarker(reprojection_image, reprojected_points[i], Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
				}

				putText(reprojection_image, "BLUE : Marker Points (t+1)", Point{ 10,20 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
				putText(reprojection_image, "GREEN : Reprojected Points (t -> t+1)", Point{ 10,45 }, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

				imshow("Reprojection " + SERIAL_NUMBERS[camera_idx] + " " + to_string(time_idx), reprojection_image);
			}
		}

		fclose(fptr);
		fs2.release();
	}
}