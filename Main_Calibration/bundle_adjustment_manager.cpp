#include "bundle_adjustment_manager.h"

namespace RSCalibration
{
	BAManager::BAManager(const map<string, Mat>& camera_intrinsics_map, const map<string, Mat>& dist_coeffs_map)
		:camera_intrinsics_map(camera_intrinsics_map), dist_coeffs_map(dist_coeffs_map)
	{
		if (!bal_problem.loadFile("../Common/Correspondence/hongo/correspondence.txt"))
		{
			cerr << "unable to open correspondence file " << "\n";
			system("PAUSE");
			exit(1);
		}
	}

	void BAManager::StartBA()
	{
		const double* const bal_observations = bal_problem.observations();
		Problem problem;

		for (int i = 0; i < bal_problem.num_observations(); i++)
		{
			int camera_idx = bal_problem.camera_idx(i);
			int marker_idx = bal_problem.marker_idx(i);

			if (camera_idx == 0)
			{
				if (marker_idx == 0)
				{
					CostFunction* cost_function =
						BaseCameraBaseMarkerReprojectionError::create(
							bal_observations + 8 * i,
							MARKER_SIDE,
							camera_intrinsics_map[SERIAL_NUMBERS[0]],
							dist_coeffs_map[SERIAL_NUMBERS[0]]);

					problem.AddResidualBlock(cost_function,
						NULL,
						bal_problem.mutable_base_marker_transform_from_base_camera(i));
				}
				else
				{
					CostFunction* cost_function =
						BaseCameraReprojectionError::create(
							bal_observations + 8 * i,
							MARKER_SIDE,
							camera_intrinsics_map[SERIAL_NUMBERS[0]],
							dist_coeffs_map[SERIAL_NUMBERS[0]]);

					problem.AddResidualBlock(cost_function,
						NULL,
						bal_problem.mutable_base_marker_transform_from_base_camera(i),
						bal_problem.mutable_marker_transform_from_base_marker(i));
				}
			}
			else
			{
				if (marker_idx == 0)
				{
					CostFunction* cost_function =
						TargetCameraBaseMarkerReprojectionError::create(
							bal_observations + 8 * i,
							MARKER_SIDE,
							camera_intrinsics_map[SERIAL_NUMBERS[camera_idx]],
							dist_coeffs_map[SERIAL_NUMBERS[camera_idx]]);

					problem.AddResidualBlock(cost_function,
						NULL,
						bal_problem.mutable_camera_transform_from_base_camera(i),
						bal_problem.mutable_base_marker_transform_from_base_camera(i));
				}
				else
				{
					CostFunction* cost_function =
						TargetCameraReprojectionError::create(
							bal_observations + 8 * i,
							MARKER_SIDE,
							camera_intrinsics_map[SERIAL_NUMBERS[camera_idx]],
							dist_coeffs_map[SERIAL_NUMBERS[camera_idx]]);

					problem.AddResidualBlock(cost_function,
						NULL,
						bal_problem.mutable_camera_transform_from_base_camera(i),
						bal_problem.mutable_base_marker_transform_from_base_camera(i),
						bal_problem.mutable_marker_transform_from_base_marker(i));
				}
			}
		}

		Solver::Options options;
		options.linear_solver_type = DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		Solver::Summary summary;
		Solve(options, &problem, &summary);
		cout << summary.FullReport() << endl;
	}

	void BAManager::Write()
	{
		cout << "Marker Transform" << endl;
		for (int marker_idx = 0; marker_idx < MARKERS; marker_idx++)
		{
			double* marker_transform = bal_problem.marker_transform(marker_idx);
			cout << marker_idx << " ";
			cout << "Rvec: " << marker_transform[0] << " " << marker_transform[1] << " " << marker_transform[2] << " ";
			cout << "tvec: " << marker_transform[3] << " " << marker_transform[4] << " " << marker_transform[5] << endl;
		}
		FileStorage fs("../Common/Correspondence/hongo/Camera_Transform.xml", FileStorage::WRITE);
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
			cout << camera_rot << endl;
			cout << "t:" << endl;
			cout << camera_tvec << endl;

			//base camera transform on target camera coordinate(Reprojection Input, BA Input)
			fs << "R" + to_string(i) << camera_rot;
			fs << "t" + to_string(i) << camera_tvec;

			//output for hongo
			//target camera transform on base camera coordinate(Hongo Input)
			Mat camera_rot_t, tvec_inv;
			camera_rot_t = camera_rot.t();
			tvec_inv = -camera_rot_t * camera_tvec;

			ofstream f_hongo;
			f_hongo.open("../Common/Calibration/Extrinsics/mat" + to_string(i) + ".txt");
			for (int row = 0; row < 3; row++)
			{
				f_hongo << camera_rot_t.at<double>(row, 0) << endl;
				f_hongo << camera_rot_t.at<double>(row, 1) << endl;
				f_hongo << camera_rot_t.at<double>(row, 2) << endl;
				f_hongo << tvec_inv.at<double>(row, 0) << endl;
			}

			f_hongo.close();
		}

		fs.release();

		vector<Point3d> object_points;
		bal_problem.getPoint3dCoordinates(object_points);

		ofstream fout;
		fout.open("../Common/Correspondence/hongo/point3d.txt");
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
	}
}