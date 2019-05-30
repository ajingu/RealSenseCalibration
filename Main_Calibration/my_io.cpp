#include "my_io.h"

namespace RSCalibration
{
	void IO::GetIntrinsics(map<string, Mat>& camera_intrinsics_map, map<string, Mat>& dist_coeffs_map)
	{
		for (int i = 0; i < CAMERAS; i++)
		{
			string sn = SERIAL_NUMBERS[i];
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

	void IO::GetMarkerGeometry(map<int, Transform>& marker_transforms)
	{
		const char* file_path = "../Common/Correspondence/hongo/marker_geometry.txt";

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
		double rvec_x, rvec_y, rvec_z, tvec_x, tvec_y, tvec_z;
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
}