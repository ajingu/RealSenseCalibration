#pragma warning(disable: 4996)

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	FILE* f_rot = fopen("../Common/Correspondence/hongo/marker_geometry_rot.txt", "r");
	if (f_rot == NULL)
	{
		cerr << "ERROR: unable to open geometry rot file " << "\n";
		system("PAUSE");
		exit(1);
	};

	ofstream fout;
    fout.open("../Common/Correspondence/hongo/marker_geometry.txt");
	

	int num_markers;
	fscanf(f_rot, "%d", &num_markers);
	fout << num_markers << endl;

	int marker_id;
	double rot_values[9];
	double tvec_x, tvec_y, tvec_z;
	for (int i = 0; i < num_markers; i++)
	{
		fscanf(f_rot, "%d", &marker_id);
		fout << marker_id << " ";

		fscanf(f_rot, "%lf", &rot_values[0]);
		fscanf(f_rot, "%lf", &rot_values[1]);
		fscanf(f_rot, "%lf", &rot_values[2]);
		fscanf(f_rot, "%lf", &rot_values[3]);
		fscanf(f_rot, "%lf", &rot_values[4]);
		fscanf(f_rot, "%lf", &rot_values[5]);
		fscanf(f_rot, "%lf", &rot_values[6]);
		fscanf(f_rot, "%lf", &rot_values[7]);
		fscanf(f_rot, "%lf", &rot_values[8]);
		Mat rot(3, 3, CV_64FC1), rvec;
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
			{
				rot.at<double>(row, col) = rot_values[row * 3 + col];
			}
		}
		Rodrigues(rot, rvec);
		fout << rvec.at<double>(0, 0) << " " << rvec.at<double>(1, 0) << " " << rvec.at<double>(2, 0) << " ";
		

		fscanf(f_rot, "%lf", &tvec_x);
		fscanf(f_rot, "%lf", &tvec_y);
		fscanf(f_rot, "%lf", &tvec_z);
		fout << tvec_x/100 << " " << tvec_y/100 << " " << tvec_z/100 << endl;
	}

	fclose(f_rot);
	fout.close();

	return 0;
}