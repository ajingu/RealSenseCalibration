#include <algorithm>
#include <iostream>
#include <numeric>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;


int main()
{
	//get intrinsics
	string serial_numbers[2] = { "819612072493", "825312072048" };
	map<string, Mat> intrinsics_map;
	
	for_each(begin(serial_numbers), end(serial_numbers), [&intrinsics_map](string sn) 
	{
		string file_name = "../Common/Calibration/Intrinsics/" + sn + ".xml";
		FileStorage fs(file_name, FileStorage::READ);
		if(!fs.isOpened()) 
		{
			std::cout << "File can not be opened." << std::endl;
			return -1;
		}

		Mat A;
		fs["intrinsics"] >> A;
		intrinsics_map[sn] = A;

		cout << "Serial Number: " << sn << endl;
		cout << "Intrinsics: " << endl << A << endl << endl;

		
		fs.release();
	});


	//get marker points
	Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	vector<Point2f> points_src, points_dst;


	for_each(begin(serial_numbers), end(serial_numbers), [&dictionary, &points_src, &points_dst, &serial_numbers](string sn)
	{
		string file_name = "../Common/Image/IR/" + sn + ".png";
		Mat image = imread(file_name);
		Mat image_copy;
		image.copyTo(image_copy);
		vector<int> ids;
		vector<vector<Point2f>> corners;

		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		if (ids.size() > 0)
		{
			//sort ids
			vector<int> indices(ids.size());
			iota(indices.begin(), indices.end(), 0);

			sort(indices.begin(), indices.end(), [&ids](int i1, int i2)
			{
				return ids[i1] < ids[i2];
			});
			/*
			cout << sn << "[id]: ";
			for (int i = 0; i < ids.size(); i++)
			{
				cout << ids[indices[i]] << ", ";
			}
			cout << endl;
			*/
			aruco::drawDetectedMarkers(image_copy, corners, ids);
			imshow(sn, image_copy);

			for (int i=0; i<ids.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (sn == serial_numbers[0])
					{
						points_src.push_back(corners[indices[i]][j]);
					}
					else
					{
						points_dst.push_back(corners[indices[i]][j]);
					}
				}

			}
		}
	});

	cout << "src size: " << points_src.size() << endl << "dst size: " << points_dst.size() << endl;

	Mat F = findFundamentalMat(points_src, points_dst);

	cout << "F: " << endl << F << endl;

	Mat A_src = intrinsics_map[serial_numbers[0]];
	Mat A_dst = intrinsics_map[serial_numbers[1]];

	//Mat E = A_src.t() * F * A_dst;
	Mat mask;
	Mat E = findEssentialMat(points_dst, points_src, A_src, RANSAC, 0.999, 1.0, mask);
	cout << "E: " << endl << E << endl;

	Mat R, t;
	recoverPose(E, points_dst, points_src, A_src, R, t, mask);
	cout << "R:" << endl << R << endl;
	cout << "t:" << endl << t << endl;
	/*
	Mat R1, R2, t;
	decomposeEssentialMat(E, R1, R2, t);
	cout << "R1:" << endl << R1 << endl;
	cout << "R2:" << endl << R2 << endl;
	cout << "t:" << endl << t << endl;
	*/

	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}
