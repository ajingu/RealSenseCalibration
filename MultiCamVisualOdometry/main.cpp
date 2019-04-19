#include <iostream>

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
	bool is_source = true;


	for_each(begin(serial_numbers), end(serial_numbers), [&dictionary, &points_src, &points_dst, &is_source](string sn)
	{
		string file_name = "../Common/Image/Camera/" + sn + ".png";
		Mat image = imread(file_name);
		Mat image_copy;
		image.copyTo(image_copy);
		vector<int> ids;
		vector<vector<Point2f>> corners;

		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		if (ids.size() == 2)
		{
			aruco::drawDetectedMarkers(image_copy, corners, ids);

			for (int i=0; i<ids.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (is_source)
					{
						points_src.push_back(corners[i][j]);
					}
					else
					{
						points_dst.push_back(corners[i][j]);
					}
				}

			}

			is_source = false;
		}

		imshow(sn, image_copy);
	});

	Mat F = findFundamentalMat(points_src, points_dst);

	cout << "F: " << endl << F << endl;

	


	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}
