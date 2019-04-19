#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

int main()
{
	Mat markerImage;
	int id = 2;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	aruco::drawMarker(dictionary, id, 200, markerImage, 1);

	namedWindow("Display", WINDOW_AUTOSIZE);
	imshow("Display", markerImage);

	string file_name = "../Common/Image/4X4_100_" + to_string(id) + ".png";
	imwrite(file_name, markerImage);

	cv::waitKey(0);
	
	return 0;
}