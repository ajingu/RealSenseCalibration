#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

int main()
{
	vector<int> ids = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

	for (int i = 0; i < ids.size(); i++)
	{
		int id = ids[i];

		Mat markerImage;
		Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
		aruco::drawMarker(dictionary, id, 200, markerImage, 1);

		imshow("Marker " + to_string(id), markerImage);

		string file_name = "../Common/Image/Marker/4X4_100_" + to_string(id) + ".png";
		imwrite(file_name, markerImage);
	}

	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}