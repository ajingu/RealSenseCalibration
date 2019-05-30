#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace RSCalibration
{
	struct Observation
	{
		int marker_id;
		vector<Point2f> points;
	};

	struct Transform
	{
		Vec3d rvec, tvec;
	};
}