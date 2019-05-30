#pragma once
#pragma warning(disable: 4996)

#include <iostream>
#include <opencv2/opencv.hpp>

#include "my_const.h"
#include "my_struct.h"

using namespace std;
using namespace cv;

namespace RSCalibration
{
	static class IO
	{
	public:
		static void GetIntrinsics(map<string, Mat>& camera_intrinsics_map, map<string, Mat>& dist_coeffs_map);
		static void GetMarkerGeometry(map<int, Transform>& marker_transforms);
	};
}