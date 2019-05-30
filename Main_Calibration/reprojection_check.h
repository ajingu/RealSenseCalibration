#pragma once
#pragma warning(disable: 4996)

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "my_const.h"

using namespace std;
using namespace cv;

namespace RSCalibration
{
	static class ReprojectionCheck
	{
	private:

	public:
		static void Reproject(vector<map<string, Mat>>& images, vector<vector<vector<Point2f>>>& image_points_per_time, map<string, Mat>& camera_intrinsics_map, map<string, Mat>& dist_coeffs_map);
	};
}