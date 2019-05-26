#pragma once

#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "my_const.h"
#include "my_struct.h"

using namespace std;
using namespace cv;


namespace RSCalibration
{
	class Correspondencer
	{
	private:
		Ptr<aruco::Dictionary> dictionary;
		map<string, Mat> camera_intrinsics_map, dist_coeffs_map;
		map<int, int> marker_idx_map;
		vector<Point3d> GetCornersInCameraWorld(Vec3d rvec, Vec3d tvec);

	public:
		Correspondencer(const map<string, Mat>& camera_intrinsics_map, const map<string, Mat>& dist_coeffs_map);
		void GetCorrespondencePoints(vector<map<string, Mat>>& images, vector<vector<Point3d>>& object_points, vector<vector<Point2f>>& image_points, vector<vector<vector<Point2f>>>& image_points_per_time, vector<vector<vector<Observation>>>& observations, map<int, Transform>& marker_transforms_from_camera, map<int, Transform>& marker_transforms_from_base, vector<Vec3d>& base_rvecs, vector<Vec3d>& base_tvecs);
		void CalculateTransforms(vector<vector<Point3d>>& object_points, vector<vector<Point2f>>& image_points, vector<Vec3d>& camera_rvecs, vector<Vec3d>& camera_tvecs);
		void Write(vector<vector<vector<Observation>>>& observations, vector<Vec3d>& camera_rvecs, vector<Vec3d>& camera_tvecs, vector<Vec3d>& base_rvecs, vector<Vec3d>& base_tvecs, map<int, Transform>& marker_transforms_from_base);
		void ReprojectionCheck(vector<map<string, Mat>>& images, vector<vector<Point3d>>& object_points, vector<vector<Point2f>>& image_points, vector<vector<vector<Observation>>>& observations, vector<Vec3d>& camera_rvecs, vector<Vec3d>& camera_tvecs);
	};
}